/*
This is a simple program for 2 ESP32 boards that passes UART messages between them.
LoRA modules are used to transmit the messages wirelessly between the two ESP32 boards.
IMPORTANT: the receiver_address variable must be set to match the ADDRESS of the LoRA device.
    This is the address assigned to the LoRA device using the AT command AT+ADDRESS=
    Alternatively, you can set receiver_address = 0. This will send messages to all LoRA devices.

Here is the wiring diagram showing the connections between the ESP32 board and the LoRA module.

	ESP32            LoRA Module
    ----------------------------
	GPIO17 (TX) ---> RX pin
	GPIO16 (RX) <--- TX pin
    GPIO4 (RST) ---> RST pin
	GND <----------> GND
*/

#include <stdio.h>
#include <string.h>
#include <math.h>  // Include math.h for log10 function
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"
#include <stdlib.h>  // Include for rand() and srand()
#include <time.h>    // Include for time()

#define UART_NUM UART_NUM_2
#define TXD_PIN GPIO_NUM_17  // TX pin connected to LoRa RX pin
#define RXD_PIN GPIO_NUM_16  // RX pin connected to LoRa TX pin
#define RST_PIN GPIO_NUM_4    // RST pin connected to LoRa RST pin
#define UART_BAUD_RATE 9600   // Set the UART baud rate to 9600 for LoRa device
#define BUF_SIZE 1024

// ================================================================================
static const int receiver_address = 0;  // Set the receiver address (ID)
// ================================================================================

static const char *TAG = "uart_example";
static char local_lora_device_address[6];  // Buffer to hold the LoRa device address as a string
static char lora_uid[30];        // Buffer to hold the LoRa UID
static int message_counter = 0;  // Counter for incrementing message number

// Function to get the LoRa device address
void get_lora_device_address() {
    char response[50];  // Buffer to hold the response from the LoRa module
    // Send the command to get the LoRa device address
    uart_write_bytes(UART_NUM, "AT+ADDRESS?\r\n", strlen("AT+ADDRESS?\r\n"));
    
    // Wait for the response (you may need to adjust the timeout)
    int len = uart_read_bytes(UART_NUM, response, sizeof(response) - 1, pdMS_TO_TICKS(1000));
    if (len > 0) {
        response[len] = '\0';  // Null terminate the received data
        ESP_LOGI(TAG, "Response received: %s", response);

        // Parse the address from the response
        char *address_start = strstr(response, "+ADDRESS=");
        if (address_start) {
            address_start += 9;  // Move past "+ADDRESS="
            sscanf(address_start, "%s", local_lora_device_address);  // Extract the LoRa device address
            ESP_LOGI(TAG, "Local LoRa device address extracted: %s", local_lora_device_address);  // Log the address
        } else {
            ESP_LOGE(TAG, "LoRa device address not found in response.");
        }
    } else {
        ESP_LOGE(TAG, "No response received for address command.");
    }
}

// Function to get the UID from the LoRa module
void get_uid_from_lora() {
    char response[50];  // Buffer to hold the response from the LoRa module
    // Send the command to get the UID
    uart_write_bytes(UART_NUM, "AT+UID?\r\n", strlen("AT+UID?\r\n"));
    
    // Wait for the response (you may need to adjust the timeout)
    int len = uart_read_bytes(UART_NUM, response, sizeof(response) - 1, pdMS_TO_TICKS(1000));
    if (len > 0) {
        response[len] = '\0';  // Null terminate the received data
        ESP_LOGI(TAG, "Response received: %s", response);

        // Parse the UID from the response
        char *uid_start = strstr(response, "+UID=");
        if (uid_start) {
            uid_start += 5;  // Move past "+UID="
            sscanf(uid_start, "%s", lora_uid);  // Extract the LoRa UID
            ESP_LOGI(TAG, "LoRa UID extracted: %s", lora_uid);  // Log the LoRa UID
        } else {
            ESP_LOGE(TAG, "LoRa UID not found in response.");
        }
    } else {
        ESP_LOGE(TAG, "No response received for UID command.");
    }
}

// Initialize UART
void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,  // Use the defined baud rate
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0));
    
    ESP_LOGI(TAG, "UART initialized successfully");
}

// Task to handle UART receiving
void uart_rx_task(void *arg) {
    uint8_t* rx_buffer = (uint8_t*) malloc(BUF_SIZE);
    int len;

    while (1) {
        // Read data from UART
        len = uart_read_bytes(UART_NUM, rx_buffer, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            rx_buffer[len] = 0;  // Null terminate the received data
            ESP_LOGI(TAG, "Received: %s", rx_buffer);

            // Check for specific responses
            if (strstr((char*)rx_buffer, "+OK") != NULL) {
                // Handle +OK response if needed
                ESP_LOGI(TAG, "Transmission successful.");
            } else if (strstr((char*)rx_buffer, "+RCV") != NULL) {
                // Handle incoming message from remote LoRa module
                ESP_LOGI(TAG, "Remote message received: %s", rx_buffer);
            } else {
                // Handle other responses or errors if necessary
                ESP_LOGE(TAG, "Unexpected response: %s", rx_buffer);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Small delay to prevent busy-waiting
    }
    free(rx_buffer);
}

// Task to handle LoRa transmitting with random backoff
void uart_tx_task(void *arg) {
    char tx_buffer[BUF_SIZE];

    while (1) {
        message_counter++;  // Increment the message counter
        
        // Construct the message as "HELLO-sequence_number"
        snprintf(tx_buffer, BUF_SIZE, "AT+SEND=%d,%d,HELLO-%d\r\n", receiver_address, 
                 strlen("HELLO") + 1 + (message_counter < 10 ? 1 : (message_counter < 100 ? 2 : 3)), message_counter);
        
        // Random backoff time between 0 and 1000 milliseconds
        int backoff_time = rand() % 1000;  // Random delay
        vTaskDelay(pdMS_TO_TICKS(backoff_time));  // Wait for the random backoff time

        // Send the command to the LoRa module
        uart_write_bytes(UART_NUM, tx_buffer, strlen(tx_buffer));
        ESP_LOGI(TAG, "Send command sent: %s", tx_buffer);
        
        // Wait for a short period to allow for any incoming messages
        vTaskDelay(pdMS_TO_TICKS(100));  // Wait for 100 ms after sending

        vTaskDelay(pdMS_TO_TICKS(2000));  // Delay for 2 seconds between messages
    }
}

void app_main(void) {
    // Get the LoRa device address first
    ESP_LOGI(TAG, "==================================================");

    // Then initialize UART and tasks
    ESP_LOGI(TAG, "Initializing the UART");
    uart_init();
    ESP_LOGI(TAG, "UART Initialized");

    ESP_LOGI(TAG, "Getting local LoRa device address");
    get_lora_device_address();
    ESP_LOGI(TAG, "Local LoRa Device Address: %s", local_lora_device_address);

    // Example of retry logic for getting the UID
    ESP_LOGI(TAG, "Getting UID from the local LoRa device");
    int retries = 3;
    while (retries > 0) {
        get_uid_from_lora();
        if (strlen(lora_uid) > 0) {  // Check if UID was successfully retrieved
            break;
        }
        retries--;
        vTaskDelay(pdMS_TO_TICKS(100));  // Wait before retrying
    }

    // Log the LoRa UID and local LoRa device address with framing
    ESP_LOGI(TAG, "Local LoRa Device UID: %s", lora_uid);

    vTaskDelay(pdMS_TO_TICKS(200));  // Wait for 100 ms after UART initialization

    ESP_LOGI(TAG, "Starting Rx and Tx tasks");

    xTaskCreate(uart_rx_task, "uart_rx_task", 2048, NULL, 5, NULL);
    xTaskCreate(uart_tx_task, "uart_tx_task", 2048, NULL, 4, NULL);

    ESP_LOGI(TAG, "Tasks started");
    ESP_LOGI(TAG, "==================================================");
    vTaskDelay(pdMS_TO_TICKS(2000));  // Wait for 100 ms after UART initialization

}