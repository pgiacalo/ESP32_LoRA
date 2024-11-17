/*
This is a simple program for 2 ESP32 boards that passes UART messages between them.
For message transport, LoRA modules are used.
Here is the wiring diagram between the ESP32 and the LoRA module.

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

#define UART_NUM UART_NUM_2
#define TXD_PIN GPIO_NUM_17  // TX pin connected to LoRa RX pin
#define RXD_PIN GPIO_NUM_16  // RX pin connected to LoRa TX pin
#define RST_PIN GPIO_NUM_4    // RST pin connected to LoRa RST pin
#define UART_BAUD_RATE 9600   // Set the UART baud rate to 9600 for LoRa device
#define BUF_SIZE 1024

// ================================================================================
static const int receiver_address = 2;  // Set the receiver address (ID)
// ================================================================================

static const char *TAG = "uart_example";
static char device_id[9];  // Store chip ID as string (8 chars + null terminator)
static int message_counter = 0;  // Counter for incrementing message number

// Generate unique device ID once at startup
void init_device_id(void) {
    uint32_t id = esp_random();  // Get random number based on hardware
    sprintf(device_id, "%08lX", (unsigned long)(id & 0xFFFFFFFF));
    ESP_LOGI(TAG, "Device initialized with ID: %s", device_id);
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

// Task to handle LoRa transmitting
void uart_tx_task(void *arg) {
    char tx_buffer[BUF_SIZE];

    while (1) {
        message_counter++;  // Increment the message counter
        
        // Calculate the length of the message correctly
        int message_length = strlen(device_id) + 1 + (message_counter < 10 ? 1 : (message_counter < 100 ? 2 : 3)); // +1 for hyphen
        
        snprintf(tx_buffer, BUF_SIZE, "AT+SEND=%d,%d,%s-%d\r\n", receiver_address, 
                 message_length, device_id, message_counter);
        
        // Send the command to the LoRa module
        uart_write_bytes(UART_NUM, tx_buffer, strlen(tx_buffer));
        ESP_LOGI(TAG, "Send command sent: %s", tx_buffer);
        
        // Wait for a short period to allow for response processing
        vTaskDelay(pdMS_TO_TICKS(2000));  // Delay for 2 seconds between messages
    }
}

void app_main(void) {
    // Generate unique ID first
    init_device_id();
    
    // Then initialize UART and tasks
    uart_init();
    
    xTaskCreate(uart_rx_task, "uart_rx_task", 2048, NULL, 5, NULL);
    xTaskCreate(uart_tx_task, "uart_tx_task", 2048, NULL, 4, NULL);
}