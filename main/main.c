/*
This is a simple program for ESP32 boards that passes UART messages between them.
LoRa modules are used to transmit the messages wirelessly between ESP32 boards.
IMPORTANT: the receiver_address variable must be set to match the ADDRESS of the LoRa device.
    This is the address assigned to the LoRa device using the AT command AT+ADDRESS=
    Alternatively, you can set receiver_address = 0. This will send messages to all LoRa devices.

This version implements a master/slave timing synchronization where:
- Device ID 1 is the master and establishes timing reference
- Other devices sync to Device 1's transmissions
- Each device uses an offset based on its ID to avoid collisions
- Non-master devices use continuous random delays until first sync with master

Message Format Configuration:
----------------------------
Set INCLUDE_OFFSET_IN_MESSAGES below to control message format:

If INCLUDE_OFFSET_IN_MESSAGES is true:
    Format: "IDx-HELLO-y-OFFz"
    where: x = device ID (1=master)
           y = message counter
           z = tx_offset in ms
    Example from master: "ID1-HELLO-42-OFF0"
    Example from device 2: "ID2-HELLO-17-OFF500"

If INCLUDE_OFFSET_IN_MESSAGES is false:
    Format: "IDx-HELLO-y"
    where: x = device ID (1=master)
           y = message counter
    Example from master: "ID1-HELLO-42"
    Example from device 2: "ID2-HELLO-17"

Note: Per LoRa specification, maximum payload size is 255 bytes.
      The AT command format is: AT+SEND=<address>,<length>,<data>
      Example: AT+SEND=50,5,HELLO  (where 5 is the length of "HELLO")

Here is the wiring diagram showing the connections between the ESP32 board and the LoRa module.

    ESP32            LoRa Module
    ----------------------------
    GPIO17 (TX) ---> RX pin
    GPIO16 (RX) <--- TX pin
    GPIO4 (RST) ---> RST pin
    GND <----------> GND
*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <inttypes.h>
#include "esp_timer.h"

// Configuration flags and constants
#define INCLUDE_OFFSET_IN_MESSAGES false  // Set to true to include offset in messages
#define MAX_LORA_PAYLOAD 255    // LoRa specification maximum payload size
#define MAX_DATA_SIZE 255       // Match LoRa max payload
#define MAX_CMD_SIZE 280        // Enough for "AT+SEND=xxx,255,<255 bytes>\r\n"
#define BUF_SIZE 1024          // Buffer size for UART operations
#define TASK_STACK_SIZE 4096   // Increased stack size for tasks
#define UART_NUM UART_NUM_2
#define TXD_PIN GPIO_NUM_17     // TX pin connected to LoRa RX pin
#define RXD_PIN GPIO_NUM_16     // RX pin connected to LoRa TX pin
#define RST_PIN GPIO_NUM_4      // RST pin connected to LoRa RST pin
#define UART_BAUD_RATE 9600     // Set the UART baud rate to 9600 for LoRa device
#define MAX_DEVICES 8           // Maximum number of devices in the network

// ================================================================================
static const int receiver_address = 0;  // Set the receiver address (ID)
// ================================================================================

static const char *TAG = "uart_example";
static char local_lora_device_address[6];  // Buffer to hold the LoRa device address as a string
static char lora_uid[30];        // Buffer to hold the LoRa UID
static int message_counter = 0;  // Counter for incrementing message number
static int device_number = 0;    // Will be set from LoRa address
static int tx_offset = 0;        // Will be calculated from device_number
static bool sync_to_master = false;  // Track if we've synchronized with device 1
static const int TX_PERIOD = 2000;   // 2 seconds between transmissions
// Add global variable to track last master message time (i.e., the time a message was last received from Device ID = 1)
static int64_t last_master_rx_time = 0;

// Function to get the LoRa device address
void get_lora_device_address() {
    char response[50];
    uart_write_bytes(UART_NUM, "AT+ADDRESS?\r\n", strlen("AT+ADDRESS?\r\n"));
    
    int len = uart_read_bytes(UART_NUM, response, sizeof(response) - 1, pdMS_TO_TICKS(1000));
    if (len > 0) {
        response[len] = '\0';
        ESP_LOGI(TAG, "Response received: %s", response);

        char *address_start = strstr(response, "+ADDRESS=");
        if (address_start) {
            address_start += 9;
            sscanf(address_start, "%s", local_lora_device_address);
            ESP_LOGI(TAG, "Local LoRa device address extracted: %s", local_lora_device_address);
        } else {
            ESP_LOGE(TAG, "LoRa device address not found in response.");
        }
    } else {
        ESP_LOGE(TAG, "No response received for address command.");
    }
}

// Function to get the UID from the LoRa module
void get_uid_from_lora() {
    char response[50];
    uart_write_bytes(UART_NUM, "AT+UID?\r\n", strlen("AT+UID?\r\n"));
    
    int len = uart_read_bytes(UART_NUM, response, sizeof(response) - 1, pdMS_TO_TICKS(1000));
    if (len > 0) {
        response[len] = '\0';
        ESP_LOGI(TAG, "Response received: %s", response);

        char *uid_start = strstr(response, "+UID=");
        if (uid_start) {
            uid_start += 5;
            sscanf(uid_start, "%s", lora_uid);
            ESP_LOGI(TAG, "LoRa UID extracted: %s", lora_uid);
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
        .baud_rate = UART_BAUD_RATE,
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

void uart_rx_task(void *arg) {
    uint8_t rx_buffer[BUF_SIZE];
    int len;

    while (1) {
        len = uart_read_bytes(UART_NUM, rx_buffer, BUF_SIZE - 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            rx_buffer[len] = 0;
            
            if (strstr((char*)rx_buffer, "+RCV") != NULL) {
                char *id_start = strstr((char*)rx_buffer, "ID");
                if (id_start) {
                    int sender_id;
                    if (sscanf(id_start, "ID%d", &sender_id) == 1) {
                        ESP_LOGI(TAG, "Received message from device ID: %d", sender_id);
                        
                        // Update timing for any message from master
                        if (sender_id == 1 && device_number != 1) {
                            last_master_rx_time = esp_timer_get_time() / 1000; // Convert to ms
                            
                            if (!sync_to_master) {
                                ESP_LOGI(TAG, "Initial sync to master device");
                                sync_to_master = true;
                            }
                        }
                    }
                }
            }
            ESP_LOGI(TAG, "Received: %s", rx_buffer);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// In uart_tx_task, time transmissions relative to last master message
void uart_tx_task(void *arg) {
    char tx_buffer[MAX_CMD_SIZE];
    char data_buffer[MAX_DATA_SIZE];
    int data_length;
    
    if (device_number == 1) {
        ESP_LOGI(TAG, "This is master device (ID=1), starting immediately");
        sync_to_master = true;
    } else {
        ESP_LOGI(TAG, "Non-master device (ID=%d), waiting for sync", device_number);
    }

    while (1) {
        if (sync_to_master || device_number == 1) {
            if (device_number != 1) {
                // Calculate time since last master message
                int64_t current_time = esp_timer_get_time() / 1000;
                int time_since_master = current_time - last_master_rx_time;
                
                // Calculate wait time to reach our offset
                int wait_time = tx_offset - (time_since_master % TX_PERIOD);
                if (wait_time < 0) {
                    wait_time += TX_PERIOD;
                }
                
                vTaskDelay(pdMS_TO_TICKS(wait_time));
            }
            
            message_counter++;
            
            // Construct message with Device ID embedded
            if (INCLUDE_OFFSET_IN_MESSAGES) {
                snprintf(data_buffer, MAX_DATA_SIZE, "ID%d-HELLO-%d-OFF%d", 
                        device_number,
                        message_counter,
                        tx_offset);
            } else {
                snprintf(data_buffer, MAX_DATA_SIZE, "ID%d-HELLO-%d", 
                        device_number,
                        message_counter);
            }
            
            data_length = strlen(data_buffer);
            
            // Verify we haven't exceeded LoRa payload size
            if (data_length > MAX_LORA_PAYLOAD) {
                ESP_LOGE(TAG, "Data exceeds LoRa maximum payload size!");
                continue;
            }
            
            // Construct full command
            snprintf(tx_buffer, MAX_CMD_SIZE, "AT+SEND=%d,%d,%s\r\n", 
                    receiver_address,
                    data_length,
                    data_buffer);
            
            uart_write_bytes(UART_NUM, tx_buffer, strlen(tx_buffer));
            ESP_LOGI(TAG, "Send command sent: %s", tx_buffer);

            vTaskDelay(pdMS_TO_TICKS(TX_PERIOD));
        } else {
            // Not synced yet
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "==================================================");

    ESP_LOGI(TAG, "Initializing the UART");
    uart_init();
    ESP_LOGI(TAG, "UART Initialized");

    ESP_LOGI(TAG, "Getting local LoRa device address");
    get_lora_device_address();
    ESP_LOGI(TAG, "Local LoRa Device Address: %s", local_lora_device_address);

    // Convert address to number (this becomes our device ID)
    device_number = atoi(local_lora_device_address);
    ESP_LOGI(TAG, "Device ID number: %d", device_number);

    // Get UID with retry logic
    ESP_LOGI(TAG, "Getting UID from the local LoRa device");
    int retries = 3;
    while (retries > 0) {
        get_uid_from_lora();
        if (strlen(lora_uid) > 0) {
            break;
        }
        retries--;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "Local LoRa Device UID: %s", lora_uid);

    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "Starting Rx and Tx tasks");
    xTaskCreate(uart_rx_task, "uart_rx_task", TASK_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(uart_tx_task, "uart_tx_task", TASK_STACK_SIZE, NULL, 4, NULL);

    ESP_LOGI(TAG, "Tasks started");
    ESP_LOGI(TAG, "==================================================");
    vTaskDelay(pdMS_TO_TICKS(2000));
}

