/*
This is a simple program for ESP32 boards that passes UART messages between them.
LoRa modules are used to transmit the messages wirelessly between ESP32 boards.
IMPORTANT: the receiver_address variable must be set to match the ADDRESS of the LoRa device.
   This is the address assigned to the LoRa device using the AT command AT+ADDRESS=
   Alternatively, you can set receiver_address = 0. This will send messages to all LoRa devices.

This version implements a master/slave timing synchronization where:
- Device ID 0 is the master and establishes timing reference
- Other devices sync to Device 0's transmissions
- Each device uses an offset based on its ID to avoid collisions

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

#define UART_NUM UART_NUM_2
#define TXD_PIN GPIO_NUM_17  // TX pin connected to LoRa RX pin
#define RXD_PIN GPIO_NUM_16  // RX pin connected to LoRa TX pin
#define RST_PIN GPIO_NUM_4   // RST pin connected to LoRa RST pin
#define UART_BAUD_RATE 9600  // Set the UART baud rate to 9600 for LoRa device
#define BUF_SIZE 1024
#define MAX_DEVICES 8        // Maximum number of devices in the network

// ================================================================================
static const int receiver_address = 0;  // Set the receiver address (ID)
// ================================================================================

static const char *TAG = "uart_example";
static char local_lora_device_address[6];  // Buffer to hold the LoRa device address as a string
static char lora_uid[30];        // Buffer to hold the LoRa UID
static int message_counter = 0;  // Counter for incrementing message number
static int device_number = 0;    // Will be set from LoRa address
static int tx_offset = 0;        // Will be calculated from device_number
static bool sync_to_master = false;  // Track if we've synchronized with device 0
static const int TX_PERIOD = 2000;   // 2 seconds between transmissions

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

// Task to handle LoRa transmitting with master/slave synchronization
void uart_tx_task(void *arg) {
   char tx_buffer[BUF_SIZE];
   
   if (device_number == 0) {
       // Device 0 (master) starts immediately with no delays
       ESP_LOGI(TAG, "This is master device (ID=0), starting immediately with no offset");
       sync_to_master = true;
       tx_offset = 0;
   } else {
       // Other devices start with random delays until they sync
       int initial_random_delay = esp_random() % 1000;
       ESP_LOGI(TAG, "Non-master device (ID=%d), starting with random delay: %d ms", 
                device_number, initial_random_delay);
       vTaskDelay(pdMS_TO_TICKS(initial_random_delay));
       ESP_LOGI(TAG, "Waiting for sync from master device (ID=0)");
   }

   while (1) {
       if (sync_to_master) {
           message_counter++;
           
           // Construct message with Device ID embedded in format: "IDx-HELLO-y-OFFz"
           // where x=device_number, y=message_counter, z=tx_offset
           snprintf(tx_buffer, BUF_SIZE, "AT+SEND=%d,%d,ID%d-HELLO-%d-OFF%d\r\n", 
                   receiver_address, 
                   strlen("ID") + 1 + strlen("HELLO") + 1 + 
                   (message_counter < 10 ? 1 : (message_counter < 100 ? 2 : 3)) + 5,
                   device_number,  // Device ID included here
                   message_counter,
                   tx_offset);
           
           uart_write_bytes(UART_NUM, tx_buffer, strlen(tx_buffer));
           ESP_LOGI(TAG, "Send command sent: %s", tx_buffer);
           
           if (device_number == 0) {
               // Master device maintains strict timing with no extra delays
               vTaskDelay(pdMS_TO_TICKS(TX_PERIOD));
           } else {
               // Non-master devices include processing delay compensation
               vTaskDelay(pdMS_TO_TICKS(100));  // Processing delay
               vTaskDelay(pdMS_TO_TICKS(TX_PERIOD - 100));
           }
       } else {
           // Not synced yet, keep waiting with random delays
           vTaskDelay(pdMS_TO_TICKS(100));
       }
   }
}

// Task to handle UART receiving and timing synchronization
void uart_rx_task(void *arg) {
   uint8_t* rx_buffer = (uint8_t*) malloc(BUF_SIZE);
   int len;

   while (1) {
       len = uart_read_bytes(UART_NUM, rx_buffer, BUF_SIZE - 1, pdMS_TO_TICKS(100));
       if (len > 0) {
           rx_buffer[len] = 0;
           
           // Check for received message
           if (strstr((char*)rx_buffer, "+RCV") != NULL) {
               // Parse the sender's device ID from the message format "IDx-HELLO-y-OFFz"
               char *id_start = strstr((char*)rx_buffer, "ID");
               if (id_start) {
                   int sender_id;
                   if (sscanf(id_start, "ID%d", &sender_id) == 1) {
                       ESP_LOGI(TAG, "Received message from device ID: %d", sender_id);
                       
                       // If this is a message from device 0 and we're not synced
                       if (sender_id == 0 && !sync_to_master && device_number != 0) {
                           ESP_LOGI(TAG, "Syncing to master device");
                           
                           // Calculate our offset based on our device number
                           tx_offset = (TX_PERIOD * device_number) / MAX_DEVICES;
                           
                           // Mark as synced and start transmitting
                           sync_to_master = true;
                           ESP_LOGI(TAG, "Synced to master, using offset: %d ms", tx_offset);
                       }
                   }
               }
           }
           
           ESP_LOGI(TAG, "Received: %s", rx_buffer);
       }
       vTaskDelay(pdMS_TO_TICKS(100));
   }
   free(rx_buffer);
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
   xTaskCreate(uart_rx_task, "uart_rx_task", 2048, NULL, 5, NULL);
   xTaskCreate(uart_tx_task, "uart_tx_task", 2048, NULL, 4, NULL);

   ESP_LOGI(TAG, "Tasks started");
   ESP_LOGI(TAG, "==================================================");
   vTaskDelay(pdMS_TO_TICKS(2000));
}