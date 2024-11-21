#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "LoRa";

// Pin definitions
#define TXD_PIN GPIO_NUM_17
#define RXD_PIN GPIO_NUM_16
#define RST_PIN GPIO_NUM_4

// UART configurations
#define UART_PORT UART_NUM_2
#define UART_BAUD_RATE 9600
#define BUF_SIZE 1024

// Application configurations
#define MASTER_ADDRESS 0    //the master LoRa device MUST have address 0
#define NUMBER_OF_SLAVES 1
#define TX_PERIOD 4000  // milliseconds
#define MAX_DEVICES 256
#define STATS_REPORT_INTERVAL 100   //the total number of messages between statistics reports

// Function prototypes
void uart_init(void);
void get_device_address(void);
void calculate_tx_delay(void);
void send_message(const char* message);
void process_received_message(const char* message);
void parse_received_data(const char* data, uint8_t* sender_address, uint16_t* msg_number);
void handle_lora_error(const char* response);
void print_statistics(void);  // Added this prototype


// Message tracking structure for each device
typedef struct {
    uint16_t messages_received;
    uint16_t messages_lost;
    uint16_t last_message_number;
} DeviceStats;

// Error mapping structure
typedef struct {
    uint8_t code;
    const char* description;
} LoRaError;

// Error code to description mapping
static const LoRaError LORA_ERRORS[] = {
    {1, "No CR/LF (0x0D 0x0A) at end of AT Command"},
    {2, "AT command header missing"},
    {4, "Unknown command"},
    {5, "Data length mismatch with actual length"},
    {10, "TX timeout exceeded"},
    {12, "CRC error"},
    {13, "TX data exceeds 240 bytes"},
    {14, "Failed to write flash memory"},
    {15, "Unknown failure"},
    {17, "Last TX was not completed"},
    {18, "Preamble value not allowed"},
    {19, "RX failed - Header error"},
    {20, "Invalid smart receiving power saving mode time setting"}
};

static const int NUM_LORA_ERRORS = sizeof(LORA_ERRORS) / sizeof(LoRaError);

// Global variables
uint8_t device_address;
uint16_t message_number = 1;
uint16_t tx_delay;
DeviceStats device_stats[MAX_DEVICES];
uint16_t total_messages_received = 0;
uint16_t total_messages_lost = 0;

void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, RST_PIN, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
}

void handle_lora_error(const char* response) {
    uint8_t error_code;
    if (sscanf(response, "+ERR=%hhu", &error_code) == 1) {
        const char* error_description = "Undefined error";
        
        for (int i = 0; i < NUM_LORA_ERRORS; i++) {
            if (LORA_ERRORS[i].code == error_code) {
                error_description = LORA_ERRORS[i].description;
                break;
            }
        }
        
        ESP_LOGE(TAG, "Error Code %d - %s", error_code, error_description);
    }
}

void send_message(const char* message) {
    char tx_buffer[256];
    
    // Add CR+LF to the AT command
    snprintf(tx_buffer, sizeof(tx_buffer), "%s\r\n", message);
    ESP_LOGI(TAG, "TX: %s", message);  // Log complete AT command
    uart_write_bytes(UART_PORT, tx_buffer, strlen(tx_buffer));
}

void get_device_address(void) {
    // Send the address query command
    uart_flush(UART_PORT);
    send_message("AT+ADDRESS?");
    
    // Now we need to wait for and read the response ourselves since send_message no longer does it
    char response[64];
    memset(response, 0, sizeof(response));
    
    int len = uart_read_bytes(UART_PORT, (uint8_t*)response, sizeof(response), pdMS_TO_TICKS(1000));
    if (len > 0) {
        response[len] = 0;
        ESP_LOGI(TAG, "Address Response: %s", response);
        
        // Remove CR/LF from response if present
        char* newline = strchr(response, '\r');
        if (newline) *newline = 0;
        newline = strchr(response, '\n');
        if (newline) *newline = 0;
        
        if (sscanf(response, "+ADDRESS=%hhu", &device_address) == 1) {
            ESP_LOGI(TAG, "Successfully got device address: %d", device_address);
        } else {
            ESP_LOGE(TAG, "Invalid address response format: %s", response);
            device_address = 0xFF;  // Set to invalid address
        }
    } else {
        ESP_LOGE(TAG, "No response received for address query");
        device_address = 0xFF;  // Set to invalid address
    }
}

void calculate_tx_delay(void) {
    if (device_address == MASTER_ADDRESS) {
        tx_delay = 0;  // Master device always has 0 delay
    } else {
        // Use the correct equation: TX_DELAY = (TX_PERIOD * ADDRESS) / (NUMBER_OF_SLAVES + 1)
        tx_delay = (TX_PERIOD * device_address) / (NUMBER_OF_SLAVES + 1);
    }
    ESP_LOGI(TAG, "Device role: %s, Address: %d, TX_DELAY: %d ms", 
             (device_address == MASTER_ADDRESS) ? "Master" : "Slave", 
             device_address, 
             tx_delay);
}

void process_received_message(const char* message) {
    if (strncmp(message, "+ERR=", 5) == 0) {
        handle_lora_error(message);
        return;
    }
    
    ESP_LOGI(TAG, "RX: %s", message);  // Log complete received message

    uint8_t sender_address;
    int msg_length;
    char msg_content[256];
    int16_t rssi;
    float snr;
    
    // Format: +RCV=1,10,HELLO,1,73,-34,11
    // Parse: sender address, message length
    if (sscanf(message, "+RCV=%hhu,%d,", &sender_address, &msg_length) != 2) {
        ESP_LOGW(TAG, "Failed to parse RCV header: %s", message);
        return;
    }

    // Find start of message content (after second comma)
    const char* msg_start = strchr(message + 5, ',');  // Skip "+RCV=" and find first comma
    if (msg_start) {
        msg_start = strchr(msg_start + 1, ',');  // Find second comma
        if (msg_start) {
            msg_start++;  // Move past the comma
            
            // Format inside message content: HELLO,1,73
            char msg_type[10];
            uint8_t msg_addr;
            uint16_t msg_seq;
            
            if (sscanf(msg_start, "%[^,],%hhu,%hu,%hd,%f", 
                      msg_type, &msg_addr, &msg_seq, &rssi, &snr) == 5) {
                
                // Verify sender address matches address in message
                if (sender_address != msg_addr) {
                    ESP_LOGW(TAG, "Address mismatch: RCV=%d, MSG=%d", sender_address, msg_addr);
                }
                
                // Check for dropped messages
                if (device_stats[sender_address].last_message_number > 0) {
                    uint16_t expected_msg_number = device_stats[sender_address].last_message_number + 1;
                    if (msg_seq > expected_msg_number) {
                        uint16_t lost_messages = msg_seq - expected_msg_number;
                        
                        ESP_LOGW(TAG, "DROPS: addr=%d, last=%d, exp=%d, rcv=%d, lost=%d", 
                                sender_address,
                                device_stats[sender_address].last_message_number,
                                expected_msg_number,
                                msg_seq,
                                lost_messages);
                        
                        device_stats[sender_address].messages_lost += lost_messages;
                        total_messages_lost += lost_messages;
                    }
                }
                
                device_stats[sender_address].messages_received++;
                device_stats[sender_address].last_message_number = msg_seq;
                total_messages_received++;
                           
                if ((total_messages_received + total_messages_lost) % STATS_REPORT_INTERVAL == 0) {
                    print_statistics();
                }
            } else {
                ESP_LOGW(TAG, "Failed to parse message content: %s", msg_start);
            }
        }
    }
}

void print_statistics(void) {
    ESP_LOGI(TAG, "============= Statistics Report =============");
    ESP_LOGI(TAG, "Messages per device:");
    
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (device_stats[i].messages_received > 0) {
            ESP_LOGI(TAG, "Device %d: Received: %u, Lost: %u",
                     i,
                     device_stats[i].messages_received,
                     device_stats[i].messages_lost);
        }
    }
    
    uint16_t total_messages = total_messages_received + total_messages_lost;
    float drop_percentage = (total_messages > 0) ?
        (100.0f * total_messages_lost) / total_messages : 0;
    
    ESP_LOGI(TAG, "Total Statistics:");
    ESP_LOGI(TAG, "Total Messages Received: %u", total_messages_received);
    ESP_LOGI(TAG, "Total Messages Lost: %u", total_messages_lost);
    ESP_LOGI(TAG, "Total Messages: %u", total_messages);
    ESP_LOGI(TAG, "Drop Percentage: %.2f%%", drop_percentage);
    ESP_LOGI(TAG, "=============================================");    
}

bool init_lora(void) {
    uart_flush(UART_PORT);
    send_message("AT");
    
    // Wait for response
    char response[64];
    memset(response, 0, sizeof(response));
    
    int len = uart_read_bytes(UART_PORT, (uint8_t*)response, sizeof(response), pdMS_TO_TICKS(1000));
    if (len > 0) {
        response[len] = 0;
        
        // Remove CR/LF from response if present
        char* newline = strchr(response, '\r');
        if (newline) *newline = 0;
        newline = strchr(response, '\n');
        if (newline) *newline = 0;
        
        if (strcmp(response, "+OK") == 0) {
            ESP_LOGI(TAG, "LoRa device connection test: successful");
            return true;
        } else {
            ESP_LOGE(TAG, "LoRa device connection test failed. Unexpected response: %s", response);
            return false;
        }
    }
    
    ESP_LOGE(TAG, "LoRa device connection test failed. No response received");
    return false;
}

void app_main(void) {
    uart_init();
    
    vTaskDelay(pdMS_TO_TICKS(1000));  // Initial delay for LoRa module
    
    // Test LoRa connection
    if (!init_lora()) {
        while(1) {
            ESP_LOGE(TAG, "Failed to initialize LoRa device. Halted.");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    get_device_address();
    calculate_tx_delay();
    
    memset(device_stats, 0, sizeof(device_stats));
    
    char rx_buffer[BUF_SIZE];
    bool waiting_for_ok = false;
    
    while (1) {
        if (device_address == MASTER_ADDRESS) {  // Master device
            // Send sync message
            char message[256];
            char payload[64];
            
            snprintf(payload, sizeof(payload), "SYNC,%d,%u", device_address, message_number++);
            int payload_len = strlen(payload);
            snprintf(message, sizeof(message), "AT+SEND=0,%d,%s", payload_len, payload);
            
            send_message(message);
            
            // Check for responses during TX_PERIOD
            uint32_t start_time = pdTICKS_TO_MS(xTaskGetTickCount());
            while (pdTICKS_TO_MS(xTaskGetTickCount()) - start_time < TX_PERIOD) {
                int len = uart_read_bytes(UART_PORT, (uint8_t*)rx_buffer, 
                                        BUF_SIZE, pdMS_TO_TICKS(10));
                if (len > 0) {
                    rx_buffer[len] = 0;
                    
                    if (strncmp(rx_buffer, "+ERR=", 5) == 0) {
                        handle_lora_error(rx_buffer);
                    } else if (strncmp(rx_buffer, "+RCV=", 5) == 0) {
                        process_received_message(rx_buffer);
                    } else if (strncmp(rx_buffer, "+OK", 3) == 0) {
                        waiting_for_ok = false;
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            
        } else {  // Slave device
            // Check for received messages
            int len = uart_read_bytes(UART_PORT, (uint8_t*)rx_buffer, 
                                    BUF_SIZE, pdMS_TO_TICKS(100));
            if (len > 0) {
                rx_buffer[len] = 0;
                
                if (strncmp(rx_buffer, "+ERR=", 5) == 0) {
                    handle_lora_error(rx_buffer);
                    waiting_for_ok = false;
                } else if (strncmp(rx_buffer, "+RCV=", 5) == 0) {
                    process_received_message(rx_buffer);
                    
                    if (!waiting_for_ok) {
                        vTaskDelay(pdMS_TO_TICKS(tx_delay));
                        
                        // Prepare hello message
                        char message[256];
                        char payload[64];
                        
                        snprintf(payload, sizeof(payload), "HELLO,%d,%u", device_address, message_number++);
                        int payload_len = strlen(payload);
                        snprintf(message, sizeof(message), "AT+SEND=0,%d,%s", payload_len, payload);
                        
                        send_message(message);
                        waiting_for_ok = true;
                    }
                } else if (strncmp(rx_buffer, "+OK", 3) == 0) {
                    waiting_for_ok = false;  // Success, just clear flag
                } else if (waiting_for_ok) {
                    ESP_LOGE(TAG, "Unexpected response while waiting for OK: %s", rx_buffer);
                    waiting_for_ok = false;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));  // Prevent watchdog timeout
        }
    }
}
