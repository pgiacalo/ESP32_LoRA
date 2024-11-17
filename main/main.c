// Prior to using the LoRA module, connect it to your computer via FTDI/USB.
// Make sure the FTDI is set for 3.3V and not 5V.
// Connect the FTDI TX to the LoRa RX, and the FTDI RX to the LoRa TX.
// Then, open a terminal and connect to the LoRa module at 115200 baud.
// Then, use the AT+ commands to configure the module (see local file REYAX_LoRa_AT_Command_RYLR998_RYLR498_EN.pdf)
// Set the ADDRESS, BAND, NETWORKID, MODE, and CPIN
// For example: AT+ADDRESS=1, AT+BAND=915000000, AT+NETWORKID=10, AT+MODE=0, AT+CPIN=A3F2B8C1
// Then, load this program onto the ESP32.
// The ESP32 will send a message, and the other device should receive it.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"          // For general ESP32 system functions
#include "esp_mac.h"             // For esp_efuse_mac_get_default()
#include "esp_chip_info.h"       // For esp_chip_info_t and esp_chip_info()

// LoRa Module Pins
#define LORA_TX  27  // Connect to RX of LoRa
#define LORA_RX  19  // Connect to TX of LoRa
#define LORA_RST 14  // Reset pin

// LoRa Settings
#define LORA_FREQUENCY    915E6  // 915 MHz
#define LORA_BANDWIDTH    125E3  // 125 kHz¬
#define LORA_SPREADING    7      // Spreading Factor
#define LORA_CODING_RATE 5      // Coding Rate 4/5

// LoRa Registers
#define REG_FIFO          0x00
#define REG_OP_MODE       0x01
#define REG_FR_MSB        0x06
#define REG_FR_MID        0x07
#define REG_FR_LSB        0x08
#define REG_PA_CONFIG     0x09
#define REG_MODEM_CONFIG1 0x1D
#define REG_MODEM_CONFIG2 0x1E
#define REG_MODEM_CONFIG3 0x26
#define REG_PAYLOAD_LEN   0x22

#define NETWORK_ID 10

static const char *TAG = "LoRa_Device";  
static const char *name = "Sam";  // Sender name
static const int receiver_address = 1;  // Receiver address
static int message_counter = 0;  // Counter for incrementing message number

// Function prototype
void lora_set_frequency(uint32_t frequency);

// Initialize UART for LoRa
static void lora_init() {
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = 115200,  // Ensure this matches on both devices
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, LORA_TX, LORA_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 256, 256, 0, NULL, 0));
    
    // Reset LoRa Module
    gpio_set_direction(LORA_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set the frequency
    lora_set_frequency(LORA_FREQUENCY);
}

// Function definition
void lora_set_frequency(uint32_t frequency) {
    char command[50];
    snprintf(command, sizeof(command), "AT+FREQ=%lu\r\n", frequency);  // Add CRLF for command termination
    uart_write_bytes(UART_NUM_1, command, strlen(command));
    ESP_LOGI(TAG, "Set frequency command sent: %s", command);
}

// Function to get the unique identifier of the ESP32 chip
void log_chip_id() {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip Info: %d cores, revision %d, %s", 
              chip_info.cores, chip_info.revision, 
              (chip_info.features & CHIP_FEATURE_BT) ? "Bluetooth" : "No Bluetooth");
}

// Function to send a message
void lora_send_message() {
    message_counter++;  // Increment the message counter
    char message[50];  // Buffer for the formatted message
    char command[100]; // Buffer for the AT command

    // Format the message as "name-counter"
    snprintf(message, sizeof(message), "%s-%d", name, message_counter);
    int payload_length = strlen(message);  // Calculate the length of the message

    // Format the command according to the expected syntax
    snprintf(command, sizeof(command), "AT+SEND=%d,%d,%s\r\n", receiver_address, payload_length, message);

    // Send the command to the LoRa module
    uart_write_bytes(UART_NUM_1, command, strlen(command));
    ESP_LOGI(TAG, "Send command sent: %s", command);
}

// Function to receive a message
void lora_receive_message() {
    uint8_t data[128];
    int length = uart_read_bytes(UART_NUM_1, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
    if (length > 0) {
        data[length] = 0;  // Null terminate
        int received_network_id;
        char sender_name[20];
        int counter;

        // Parse the message (assuming format is "networkid-name-counter")
        if (sscanf((char*)data, "%d-%[^-]-%d", &received_network_id, sender_name, &counter) == 3) {
            if (received_network_id == NETWORK_ID) {
                ESP_LOGI(TAG, "Message received: %s", (char*)data);
            } else {
                ESP_LOGI(TAG, "Ignored message from different network ID: %d", received_network_id);
            }
        }
    }
}

void app_main() {
    lora_init();

    while (1) {
        lora_send_message();  // Send a message
        vTaskDelay(pdMS_TO_TICKS(2000));  // Delay for 2 seconds between messages
        lora_receive_message();  // Check for received messages
    }
}

void lora_set_band(int band) {
    char command[50];
    snprintf(command, sizeof(command), "AT+BAND=%d\r\n", band);  // Add CRLF for command termination
    uart_write_bytes(UART_NUM_1, command, strlen(command));
    ESP_LOGI(TAG, "Set band command sent: %s", command);
}