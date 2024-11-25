#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#define UART_NUM UART_NUM_0         // UART port number
#define UART_TX_PIN 1             // GPIO for TX
#define UART_RX_PIN 3              // GPIO for RX
#define BAUD_RATE 2400              // UART baud rate
#define BUF_SIZE 1024               // UART buffer size
#define EEPROM_NAMESPACE "storage"  // NVS namespace

#define LED_BUILTIN GPIO_NUM_2      // Built-in LED GPIO pin
#define LED_BLINK_RECEPTION 500     // Blink delay in ms for reception
#define LED_BLINK_TRANSMISSION 100  // Blink delay in ms for transmission

static const char *TAG = "ESP32_UART_LED";

// Initialize UART
void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART initialized.");
}

// Initialize LED GPIO
void led_init() {
    gpio_pad_select_gpio(LED_BUILTIN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_BUILTIN, 1);  // Turn off LED initially
}

// Blink the LED with a specified delay
void blink_led(int delay_ms) {
    gpio_set_level(LED_BUILTIN, 1);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    gpio_set_level(LED_BUILTIN, 0);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

// Store data in EEPROM
void eeprom_store_data(const char *data, size_t length) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(EEPROM_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return;
    }

    err = nvs_set_blob(nvs_handle, "data", data, length);
    if (err == ESP_OK) {
        nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
}

// Read data from EEPROM
size_t eeprom_read_data(char *buffer, size_t buffer_size) {
    nvs_handle_t nvs_handle;
    size_t data_size = 0;
    esp_err_t err = nvs_open(EEPROM_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return 0;
    }

    err = nvs_get_blob(nvs_handle, "data", NULL, &data_size);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return 0;
    }

    if (data_size > buffer_size) {
        data_size = buffer_size;
    }

    err = nvs_get_blob(nvs_handle, "data", buffer, &data_size);
    if (err == ESP_OK) {
    } else {
        data_size = 0;
    }

    nvs_close(nvs_handle);
    return data_size;
}

void app_main() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize peripherals
    uart_init();
    led_init();

    char rx_buffer[BUF_SIZE];
    char tx_buffer[BUF_SIZE];
    size_t received_length;

    while (1) {
        // **Receive Data**
        received_length = uart_read_bytes(UART_NUM, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (received_length > 0) {
            rx_buffer[received_length] = '\0';  // Null-terminate the received data

            // Blink LED for reception
            blink_led(LED_BLINK_RECEPTION);

            // Store received data in EEPROM
            eeprom_store_data(rx_buffer, received_length);

            // Read data back from EEPROM
            size_t read_length = eeprom_read_data(tx_buffer, BUF_SIZE);
            if (read_length > 0) {
                // **Transmit Data**
                for (size_t i = 0; i < read_length; i++) {
                    uart_write_bytes(UART_NUM, &tx_buffer[i], 1);
                    blink_led(LED_BLINK_TRANSMISSION);  // Blink LED faster during transmission
                }
            }
        }
    }
}