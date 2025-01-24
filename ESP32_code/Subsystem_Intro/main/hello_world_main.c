#include <stdio.h> //standard input and outputs (printf)
#include "freertos/FreeRTOS.h" //handles tasks and scheduling on the ESP32
#include "freertos/task.h"
#include "esp_system.h" //esp32 functions
#include "driver/gpio.h" //Controls GPIO pins 
#include "driver/uart.h" // UART
#include "esp_vfs_dev.h"


#define UART_PORT_NUM      UART_NUM_0
#define UART_BAUD_RATE     115200
#define UART_TX_PIN        GPIO_NUM_1  // TX pin
#define UART_RX_PIN        GPIO_NUM_3  // RX pin

// LEDs are connected to pins D4 and D5 on the dev kit, define them here
#define LED1_GPIO 5  // LED 1 in D5
#define LED2_GPIO 4  // LED 2 in D4

void init_uart() {
    const uart_config_t uart_config = { // requirements to configure UART https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/uart.html
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    // Setup UART buffered IO with an event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
}

void hello_world_uart(void *pvParameter) {
    char* test_str = "hello world.\r\n";
    for (int i = 0; i < 20; i++) {
        // Write data to UART
        uart_write_bytes(UART_PORT_NUM, (const char*)test_str, strlen(test_str));
        vTaskDelay(3000 / portTICK_PERIOD_MS);  // Delay 3 seconds
    }
}

// blink a few LEDs, two in my case
void blinky(void *pvParameter) {
    gpio_reset_pin(LED1_GPIO); // Reset the pins 
    gpio_reset_pin(LED2_GPIO);
    gpio_set_direction(LED1_GPIO, GPIO_MODE_OUTPUT); //declare the LEDs as an output
    gpio_set_direction(LED2_GPIO, GPIO_MODE_OUTPUT);
    
    while(1) {
        // Turn LED 1 on and keep LED 2 off
        gpio_set_level(LED1_GPIO, 1); // 1 is high voltage (3.3V)
        gpio_set_level(LED2_GPIO, 0); // 0 is low voltage
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // every second

        // Turn LED 1 off, LED 2 on
        gpio_set_level(LED1_GPIO, 0);
        gpio_set_level(LED2_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // every second
    }
}

void app_main() {
    // Create tasks for LED blinking and "Hello World" printing
    init_uart();
    xTaskCreate(&hello_world_uart, "hello_world_uart", 2048, NULL, 5, NULL); // pointer to function, stack size, 5 is for the task priority
    xTaskCreate(&blinky, "blinky", 2048, NULL, 5, NULL);
}