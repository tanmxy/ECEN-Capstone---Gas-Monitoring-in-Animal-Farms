#include <stdio.h> // I/O library for input and output functions such as printf
#include <string.h> //String handling

#include "freertos/FreeRTOS.h" //headers for task management
#include "freertos/task.h" //headers for task handling
#include "freertos/event_groups.h" //header for event groups

#include "esp_system.h" //system related functions
#include "esp_wifi.h" //Wi-Fi functionality 
#include "esp_event.h" //event management for esp32
#include "esp_log.h" // for logging
#include "nvs_flash.h" // Non-Volatile Storage for esp32

#include "lwip/inet.h" // LwIP library for internet functions
#include "lwip/netdb.h" // LwIP for DNS and address management
#include "lwip/sockets.h" // LwIP socket functions for network communications

#include "driver/gpio.h" //Controls GPIO pins 
#include "driver/uart.h" //UART communications library

#define PORT 3333 // Defining port to connect to on the server
#define SERVER_IP "192.168.4.1" // IP address of ESP32 #1 (server)
#define WIFI_SSID "ESP32-Access-Point" // Defining my Wi-Fi SSID to connect to 
#define WIFI_PASS "joaquinsalas" // Defining the Wi-Fi password

#define BLUE_LED_GPIO GPIO_NUM_19 // Define GPIO pin for blue LED
#define GREEN_LED_GPIO GPIO_NUM_18 // Define GPIO pin for green LED

#define UART_PORT_NUM      UART_NUM_1   // UART port number
#define UART_BAUD_RATE     115200       // UART baud rate
#define UART_TX_PIN        GPIO_NUM_17  // TX pin
#define UART_RX_PIN        GPIO_NUM_16  // RX pin

static const char *TAG = "TCP_SOCKET_CLIENT"; // Log tag to indentify logs related to this module

static EventGroupHandle_t wifi_event_group; // Created event group to manage Wi-Fi status
#define WIFI_CONNECTED_BIT BIT0 // Bit mask fr Wi-Fi connected event

// Global variables for sensor data
float temperature;
float humidity;
float ammonia;
float h2s;
float co2;
float methane;

//////////////////////////////////////// UART DRIVER ////////////////////////////////////////
void uart_init(void) {
    const uart_config_t uart_config = {     // Configuring UART parameters
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    //Configure UART settings using the defined parameters
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    //Set UART pins for TX and RX
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Setup UART buffered IO using event queue
    const int uart_buffer_size = (1024 * 2); // UART buffer size
    QueueHandle_t uart_queue; //Declare queue for UART
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0)); // Install UART driver
}

void send_data_over_uart(const char *data) {
    uart_write_bytes(UART_PORT_NUM, data, strlen(data)); // Send data over UART
}

// void update_and_send_uart_data() {
//     // Using updated sensor values
//     char uart_data[256];
//     snprintf(uart_data, sizeof(uart_data),
//             "Temp: %.2f, Humidity: %.2f, NH3: %.2f, H2S: %.2f, CO2: %.2f, CH4: %.2f\n",
//             temperature, humidity, ammonia, h2s, co2, methane);
//     send_data_over_uart(uart_data); // Send updated data over UART
// }
//////////////////////////////////////// UART DRIVER ////////////////////////////////////////

// Function to blink LED
void blink_led(gpio_num_t gpio, int delay_ms, int blinks) { // Create the inputs for when I call the function
    for (int i = 0; i < blinks; i++) {
        gpio_set_level(gpio, 1); // turn LED on
        vTaskDelay(delay_ms / portTICK_PERIOD_MS); // Delay for the blink duration
        gpio_set_level(gpio, 0); // turn LED off
        vTaskDelay(delay_ms / portTICK_PERIOD_MS); // Delay for the blink duration
    }
}

// Wi-Fi event handler for different Wi-Fi events
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect(); // Connect to the Wi-Fi network
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect(); // Reconnect if disconnected
        ESP_LOGI(TAG, "Reconnecting to the AP...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT); // Set the event bit when connected to Wi-Fi
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip)); // Output (log) IP address when server is connected
    }
}

// Initialize Wi-Fi in STATION mode
void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate(); // Create event gorup for Wi-Fi events
    esp_netif_init(); // Initialize the esp32 network interface
    esp_event_loop_create_default(); // Create default event loop
    esp_netif_create_default_wifi_sta(); // Create the Wi-Fi STA interface
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // Set default Wi-Fi configurations
    esp_wifi_init(&cfg); // Initialize Wi-Fi

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id); // Register event handler for Wi-Fi events
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip); // Register event handler for IP get event

    wifi_config_t wifi_config = { // Configure Wi-Fi credentials with the ones defined
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA); // Set Wi-Fi mode to station (STA)
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config); // Set Wi-Fi configuration
    esp_wifi_start(); // Start the Wi-Fi
}

// TCP Client Task for managing communications with the server
void tcp_client(void *pvParameters) {
    char rx_buffer[128]; // Buffer for receiving data 
    int addr_family = AF_INET; // Set address family for IPv4
    int ip_protocol = IPPROTO_IP; // Set IP protocol

    struct sockaddr_in dest_addr; // Define server address
    dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP); // Set server IP address
    dest_addr.sin_family = AF_INET; // Set address family for IPv4
    dest_addr.sin_port = htons(PORT); // Set server port (3333)

    while (1) { // While esp32 is initializing
        // Create socket
        int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) { // Have a check for socket creation error
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno); // Log error if socket creation fails
            vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait a second before retrying
            continue;
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", SERVER_IP, PORT); // If successful log the message 

        // Connect to server
        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) { // Check for connection error
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno); // Log error if connection fails
            close(sock); // Close the socket
            vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait a second before retrying
            continue;
        }
        ESP_LOGI(TAG, "Successfully connected");  // If successful log the message 

        // Blink green LED rapidly to indicate Wi-Fi initialization
        blink_led(GREEN_LED_GPIO, 100, 10);

        // Send a request message to indicate data transfer
        char *request = "GET_SENSOR_DATA";
        send(sock, request, strlen(request), 0); // Send request to the server

        while (1) {
            // Receive data from server
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len < 0) { // Check for receiving error
                ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno); // Log receiving error
                break;
            } else if (len == 0) { // Check for closed connection
                ESP_LOGI(TAG, "Connection closed by server"); // Log server disconnection
                break;
            } else {
                rx_buffer[len] = '\0'; // Null-terminate the received data
                ESP_LOGI(TAG, "Received data: %s", rx_buffer); // Log received data

                // //Update and send real-time sensor data over UART
                // update_and_send_uart_data();

                // Blink blue LED on each data retrival
                blink_led(BLUE_LED_GPIO, 500, 1);

                // Forward the data via UART
                send_data_over_uart(rx_buffer); // Send data over UART
            }
            vTaskDelay(5000 / portTICK_PERIOD_MS); // Have a delay to avoid flooding the sercer with requests
        }

        // Clean up
        if (sock != -1) { // Check if the socket is valid
            ESP_LOGI(TAG, "Shutting down socket and restarting..."); // Log shutdown message
            shutdown(sock, 0); // Shutdown socket
            close(sock); // Close socket
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS); // Retry connection every 5 seconds if disconnected
    }
}

void app_main(void) {
    // Initialize global variables set to zero
    temperature = 0.0;
    humidity = 0.0;
    ammonia = 0.0;
    h2s = 0.0;
    co2 = 0.0;
    methane = 0.0;

    // Initialize NVS (Non-Volatile Storage) for ESP32
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) { // Check for NVS errors
        nvs_flash_erase(); // Erase NVS is required
        nvs_flash_init(); // Reinitialize NVS
    }
    
    // Initialize LED GPIO pins for blue and green LEDs
    esp_rom_gpio_pad_select_gpio(BLUE_LED_GPIO);
    gpio_set_direction(BLUE_LED_GPIO, GPIO_MODE_OUTPUT); // Set blue LED GPIO as output
    esp_rom_gpio_pad_select_gpio(GREEN_LED_GPIO);
    gpio_set_direction(GREEN_LED_GPIO, GPIO_MODE_OUTPUT); // Set green LED GPIO as output

    // Blink green LED to indicate system initialization
    blink_led(GREEN_LED_GPIO, 200, 5); // Blink green LED 5 times with 200ms delay
    
    // Initialize WiFi with highest priority to ensure connection first
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA"); // Log Wi-Fi mode
    wifi_init_sta(); // Initialize Wi-Fi in STA mode

    // Initialize UART for testing
    uart_init(); // Initialize UART
    char uart_data[256]; // Buffer for UART
    snprintf(uart_data, sizeof(uart_data), // Send sensor data as a string
            "Temp: %.2f, Humidity: %.2f, NH3: %.2f, H2S: %.2f, CO2: %.2f, CH4: %.2f\n",
            temperature, humidity, ammonia, h2s, co2, methane); // In this format
    send_data_over_uart(uart_data); // Send formatted data over UART

    // Wait for Wi-Fi to connect
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY); // Block until Wi-Fi is connected

    // Start TCP client task
    xTaskCreate(tcp_client, "tcp_client", 4096, NULL, 5, NULL); // Create the TCP client task
}