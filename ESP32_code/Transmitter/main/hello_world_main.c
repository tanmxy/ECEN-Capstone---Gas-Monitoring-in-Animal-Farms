#include <stdio.h> //for basic printf commands
#include <string.h> //for handling strings

#include "freertos/FreeRTOS.h" //for delay,mutexs,semphrs rtos operations
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h" //esp_wifi_init functions and wifi operations
#include "esp_log.h" //for showing logs
#include "esp_mac.h" 
#include "esp_event.h" //for wifi event
#include "esp_system.h" //esp_init funtions esp_err_t 

#include "lwip/sockets.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h" //light weight ip packets error handling
#include "lwip/sys.h" //system applications for light weight ip apps

#include "nvs_flash.h" //non volatile storage
#include "driver/i2c.h" // I2C communication
#include "si7021.h" // Custom Si7021 library
#include "ADC.h" // My ADC simulation

#define PORT 3333 // Defining port to connect to on the server
#define EXAMPLE_ESP_WIFI_SSID "ESP32-Access-Point" // Defining my Wi-Fi SSID
#define EXAMPLE_ESP_WIFI_PASS "joaquinsalas" // Defining the Wi-Fi password
#define EXAMPLE_ESP_WIFI_CHANNEL    1 // Wi-Fi channel to operate on
#define EXAMPLE_MAX_STA_CONN        4 // Max number of devices to connect to the Access Point
#define BLUE_LED_GPIO GPIO_NUM_19  // Define GPIO pin for blue LED
#define GREEN_LED_GPIO GPIO_NUM_18  // Define GPIO pin for green LED

static const char *TAG = "TCP_SOCKET_SERVER"; // Log tag to indentify logs related to this module

// Global variables for sensor data
float temperature;
float humidity;
float ammonia;
float h2s;
float co2;
float methane;

// Function to blink LED
void blink_led(gpio_num_t gpio, int delay_ms, int blinks) { // Create the inputs for when I call the function
    for (int i = 0; i < blinks; i++) {
        gpio_set_level(gpio, 1); // turn LED on
        vTaskDelay(delay_ms / portTICK_PERIOD_MS); // Delay for the blink duration
        gpio_set_level(gpio, 0); // turn LED off
        vTaskDelay(delay_ms / portTICK_PERIOD_MS); // Delay for the blink duration
    }
}

// Event handler for WiFi for different Wi-Fi events
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) { // If a client connects to the Access Point
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) { // A client disconnecting from the Access Point
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d, reason=%d", MAC2STR(event->mac), event->aid, event->reason);
    }
}

// Setup Soft AP mode
void wifi_init_softap(void) {

    ESP_ERROR_CHECK(esp_netif_init()); // Initialize esp32 network interface and create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // Default Wi-Fi configuration
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)); // Initialize Wi-Fi with the default config
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, 
                    &wifi_event_handler, NULL, NULL)); // Register Wi-Fi event handler

    wifi_config_t wifi_config = {

        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID, // Set SSID for AP
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID), // SSID length
            .channel = EXAMPLE_ESP_WIFI_CHANNEL, // Wi-Fi channel
            .password = EXAMPLE_ESP_WIFI_PASS, // Wi-Fi password
            .max_connection = EXAMPLE_MAX_STA_CONN, // Max number of client connections
            .authmode = WIFI_AUTH_WPA2_PSK, // Set WPA2 PSK authentication
            .pmf_cfg = {.required = true}, // Set Protected Management Frames
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) { // For a case when no password, in my case there is
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP)); // Set device as an access point
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config)); // Set AP configuration

    // Blink green LED rapidly to indicate Wi-Fi initialization
    blink_led(GREEN_LED_GPIO, 100, 10);

    ESP_ERROR_CHECK(esp_wifi_start()); // Start the Wi-Fi

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
                        EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL); // Log the Wi-Fi setup completion
}

static void tcp_server_task(void *pvParameters) {
    char addr_str[128];
    int addr_family = (int)pvParameters; // Set address family as IPv4
    int ip_protocol = IPPROTO_IP; // Set IP protocol to IPv4

    struct sockaddr_in dest_addr; // Define the destination address structure
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY); // Set address to any (so that server accepts any incoming connections)
    dest_addr.sin_family = AF_INET; // Use IPv4
    dest_addr.sin_port = htons(PORT); // Set port for connection

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol); // Create TCP socket
    if (listen_sock < 0) { // Check for socket creation faliure
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno); // Log if error
        vTaskDelete(NULL); // Delete tasks when theres an error
        return;
    }
    ESP_LOGI(TAG, "Socket created"); // Log if successful

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)); // Bind the socket to the address
    if (err != 0) { // Check if the bind fails
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno); // Log error
        close(listen_sock); // Close the socket
        vTaskDelete(NULL); // Delete task when theres an error
        return;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT); // Log successful binding

    err = listen(listen_sock, 1); // Listen for incoming connections
    if (err != 0) { // Check if listening fails
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno); // Log if error
        close(listen_sock); // Close the socket
        vTaskDelete(NULL); // Delete task on error
        return;
    }

    while (1) { // Infinite loop to take client connections
        ESP_LOGI(TAG, "Socket listening"); // Log that the server is listening for connections

        struct sockaddr_storage source_addr; // Address of the incoming client
        socklen_t addr_len = sizeof(source_addr); // Length of address
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len); // Accept incoming connection
        if (sock < 0) { // Check if accept fails
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno); // Log error
            break;
        }
        ESP_LOGI(TAG, "Socket accepted"); // Log successful connection acceptance

        while (1) { // Infinite loop to handle data transmission with client 
            char data_to_send[128];
            snprintf(data_to_send, sizeof(data_to_send), // Format the sensor data as shown below
                     "Temp:%.2f,Humidity:%.2f,NH3:%.2f,H2S:%.2f,CO2:%.2f,CH4:%.2f",
                     temperature, humidity, ammonia, h2s, co2, methane);

            int err = send(sock, data_to_send, strlen(data_to_send), 0); // Send the data to the client
            if (err < 0) { // Check if sending fails
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno); // Log the error
                break;
            }
            ESP_LOGI(TAG, "Data sent: %s", data_to_send); // Log the sent data
            vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait for 5 seconds before sending data again
        }

        if (sock != -1) { // Check if the socket is valid
            ESP_LOGE(TAG, "Shutting down socket and restarting..."); // Log that the socket is being closed
            shutdown(sock, 0); // Shutdown the socket
            close(sock); // Close the socket
        }
    }

    close(listen_sock); // Close the listening socket when done
    vTaskDelete(NULL); // Delete task after completing its work
}

// Need to create a task to read the sensor data
void sensor_task(void *pvParameters) {
    // Initialize the Si7021 sensor with the I2C port and pins
    esp_err_t ret = si7021_init(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, GPIO_PULLUP_ENABLE, GPIO_PULLUP_ENABLE);
    int retires = 0; // To take count of retry attempts
    while (ret != ESP_OK && retires < 5) {
        ESP_LOGE("SI7021", "Failed to initialize Si7021 sensor, retrying...");
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Retry after a second
        ret = si7021_init(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22, GPIO_PULLUP_ENABLE, GPIO_PULLUP_ENABLE);
        retires++;
    }
    if (ret != ESP_OK) { // Check if the sensor initialization fails
        ESP_LOGE("SI7021", "Failed to initialize Si7021 sensor after multiple attempts, error code: %d", ret);
        vTaskDelete(NULL); // Delete task if sensor initialization fails
    }
    // Infinite loop to keep reading and printing sensor data
    while (1) {
        // Read temperature and humidity using the library functions
        float temperature_c = si7021_read_temperature();
        temperature = (temperature_c * 9.0 / 5.0) + 32.0; // Converting from Celsius to Farenheit
        humidity = si7021_read_humidity(); // Read humidity

        // Log the temperature and humidity values to the terminal
        ESP_LOGI("SI7021", "Temperature: %.2f°F, Humidity: %.2f%%", temperature, humidity);

        // Wait for 5 seconds before the next reading
        vTaskDelay(5000 / portTICK_PERIOD_MS);  // Adjust the delay time as needed
    }
}

// Task for reading sensor data from csv file
void csv_task(void *pvParameters) {
    //Infinite loop to continuously read from the CSV file
    while(1) {
        read_sensor_data_csv(); // Read data from CSV file

        // Get latest sensor data
        sensor_data_t data = get_current_data();

        // Blink blue LED on each data retrival
        blink_led(BLUE_LED_GPIO, 500, 1);

        // Update global variables with latest sensor readings
        ammonia = data.ammonia;
        h2s = data.h2s;
        co2 = data.co2;
        methane = data.methane;

        vTaskDelay(5000 / portTICK_PERIOD_MS); // Delay to prevent constant readings
    }
}

// Main application entry point
void app_main() {

    // Initialize Non-Volatile Storage for esp32
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); // Erase NVS if theres an error with free pages or version mismatch
        ret = nvs_flash_init(); // Re-initialize NVS
    }
    ESP_ERROR_CHECK(ret);

    // Initialize LED GPIO
    esp_rom_gpio_pad_select_gpio(BLUE_LED_GPIO); // Select GPIO for blue LEDEC
    gpio_set_direction(BLUE_LED_GPIO, GPIO_MODE_OUTPUT); // Set blue LED GPIO as output
    esp_rom_gpio_pad_select_gpio(GREEN_LED_GPIO); // Select GPIO for green LED
    gpio_set_direction(GREEN_LED_GPIO, GPIO_MODE_OUTPUT); // Set green LED GPIO as output

    // Blink green LED to indicate system initialization
    blink_led(GREEN_LED_GPIO, 200, 5);

    // Initialize Wi-Fi access point
    wifi_init_softap();

    // Initialize sensors
    adc_init();  // Initialize CSV file reading for simulated sensors

    // Start TCP server task
    xTaskCreate(tcp_server_task, "tcp_server_task", 4096, (void *)AF_INET, 10, NULL);

    // Create separate tasks for reading sensors
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 9, NULL); // Sensor task for real sensor
    xTaskCreate(csv_task, "csv_task", 4096, NULL, 8, NULL); // CSV task for simulated sensor data
}