// Standard C Libraries
#include <stdio.h>       // printf commands
#include <string.h>      // String handling
#include <math.h>        // Math operations (pow, etc.)
#include <inttypes.h>    // PRIX32 format specifier

// FreeRTOS Libraries
#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"     
#include "freertos/queue.h"    
#include "freertos/semphr.h"  

// ESP System & Networking
#include "esp_system.h"   // esp_init functions, esp_err_t
#include "esp_wifi.h"     // Wi-Fi operations
#include "esp_event.h"    // Wi-Fi event handling
#include "esp_log.h"      // Logging
#include "esp_mac.h"      // MAC address functions
#include "nvs_flash.h"    // Non-volatile storage

// AWS IoT & MQTT Libraries
#include "network_transport.h"
#include "core_mqtt.h"
#include "core_mqtt_serializer.h"
#include "core_mqtt_state.h"
#include "transport_interface.h"
#include "backoff_algorithm.h"
#include "clock.h"        // Ensure clock utilities are included

// I2C Libraries (Sensor Communication)
// #include "driver/i2c.h"   // I2C communication
#include "esp_err.h"      // ESP error handling
#include "hdc1000.h"      // HDC1000 Temperature & Humidity Sensor
#include "scd4x.h"        // SCD4X CO2 Sensor
#include "i2cdev.h"       // I2C device handling
#include "esp_idf_lib_helpers.h" // (if needed by the drivers)

// SPI & GPIO (Sensor Communication)
#include "driver/spi_master.h"
#include "driver/gpio.h"

// Project-Specific Files
#include "wifi.h"         // Wi-Fi functionality
#include "sdkconfig.h"    // ESP-IDF Project Configuration

void mqtt_process_task(void *arg);
void set_wifi_status(bool connected);

#define APP_CPU_NUM PRO_CPU_NUM
#define I2C_MASTER_SDA GPIO_NUM_6
#define I2C_MASTER_SCL GPIO_NUM_4
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

// /* Pin Definitions */
#define AD7718_MOSI    GPIO_NUM_21  // MOSI
#define AD7718_MISO    GPIO_NUM_19  // MISO
#define AD7718_CLK     GPIO_NUM_18  // CLK
#define AD7718_CS      GPIO_NUM_5   // Chip Select
#define AD7718_RESET   GPIO_NUM_7   // Reset pin
#define AD7718_RDY     GPIO_NUM_9   // Data Ready pin

// AD7718 Register Addresses
#define AD7718_COMM_REG   0x00
#define AD7718_SETUP_REG  0x01
#define AD7718_CLOCK_REG  0x02
#define AD7718_DATA_REG   0x44

/* Command Definitions */
#define AD7718_RD 0x40  // Read command base (0x40 | register)
#define AD7718_WR 0x00  // Write command base

spi_device_handle_t ad7718_spi = NULL; // define SPI device handle 

static const char* TAG = "gasmonitor";

const char* topic = "sensor/reading";
const char* thing_name = "GasMonitor_ESP32";
const char* user_name = "user";
const char* endpoint = "a2etjzbib4sdus-ats.iot.us-east-2.amazonaws.com";
static char mqtt_string[128];  // Persistent memory for MQTT payload
float sensor_readings[6] = { 0 };
static bool adc_updated = false;
static bool scd_updated = false;
static SemaphoreHandle_t sensor_mutex;
uint16_t co2_scd4x = 0;
float temp_scd4x = 0;
float humidity_scd4x = 0;
float methane = 0;
float ammonia = 0;
float h2s = 0;

NetworkContext_t network_context; // cert + endpoint
TransportInterface_t transport; // func for send + recv
MQTTConnectInfo_t connect_info; // aws credentials
MQTTFixedBuffer_t network_buffer;
MQTTPublishInfo_t publish_info;
MQTTContext_t mqtt_context; // comm related - previous packs and timestamps

QueueHandle_t sensor_queue;
TaskHandle_t mqtt_task_handle; //mqtt task
QueueHandle_t mqtt_aws_queue;

bool ses_present = 0;
bool connected_to_aws = false;
bool data_ready = false;
uint8_t buffer[1024]; //for sensing data

extern const char root_cert_auth_start[] asm("_binary_root_cert_auth_crt_start");
extern const char root_cert_auth_end[] asm("_binary_root_cert_auth_crt_end");
extern const char client_cert_start[] asm("_binary_client_crt_start");
extern const char client_cert_end[] asm("_binary_client_crt_end");
extern const char client_key_start[] asm("_binary_client_key_start");
extern const char client_key_end[] asm("_binary_client_key_end");

void queue_mqtt_readings(void)
{
    // Only send if all sensor groups have updated
    if (scd_updated && adc_updated) {
        if (xQueueSend(sensor_queue, sensor_readings, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to send combined sensor data to queue.");
        }
        // Reset flags for the next update cycle
        scd_updated = false;
        adc_updated = false;
    }
}

void scd41_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(5000));  // Allow time for sensors to power up
    i2c_dev_t dev = { 0 };
    
    // Initialize the sensor descriptor
    ESP_ERROR_CHECK(scd4x_init_desc(&dev, 0, I2C_MASTER_SDA, I2C_MASTER_SCL));

    // ESP_ERROR_CHECK(scd4x_wake_up(&dev));
    ESP_ERROR_CHECK(scd4x_stop_periodic_measurement(&dev));
    ESP_ERROR_CHECK(scd4x_reinit(&dev));
    ESP_LOGI("SCD41", "Sensor initialized!");

    // uint16_t serial[3]; // Retrieve and log the serial number
    // ESP_ERROR_CHECK(scd4x_get_serial_number(&dev, serial, serial + 1, serial + 2));
    // ESP_LOGI("SCD41", "Sensor serial number: 0x%04x%04x%04x", serial[0], serial[1], serial[2]);

    // Start periodic measurements
    ESP_ERROR_CHECK(scd4x_start_periodic_measurement(&dev));
    vTaskDelay(pdMS_TO_TICKS(5000));

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Check if new data is available
        esp_err_t status = scd4x_get_data_ready_status(&dev, &data_ready);
        if (status != ESP_OK)
        {
            ESP_LOGE("SCD41", "Error checking data ready status: %s", esp_err_to_name(status));
            continue;
        }
        if (!data_ready)
        {
            ESP_LOGW("SCD41", "Data not ready (data_ready = %d), skipping measurement", data_ready);
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
        esp_err_t res = scd4x_read_measurement(&dev, &co2_scd4x, &temp_scd4x, &humidity_scd4x);
        if (res != ESP_OK)
        {
            ESP_LOGE("SCD41", "Error reading results: %s", esp_err_to_name(res));
            continue;
        }

        if (co2_scd4x == 0) // Check for invalid sample
        {
            ESP_LOGW("SCD41", "Invalid sample detected, skipping...");
            continue;
        }
        // ESP_LOGI("SCD41", "CO2: %u ppm, Temperature: %.2f°C, Humidity: %.2f%%", co2_scd4x, temp_scd4x, humidity_scd4x);

        // Update global sensor_readings and convert co2 to float
        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        sensor_readings[0] = temp_scd4x;
        sensor_readings[1] = humidity_scd4x;
        sensor_readings[2] = (float)co2_scd4x;
        scd_updated = true;
        queue_mqtt_readings();
        xSemaphoreGive(sensor_mutex);
    }
}
// ******************************* SCD41 *******************************

// ******************************* AD7718 *******************************
// Writes one byte to the specified register.
void ad7718_write(uint8_t address, uint8_t data) 
{
    esp_err_t ret;
    uint8_t tx_data[2] = {address, data}; // Store address + data in an array
    spi_transaction_t trans = {
        .length = 16, // Send 16 bits (2 bytes)
        .tx_buffer = tx_data, // Pointer to the data to transmit
    };
    ret = spi_device_transmit(ad7718_spi, &trans); // Perform SPI transaction
    assert(ret == ESP_OK); // make sure transaction is successful
}

// Reads 3 bytes (24-bit value) from a specified register and returns a 24-bit value.
uint32_t ad7718_read(uint8_t address)
{
    // Explanation
    // Single Transaction: The entire 32‑bit transfer happens with CS held low.
    // This meets the datasheet’s requirement that once the communications register is written
    // the ADC immediately begins outputting the 24‑bit conversion result.
    esp_err_t ret;
    uint8_t tx_buffer[4] = { address | AD7718_RD, 0x00, 0x00, 0x00 }; // Prepare a 4-byte transmit buffer: 1 byte command + 3 dummy bytes.
    uint8_t rx_buffer[4] = { 0 }; // Prepare a 4-byte receive buffer to store the incoming data.

    spi_transaction_t trans = { // Set up an SPI transaction for 32 bits (4 bytes)
        .length = 32, // Total of 32 bits: 8bit command + 24bit data
        .tx_buffer = tx_buffer, // Pointer to the transmit buffer
        .rx_buffer = rx_buffer, // Pointer to the receive buffer
    };

    ret = spi_device_transmit(ad7718_spi, &trans); // Assert CS for the entire transaction
    assert(ret == ESP_OK);

    // Now the first recieved byte is dummy, the next 3 bytes form the ADC data. 
    uint32_t adc_value = ((uint32_t)rx_buffer[1] << 16) | ((uint32_t)rx_buffer[2] << 8) | rx_buffer[3];
    return adc_value;
}

// Reset the AD7718 using GPIO
void ad7718_reset() 
{
    gpio_set_level(AD7718_RESET, 0); // Pull RESET pin LOW to reset AD7718
    vTaskDelay(pdMS_TO_TICKS(1));  // Wait for 1 ms
    gpio_set_level(AD7718_RESET, 1); // Pull RESET pin HIGH to exit reset state
    vTaskDelay(pdMS_TO_TICKS(10)); // wait 10 ms for reset to complete
}

// Initializes the SPI bus and adds the AD7718 device.
void ad7718_init_spi(void)
{
    esp_err_t ret;

    // Define SPI bus configuration
    spi_bus_config_t buscfg = {
        .mosi_io_num = AD7718_MOSI, // SPI MOSI pin
        .miso_io_num = AD7718_MISO, // SPI MISO pin
        .sclk_io_num = AD7718_CLK, // SPI Clock pin
        .quadwp_io_num = -1, // Not used
        .quadhd_io_num = -1, // Not used
        .max_transfer_sz = 32, // Max transfer size of 32 bytes
    };

    // Define SPI device configuration for AD7718
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, // 1 MHz
        .mode = 3, // SPI mode 3 (CPOL=1, CPHA=1) as per datasheet.
        // .spics_io_num = -1,                      // -1 for manual CS control OR AD7718_CS for automatic CS control
        .spics_io_num = AD7718_CS, // Automatic Chip Select pin for AD7718
        .queue_size = 1, // Queue size of 1 for transactions
    };

    // Initialize SPI bus
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK); // Stop if doesnt work
    
    // Attach AD7718 as an SPI device
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &ad7718_spi);
    assert(ret == ESP_OK); // Stop if doesnt work
}

void ad7718_task(void *pvParameters)
{
    // Configure the RESET pin as output and the RDY pin as input.
    gpio_set_direction(AD7718_RESET, GPIO_MODE_OUTPUT);
    gpio_set_direction(AD7718_RDY, GPIO_MODE_INPUT);

    // Initialize SPI and Reset AD7718
    ad7718_init_spi();
    ad7718_reset();

    // Configure the AD7718 for continuous conversion in a 10-channel mode.
    ad7718_write(AD7718_SETUP_REG, 0x33); // 0x33 configures continuous conversion, 10-channel mode.
    ad7718_write(0x03, 0x17); // Set ADC filter
    ESP_LOGI("AD7718", "AD7718 initialized!");
    vTaskDelay(pdMS_TO_TICKS(5000));

    const uint8_t ain_channels[3] = {0x07, 0x17, 0x27};
    const char *ain_labels[3] = {"AIN1", "AIN2", "AIN3"};
    float adc_voltages[3] = { 0 };

    while(1) 
    {
        for (int i = 0; i < 3; i++) { // Loop to cycle through AIN1, AIN2, AIN3
            ad7718_write(0x02, ain_channels[i]); // Select AINx
            vTaskDelay(pdMS_TO_TICKS(200));

            (void)ad7718_read(AD7718_DATA_REG); // dummy read to flush out old data

            while (gpio_get_level(AD7718_RDY) != 0) { // check if data is ready
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            uint32_t adc_value = ad7718_read(AD7718_DATA_REG);
            float voltage = ((((float)adc_value / pow(2, 23)) - 1) * 2.5) + 0.0278177268;
            float corrected_voltage = (voltage - 0.0287) / 0.9758; // linearizing voltage
            adc_voltages[i] = corrected_voltage;
            ESP_LOGI("AD7718", "%s Voltage: %.6f V", ain_labels[i], corrected_voltage);
        }

        // Update global sensor_readings 3, 4, 5
        xSemaphoreTake(sensor_mutex, portMAX_DELAY);
        methane = adc_voltages[0];
        ammonia = adc_voltages[1];
        h2s = adc_voltages[2];
        sensor_readings[3] = methane;
        sensor_readings[4] = ammonia;
        sensor_readings[5] = h2s;
        adc_updated = true;
        queue_mqtt_readings();
        xSemaphoreGive(sensor_mutex);
        vTaskDelay(pdMS_TO_TICKS(5000)); // read every second
    }
}
// ******************************* AD7718 *******************************

MQTTStatus_t mqtt_subscribe_to(MQTTContext_t* mqtt_context,const char* topic,MQTTQoS_t qos_level)
{
    uint16_t pkt = MQTT_GetPacketId(mqtt_context);
    MQTTSubscribeInfo_t subscribe_topic;
    subscribe_topic.qos = qos_level;
    subscribe_topic.pTopicFilter = topic;
    subscribe_topic.topicFilterLength = strlen(topic);
    return MQTT_Subscribe(mqtt_context,&subscribe_topic,1,pkt);
}

static void mqtt_event_cb(MQTTContext_t* pMQTTContext,MQTTPacketInfo_t* pPacketInfo, MQTTDeserializedInfo_t* pDeserializedInfo)
{
   switch(pPacketInfo->type)
   {
      case MQTT_PACKET_TYPE_PUBLISH : //Message received
        ESP_LOGI(TAG,"MQTT_PACKET_TYPE_PUBLISH");
        ESP_LOGI(TAG,"received :\n%s",(const char*)(pDeserializedInfo->pPublishInfo->pPayload));        break;
     case MQTT_PACKET_TYPE_SUBSCRIBE :
        ESP_LOGI(TAG,"MQTT_PACKET_TYPE_SUBSCRIBE");
        break;
    case MQTT_PACKET_TYPE_CONNECT  :
        ESP_LOGI(TAG,"MQTT_PACKET_TYPE_CONNECT");
        break;  
    case MQTT_PACKET_TYPE_CONNACK :
        ESP_LOGI(TAG,"MQTT_PACKET_TYPE_CONNACK");
        break;
        break;
    default:
    break;
   }
}

void mqtt_process_task(void* arg)
{
    ESP_LOGI(TAG,"mqtt_process_task started");
    mqtt_subscribe_to(&mqtt_context, "/read", MQTTQoS0);

    float local_sensor_readings[6];  // Local buffer for sensor data
    MQTTStatus_t status;
    
    while(1)
    {
        // Wait for sensor data
        // xQueueReceive(sensor_queue, sensor_readings, portMAX_DELAY);
        xQueueReceive(sensor_queue, local_sensor_readings, portMAX_DELAY);

        // Make sure MQTT is running
        // MQTT_ProcessLoop(&mqtt_context);
        // Check for MQTT connection errors

        // Check the MQTT processing loop for errors.
        status = MQTT_ProcessLoop(&mqtt_context);
        if (status != MQTTSuccess) {
            ESP_LOGE(TAG, "MQTT_ProcessLoop error: %d", status);
            // Clean up and attempt reconnection:
            MQTT_Disconnect(&mqtt_context);
            vTaskDelay(pdMS_TO_TICKS(2000));  // Allow network to settle
            set_wifi_status(true);            // Reinitialize TLS/MQTT connection
            vTaskDelete(NULL);                // Terminate this task so a new one can be created
        }

        // Format JSON message for AWS IoT
        snprintf(mqtt_string, sizeof(mqtt_string), 
                "{"
                "\"temperature\":%.2f, \"humidity\":%.2f, \"co2\":%.2f, "
                "\"AIN1\":%.6f, \"AIN2\":%.6f, \"AIN3\":%.6f"
                "}",
                local_sensor_readings[0], local_sensor_readings[1], local_sensor_readings[2],
                local_sensor_readings[3], local_sensor_readings[4], local_sensor_readings[5]);

        ESP_LOGI(TAG, "Publishing: %s", mqtt_string); // For testing

        publish_info.pPayload = mqtt_string;
        publish_info.payloadLength = strlen(mqtt_string);
        // uint16_t packet_id = MQTT_GetPacketId(&mqtt_context);
        // MQTT_Publish(&mqtt_context, &publish_info, packet_id);

        uint16_t packet_id = MQTT_GetPacketId(&mqtt_context);
        status = MQTT_Publish(&mqtt_context, &publish_info, packet_id);
        if (status != MQTTSuccess) {
            ESP_LOGE(TAG, "MQTT_Publish error: %d. Reconnecting...", status);
            MQTT_Disconnect(&mqtt_context);
            vTaskDelay(pdMS_TO_TICKS(2000));  // Allow time for the network to settle
            set_wifi_status(true);            // Reinitialize the TLS/MQTT connection
            vTaskDelete(NULL);                // Terminate this task so that a fresh task is created
        }
    }
}

void aws_init(void)
{
    network_context.pcHostname = endpoint;
    network_context.xPort = 8883;
    network_context.pxTls = NULL;
    network_context.xTlsContextSemaphore = xSemaphoreCreateMutex();
    network_context.disableSni = false;
    network_context.pcServerRootCA = root_cert_auth_start;
    network_context.pcServerRootCASize = root_cert_auth_end - root_cert_auth_start;
    network_context.pcClientCert = client_cert_start;
    network_context.pcClientCertSize = client_cert_end - client_cert_start;
    network_context.pcClientKey = client_key_start;
    network_context.pcClientKeySize = client_key_end - client_key_start;
    network_context.pAlpnProtos = NULL;
    transport.pNetworkContext = &network_context;
    transport.recv = espTlsTransportRecv;
    transport.send = espTlsTransportSend;
    transport.writev = NULL;
    network_buffer.pBuffer = buffer;
    network_buffer.size = sizeof(buffer);
    connect_info.cleanSession = true;
    connect_info.pClientIdentifier = thing_name;
    connect_info.clientIdentifierLength = strlen(thing_name);
    connect_info.keepAliveSeconds = 60;
    connect_info.pUserName = user_name;
    connect_info.userNameLength = strlen(user_name);
    publish_info.qos = MQTTQoS0;
    publish_info.pTopicName = topic;
    publish_info.topicNameLength = strlen(topic);
    MQTT_Init(&mqtt_context, &transport, Clock_GetTimeMs, &mqtt_event_cb, &network_buffer);
}

void set_wifi_status(bool connected)
{
    // Retry TLS handshake until successful
    int tlsErr = 0;
    do {
        tlsErr = xTlsConnect(&network_context);
        if (tlsErr != 0) {
            ESP_LOGE(TAG, "TLS connection failed with (error: %d). Retrying in 5 seconds...", tlsErr);
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    } while (tlsErr != 0);
    ESP_LOGI(TAG, "TLS connection successful");
    vTaskDelay(pdMS_TO_TICKS(500));

    // Retry MQTT connection until successful
    int mqttErr = 0;
    do {
        mqttErr = MQTT_Connect(&mqtt_context, &connect_info, NULL, 5000, &ses_present);
        if (mqttErr != MQTTSuccess) {
            ESP_LOGE(TAG, "MQTT connection failed with (error: %d). Retrying in 5 seconds...", mqttErr);
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    } while (mqttErr != MQTTSuccess);
    ESP_LOGI(TAG, "MQTT connection successful");
    vTaskDelay(pdMS_TO_TICKS(500));

    // Create the MQTT processing task
    xTaskCreate(mqtt_process_task, "mqtt_process_task", 2048 * 2, NULL, 9, &mqtt_task_handle);
}

void user_main_task(void* arg)
{
    ESP_LOGI(TAG,"main_task started");
    while(1)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// Main application entry point
void app_main() {
    esp_err_t ret;

    // Initialize Non-Volatile Storage for esp32
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); // Erase NVS if theres an error with free pages or version mismatch
        ret = nvs_flash_init(); // Re-initialize NVS
    }
    ESP_ERROR_CHECK(ret);

    // I2C Driver
    ESP_ERROR_CHECK(i2cdev_init());
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "I2C driver initialized!");

    sensor_mutex = xSemaphoreCreateMutex();
    if (sensor_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor mutex!");
        return;
    }

    // Create queues for MQTT & sensor data
    mqtt_aws_queue = xQueueCreate(10, sizeof(uint32_t));
    sensor_queue = xQueueCreate(5, sizeof(float[6]));
    if (!mqtt_aws_queue || !sensor_queue) {
        ESP_LOGE(TAG, "Queue creation failed! System out of memory.");
        return; // Prevent execution if queues failed
    }

    // Initialize Wi-Fi station mode
    wifi_init();
    while (wifi_connect() != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi not available, retrying...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    aws_init();
    set_wifi_status(true);

    xTaskCreatePinnedToCore(scd41_task, "scd41_task", configMINIMAL_STACK_SIZE * 8, NULL, 4, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(500));

    xTaskCreatePinnedToCore(ad7718_task, "ad7718_task", 4096, NULL, 3, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(500));

    // // // xTaskCreatePinnedToCore(mqtt_process_task, "mqtt_process_task", 4096, NULL, 9, &mqtt_task_handle, 1);
}