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

#include "freertos/semphr.h"

static bool hdc_updated = false;
static bool scd_updated = false;
static SemaphoreHandle_t sensor_mutex;

#define APP_CPU_NUM PRO_CPU_NUM
// #define I2C_MASTER_SDA GPIO_NUM_6
// #define I2C_MASTER_SCL GPIO_NUM_4 // For esp32-s3
#define I2C_MASTER_SDA GPIO_NUM_21
#define I2C_MASTER_SCL GPIO_NUM_22
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

// /* Pin Definitions */
// #define AD7718_MOSI    GPIO_NUM_21  // MOSI
// #define AD7718_MISO    GPIO_NUM_19  // MISO
// #define AD7718_CLK     GPIO_NUM_18  // CLK
// #define AD7718_CS      GPIO_NUM_5   // Chip Select
// #define AD7718_RESET   GPIO_NUM_7   // Reset pin
// #define AD7718_RDY     GPIO_NUM_9   // Data Ready pin

// // AD7718 Register Addresses
// #define AD7718_COMM_REG   0x00
// #define AD7718_SETUP_REG  0x01
// #define AD7718_CLOCK_REG  0x02
// #define AD7718_DATA_REG   0x44

// /* Command Definitions */
// #define AD7718_RD 0x40  // Read command base (0x40 | register)
// #define AD7718_WR 0x00  // Write command base

// spi_device_handle_t ad7718_spi = NULL; // define SPI device handle 

static const char* TAG = "gasmonitor";

const char* topic = "sensor/reading";
const char* thing_name = "GasMonitor_ESP32";
const char* user_name = "user";
const char* endpoint = "a2etjzbib4sdus-ats.iot.us-east-2.amazonaws.com";
static char mqtt_string[128];  // Persistent memory for MQTT payload
float sensor_readings[5] = { 0 };
float temp_hdc1000 = 0;
float humidity_hdc1000 = 0;
uint16_t co2_scd4x = 0;
float temp_scd4x = 0;
float humidity_scd4x = 0;


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

// ******************************* HDC1080 & SCD41 *******************************

void queue_mqtt_readings(float temp_hdc1000, float humidity_hdc1000, float temp_scd4x, float humidity_scd4x, uint16_t co2_scd4x)
{
    static float latest_temp_hdc1000, latest_humidity_hdc1000;
    static float latest_temp_scd4x, latest_humidity_scd4x;
    static uint16_t latest_co2_scd4x;

    // Lock before modifying shared flags
    xSemaphoreTake(sensor_mutex, portMAX_DELAY);

    // Determine which sensor sent the data
    if (temp_hdc1000 >= 0 && humidity_hdc1000 >= 0)
    {
        latest_temp_hdc1000 = temp_hdc1000;
        latest_humidity_hdc1000 = humidity_hdc1000;
        hdc_updated = true;  // Mark HDC1080 as updated
    }
    if (temp_scd4x >= 0 && humidity_scd4x >= 0 && co2_scd4x > 0)
    {
        latest_temp_scd4x = temp_scd4x;
        latest_humidity_scd4x = humidity_scd4x;
        latest_co2_scd4x = co2_scd4x;
        scd_updated = true;  // Mark SCD41 as updated
    }

    // If both sensors have updated, publish data
    if (hdc_updated && scd_updated)
    {
        sensor_readings[0] = latest_temp_hdc1000;
        sensor_readings[1] = latest_humidity_hdc1000;
        sensor_readings[2] = latest_temp_scd4x;
        sensor_readings[3] = latest_humidity_scd4x;
        sensor_readings[4] = latest_co2_scd4x;

        // Send to the queue only once
        xQueueSend(sensor_queue, sensor_readings, 100);

        // Reset flags for the next reading cycle
        hdc_updated = false;
        scd_updated = false;
    }

    // Release the lock
    xSemaphoreGive(sensor_mutex);
}

void hdc1080_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(5500));  // Allow time for sensors to power up
    hdc1000_t dev = { 0 };

    ESP_ERROR_CHECK(hdc1000_init_desc(&dev, HDC1000_I2C_ADDRESS_0, I2C_MASTER_NUM, I2C_MASTER_SDA, I2C_MASTER_SCL));
    ESP_ERROR_CHECK(hdc1000_init(&dev));

    uint32_t serial[2];
    uint16_t manuf_id, dev_id;

    ESP_ERROR_CHECK(hdc1000_get_serial(&dev, (uint64_t *)serial));
    ESP_ERROR_CHECK(hdc1000_get_manufacturer_id(&dev, &manuf_id));
    ESP_ERROR_CHECK(hdc1000_get_device_id(&dev, &dev_id));
    ESP_LOGI("HDC1080", "Sensor initialized!");

    ESP_LOGI("HDC1080", "Manufacturer ID: 0x%04x, Device ID: 0x%04x, Serial: 0x%08" PRIx32 "%08" PRIx32,
            manuf_id, dev_id, serial[0], serial[1]);

    vTaskDelay(pdMS_TO_TICKS(5000));
    while (1)
    {
        esp_err_t res = hdc1000_measure(&dev, &temp_hdc1000, &humidity_hdc1000);
        if (res == ESP_OK)
        {
            ESP_LOGI("HDC1080", "Temperature: %.2f°C, Humidity: %.2f%%", temp_hdc1000, humidity_hdc1000);
        }
        else
        {
            ESP_LOGE("HDC1080", "Error reading data: %s", esp_err_to_name(res));
        }
        queue_mqtt_readings(temp_hdc1000, humidity_hdc1000, temp_scd4x, humidity_scd4x, co2_scd4x);
        vTaskDelay(pdMS_TO_TICKS(5000));
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

    // Retrieve and log the serial number
    uint16_t serial[3];
    ESP_ERROR_CHECK(scd4x_get_serial_number(&dev, serial, serial + 1, serial + 2));
    ESP_LOGI("SCD41", "Sensor serial number: 0x%04x%04x%04x", serial[0], serial[1], serial[2]);

    // Start periodic measurements
    ESP_ERROR_CHECK(scd4x_start_periodic_measurement(&dev));

    vTaskDelay(pdMS_TO_TICKS(5000));
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000)); // Delay to match the sensor's update rate

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
        ESP_LOGI("SCD41", "CO2: %u ppm, Temperature: %.2f°C, Humidity: %.2f%%", co2_scd4x, temp_scd4x, humidity_scd4x);
        queue_mqtt_readings(temp_hdc1000, humidity_hdc1000, temp_scd4x, humidity_scd4x, co2_scd4x);
    }
}
// ******************************* HDC1080 & SCD41 *******************************

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
    mqtt_subscribe_to(&mqtt_context,"/read",MQTTQoS0);
    
    while(1)
    {
        // Wait for sensor data
        xQueueReceive(sensor_queue, sensor_readings, portMAX_DELAY);

        // Make sure MQTT is running
        MQTT_ProcessLoop(&mqtt_context);

        // Format JSON message for AWS IoT
        snprintf(mqtt_string, sizeof(mqtt_string), 
                "{"
                "\"temperature_hdc1000\":%.2f, \"humidity_hdc1000\":%.2f, "
                "\"temperature_scd41\":%.2f, \"humidity_scd41\":%.2f, "
                "\"co2_scd41\":%u"
                "}",
                sensor_readings[0], sensor_readings[1], sensor_readings[2], sensor_readings[3], (uint16_t)sensor_readings[4]);

        ESP_LOGI(TAG, "Publishing: %s", mqtt_string); // For testing

        publish_info.pPayload = mqtt_string;
        publish_info.payloadLength = strlen(mqtt_string);
        uint16_t packet_id = MQTT_GetPacketId(&mqtt_context);
        MQTT_Publish(&mqtt_context,&publish_info,packet_id);
        
    }
}

void set_wifi_status(bool connected)
{
    ESP_LOGI(TAG,"tls err : %d",xTlsConnect(&network_context));
    ESP_LOGI(TAG,"MQTT Connect err : %d",MQTT_Connect(&mqtt_context,&connect_info,NULL,1000,&ses_present));
    xTaskCreate(mqtt_process_task,"mqtt_process_task", 2048 * 2,NULL,5,&mqtt_task_handle);
   // connected_to_aws = MQTT_Connect(&mqtt_context,&connect_info,NULL,2000,&ses_present);
    
}

void user_main_task(void* arg)
{
    ESP_LOGI(TAG,"main_task started");
    while(1)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
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
    MQTT_Init(&mqtt_context,&transport,Clock_GetTimeMs,&mqtt_event_cb,&network_buffer);
}

// ******************************* AD7718 *******************************

// // Writes one byte to the specified register.
// void ad7718_write(uint8_t address, uint8_t data) 
// {
//     esp_err_t ret;
//     uint8_t tx_data[2] = {address, data}; // Store address + data in an array
//     spi_transaction_t trans = {
//         .length = 16, // Send 16 bits (2 bytes)
//         .tx_buffer = tx_data, // Pointer to transmit data
//     };
//     ret = spi_device_transmit(ad7718_spi, &trans); // Perform SPI transaction
//     assert(ret == ESP_OK); // make sure transaction is successful
// }

// // Reads 3 bytes (24-bit value) from a specified register and returns a 24-bit value.
// uint32_t ad7718_read(uint8_t address)
// {
//     // Explanation
//     // Single Transaction: The entire 32‑bit transfer happens with CS held low.
//     // This meets the datasheet’s requirement that once the communications register is written
//     // the ADC immediately begins outputting the 24‑bit conversion result.
//     esp_err_t ret;
//     uint8_t tx_buffer[4] = { address | AD7718_RD, 0x00, 0x00, 0x00 };
//     uint8_t rx_buffer[4] = { 0 };

//     spi_transaction_t trans = {
//         .length = 32, // Total of 32 bits: 8bit command + 24bit data
//         .tx_buffer = tx_buffer,
//         .rx_buffer = rx_buffer,
//     };

//     // Assert CS for the entire transaction
//     ret = spi_device_transmit(ad7718_spi, &trans);
//     assert(ret == ESP_OK);

//     // Now the first recieved byte is dummy, the next 3 bytes form the ADC data. 
//     uint32_t adc_value = ((uint32_t)rx_buffer[1] << 16) | ((uint32_t)rx_buffer[2] << 8) | rx_buffer[3];

//     ESP_LOGI("AD7718", "Raw ADC Data: %lu", adc_value);

//     return adc_value;
// }

// // Reset the AD7718 using GPIO
// void ad7718_reset() 
// {
//     gpio_set_level(AD7718_RESET, 0); // Pull RESET pin LOW to reset AD7718
//     vTaskDelay(pdMS_TO_TICKS(1));  // Wait for 1 ms
//     gpio_set_level(AD7718_RESET, 1); // Pull RESET pin HIGH to exit reset state
//     vTaskDelay(pdMS_TO_TICKS(10)); // wait 10 ms for reset to complete
// }

// // Initializes the SPI bus and adds the AD7718 device.
// void ad7718_init_spi(void)
// {
//     esp_err_t ret;

//     // Define SPI bus configuration
//     spi_bus_config_t buscfg = {
//         .mosi_io_num = AD7718_MOSI, // SPI MOSI pin
//         .miso_io_num = AD7718_MISO, // SPI MISO pin
//         .sclk_io_num = AD7718_CLK, // SPI Clock pin
//         .quadwp_io_num = -1, // Not used
//         .quadhd_io_num = -1, // Not used
//         .max_transfer_sz = 32, // Max transfer size of 32 bytes
//     };

//     // Define SPI device configuration for AD7718
//     spi_device_interface_config_t devcfg = {
//         .clock_speed_hz = 1000000, // 1 MHz
//         .mode = 3,                 // SPI mode 1 per AD7718 datasheet               /////// Change this to 3
//         // .spics_io_num = -1,                      // -1 for manual CS control OR AD7718_CS for automatic CS control
//         .spics_io_num = AD7718_CS, // Automatic Chip Select pin for AD7718
//         .queue_size = 1, // Queue size of 1 for transactions
//     };

//     // Initialize SPI bus
//     ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
//     assert(ret == ESP_OK); // Stop if doesnt work
    
//     // Attach AD7718 as an SPI device
//     ret = spi_bus_add_device(SPI2_HOST, &devcfg, &ad7718_spi);
//     assert(ret == ESP_OK); // Stop if doesnt work
// }

// void ad7718_task(void *pvParameters)
// {
//     // Configure GPIO
//     gpio_set_direction(AD7718_RESET, GPIO_MODE_OUTPUT);
//     gpio_set_direction(AD7718_RDY, GPIO_MODE_INPUT);

//     // Initialize SPI and Reset AD7718
//     ad7718_init_spi();
//     ad7718_reset();

//     // AD7718 Configuration
//     ESP_LOGI("AD7718", "Setting ADC Mode to Continuous");
//     ad7718_write(AD7718_SETUP_REG, 0x33); // Continuous conversion, 10-channel mode

//     ESP_LOGI("AD7718", "Setting ADC Filter Register");
//     ad7718_write(0x03, 0x17); // Set ADC filter

//     ESP_LOGI("AD7718", "Configuring input channels...");

//     ad7718_write(0x02, 0x07);

//     // Main loop to read ADC values
//     while(1) {
//         if (gpio_get_level(AD7718_RDY) == 0) { // Check if data is ready

//             ESP_LOGI("AD7718", "Data Ready - Reading ADC Value...");

//             // Read raw ADC value
//             // uint8_t raw_data[3] = {0};
//             uint32_t adc_value = ad7718_read(AD7718_DATA_REG); // Issue may be here

//             ESP_LOGI("AD7718", "Raw ADC Data: 0x%08" PRIX32, adc_value);
//             ESP_LOGI("AD7718", "ADC Decimal Value: %" PRIu32, adc_value);

//             // Convert to voltage (assuming 2.5V reference and 24-bit ADC)
//             float voltage = ((((float)adc_value / pow(2, 23)) - 1) * 2.5 + 0.0278177268);
//             ESP_LOGI("AD7718", "Converted Voltage: %.6f V", voltage);
//         }
//         vTaskDelay(pdMS_TO_TICKS(1000)); // read every second
//     }
// }
// ******************************* AD7718 *******************************


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
    // sensor_queue = xQueueCreate(5, sizeof(float[2])); // Only stores temp & humidity
    sensor_queue = xQueueCreate(5, sizeof(float[5]));
    if (!mqtt_aws_queue || !sensor_queue) {
        ESP_LOGE(TAG, "Queue creation failed! System out of memory.");
        return; // Prevent execution if queues failed
    }

    // Initialize Wi-Fi station mode
    wifi_init();
    if (wifi_connect() != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi not available. Stopping program.");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // Perform TLS Handshake
    aws_init();
    int tlsErr = xTlsConnect(&network_context); // TLS handshake
    if (tlsErr != 0)
    {
        ESP_LOGE(TAG, "TLS connection failed with error: %d", tlsErr);
        return;
    }
    ESP_LOGI(TAG, "TLS connection successful!");

    // Connect to MQTT Broker
    int mqttErr = MQTT_Connect(&mqtt_context, &connect_info, NULL, 5000, &ses_present);
    if (mqttErr != MQTTSuccess)
    {
        ESP_LOGE(TAG, "MQTT connection failed with error: %d", mqttErr);
        return;
    }
    ESP_LOGI(TAG, "MQTT connection successful!");

    // xTaskCreatePinnedToCore(hdc1080_task, "hdc1080_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(hdc1080_task, "hdc1080_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    // xTaskCreatePinnedToCore(scd41_task, "scd41_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(scd41_task, "scd41_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    // xTaskCreatePinnedToCore(ad7718_task, "ad7718_task", 4096, NULL, 10, NULL, 1);
    // vTaskDelay(pdMS_TO_TICKS(500));
    xTaskCreatePinnedToCore(mqtt_process_task, "mqtt_process_task", 4096, NULL, 9, &mqtt_task_handle, 1);
}