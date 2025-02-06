// Commented out code is for future Sensor Integrations

#include <stdio.h> //for basic printf commands
#include <string.h> //for handling strings

#include "freertos/FreeRTOS.h" //for delay,mutexs,semphrs rtos operations
#include "freertos/task.h"

#include "esp_wifi.h" //esp_wifi_init functions and wifi operations
#include "esp_log.h" //for showing logs
#include "esp_event.h" //for wifi event
#include "esp_system.h" //esp_init funtions esp_err_t 
#include "esp_mac.h"  // Include esp_mac.h for esp_read_mac()

#include "nvs_flash.h" //non volatile storage
#include "si7021.h" // Custom Si7021 library
#include "esp_system.h"
#include "wifi.h"

#include "freertos/semphr.h"
#include "network_transport.h"
#include "sdkconfig.h"
#include "freertos/queue.h"

#include "core_mqtt.h"
#include "core_mqtt_serializer.h"
#include "core_mqtt_state.h"
#include "transport_interface.h"
#include "backoff_algorithm.h"
#include "clock.h"  // Ensure clock utilities are included

// I2C Libraries
#include "driver/i2c.h" // I2C communication
#include "esp_err.h"
#include "hdc1000.h"
#include "scd4x.h"    
#include "i2cdev.h"    // From the i2cdev component
#include "esp_idf_lib_helpers.h" // (if needed by the drivers)

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <math.h>  // Include this for pow function
#include <inttypes.h>  // Include this for PRIX32 format specifier

#define WIFI_MAXIMUM_RETRY          10

static const char* TAG = "gasmonitor";
const char* topic = "sensor/reading";
const char* thing_name = "GasMonitor_ESP32";
const char* user_name = "user";
const char* endpoint = "a2etjzbib4sdus-ats.iot.us-east-2.amazonaws.com";
static char mqtt_string[128];  // Persistent memory for MQTT payload
float sensor_readings[5];
// float temp_hdc1000;
// float humidity_hdc1000;
// uint16_t co2;
// float temp_scd4x;
// float humidity_scd4x;

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
uint8_t buffer[1024]; //for sensing data

extern const char root_cert_auth_start[] asm("_binary_root_cert_auth_crt_start");
extern const char root_cert_auth_end[] asm("_binary_root_cert_auth_crt_end");
extern const char client_cert_start[] asm("_binary_client_crt_start");
extern const char client_cert_end[] asm("_binary_client_crt_end");
extern const char client_key_start[] asm("_binary_client_key_start");
extern const char client_key_end[] asm("_binary_client_key_end");

/*******************************************************************/
MQTTStatus_t mqtt_subscribe_to(MQTTContext_t* mqtt_context,const char* topic,MQTTQoS_t qos_level)
{
    uint16_t pkt = MQTT_GetPacketId(mqtt_context);
    MQTTSubscribeInfo_t subscribe_topic;
    subscribe_topic.qos = qos_level;
    subscribe_topic.pTopicFilter = topic;
    subscribe_topic.topicFilterLength = strlen(topic);
    return MQTT_Subscribe(mqtt_context,&subscribe_topic,1,pkt);
}

// ******************************* HDC1080 & SCD41 *******************************

#define APP_CPU_NUM PRO_CPU_NUM
#define I2C_MASTER_SDA GPIO_NUM_6
#define I2C_MASTER_SCL GPIO_NUM_4
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

void scan_i2c_bus()
{
    ESP_LOGI("I2C", "Scanning I2C bus...");
    for (uint8_t address = 1; address < 127; address++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
            ESP_LOGI("I2C", "Device found at address: 0x%02X", address);
    }
    ESP_LOGI("I2C", "I2C scan complete.");
}

void hdc1080_task(void *pvParameters)
{
    hdc1000_t dev = { 0 };

    ESP_LOGI("I2C", "Initializing HDC1080...");
    ESP_ERROR_CHECK(hdc1000_init_desc(&dev, HDC1000_I2C_ADDRESS_0, I2C_MASTER_NUM, I2C_MASTER_SDA, I2C_MASTER_SCL));
    ESP_ERROR_CHECK(hdc1000_init(&dev));

    uint32_t serial[2];
    uint16_t manuf_id, dev_id;

    ESP_ERROR_CHECK(hdc1000_get_serial(&dev, (uint64_t *)serial));
    ESP_ERROR_CHECK(hdc1000_get_manufacturer_id(&dev, &manuf_id));
    ESP_ERROR_CHECK(hdc1000_get_device_id(&dev, &dev_id));

    ESP_LOGI("I2C", "HDC1000, Manufacturer ID: 0x%04x, Device ID: 0x%04x, Serial: 0x%08" PRIx32 "%08" PRIx32,
            manuf_id, dev_id, serial[0], serial[1]);

    // float hdc1080_readings[2];
    // temp_hdc1000 = 0;
    // humidity_hdc1000 = 0;
    // float sensor_readings[5] = {0};
    sensor_readings[5] = {0};

    while (1)
    {
        esp_err_t res = hdc1000_measure(&dev, &sensor_readings[0], &sensor_readings[1]);
        if (res == ESP_OK)
        {
            ESP_LOGI("I2C", "Temperature: %.2f°C, Humidity: %.2f%%", sensor_readings[0], sensor_readings[1]);
        }
        else
        {
            ESP_LOGE("I2C", "Error reading data: %s", esp_err_to_name(res));
        }

        // Send readings to aws
        // hdc1080_readings[0] = temp_hdc1000;
        // hdc1080_readings[1] = humidity_hdc1000;
        xQueueSend(sensor_queue, sensor_readings, 100);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void scd41_task(void *pvParameters)
{
    i2c_dev_t dev = { 0 };

    ESP_LOGI("I2C", "Initializing SCD41...");
    
    // Initialize the sensor descriptor
    ESP_ERROR_CHECK(scd4x_init_desc(&dev, 0, I2C_MASTER_SDA, I2C_MASTER_SCL));

    ESP_LOGI("I2C", "Initializing sensor...");
    // ESP_ERROR_CHECK(scd4x_wake_up(&dev));
    ESP_ERROR_CHECK(scd4x_stop_periodic_measurement(&dev));
    ESP_ERROR_CHECK(scd4x_reinit(&dev));
    ESP_LOGI("I2C", "Sensor initialized");

    // Retrieve and log the serial number
    uint16_t serial[3];
    ESP_ERROR_CHECK(scd4x_get_serial_number(&dev, serial, serial + 1, serial + 2));
    ESP_LOGI("I2C", "Sensor serial number: 0x%04x%04x%04x", serial[0], serial[1], serial[2]);

    // Start periodic measurements
    ESP_ERROR_CHECK(scd4x_start_periodic_measurement(&dev));
    ESP_LOGI("I2C", "Periodic measurements started");

    uint16_t co2 = 0;
    // temp_scd4x = 0;
    // humidity_scd4x = 0;
    // float scd41_readings[3];
    float sensor_readings[5] = {0};  // Use the same structure for MQTT queue
    bool data_ready = false;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(6000)); // Delay to match the sensor's update rate

        // Check if new data is available
        esp_err_t status = scd4x_get_data_ready_status(&dev, &data_ready);
        if (status != ESP_OK)
        {
            ESP_LOGE("I2C", "Error checking data ready status: %s", esp_err_to_name(status));
            continue;
        }
        if (!data_ready)
        {
            ESP_LOGW("I2C", "Data not ready, skipping measurement");
            continue;
        }

        esp_err_t res = scd4x_read_measurement(&dev, &co2, &sensor_readings[2], &sensor_readings[3]);
        if (res != ESP_OK)
        {
            ESP_LOGE("I2C", "Error reading results: %s", esp_err_to_name(res));
            continue;
        }

        if (co2 == 0) // Check for invalid sample
        {
            ESP_LOGW("I2C", "Invalid sample detected, skipping...");
            continue;
        }
        sensor_readings[4] = co2;
        ESP_LOGI("I2C", "CO2: %u ppm, Temperature: %.2f°C, Humidity: %.2f%%", co2, sensor_readings[2], sensor_readings[3]);

        // scd41_readings[0] = temp_scd4x;
        // scd41_readings[1] = humidity_scd4x;
        xQueueSend(sensor_queue, sensor_readings, 100);
    }
}
// ******************************* HDC1080 & SCD41 *******************************

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
    default:break;
   }

}

void mqtt_process_task(void* arg)
{
    // float sensor_readings[5];
    ESP_LOGI(TAG,"mqtt_process_task started");
    mqtt_subscribe_to(&mqtt_context,"/read",MQTTQoS0);
    
    while(1)
    {
        // Wait for sensor data
        xQueueReceive(sensor_queue,sensor_readings,portMAX_DELAY);

        // Make sure MQTT is running
        MQTT_ProcessLoop(&mqtt_context);

        // Format JSON message for AWS IoT
        snprintf(mqtt_string, sizeof(mqtt_string), 
                "{"
                "\"temperature_hdc1000\":%.2f, \"humidity_hdc1000\":%.2f, "
                "\"temperature_scd41\":%.2f, \"humidity_scd41\":%.2f, "
                "\"co2\":%u"
                "}",
                sensor_readings[0], sensor_readings[1],  // HDC1080 Temp & Humidity
                sensor_readings[2], sensor_readings[3],  // SCD41 Temp & Humidity
                (uint16_t)sensor_readings[4]);           // CO2 in PPM

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
    ESP_LOGI(TAG,"user_main_task started");
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
/* Pin Definitions */
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

// Writes one byte to the specified register.
void ad7718_write(uint8_t address, uint8_t data) 
{
    esp_err_t ret;
    uint8_t tx_data[2] = {address, data}; // Store address + data in an array
    spi_transaction_t trans = {
        .length = 16, // Send 16 bits (2 bytes)
        .tx_buffer = tx_data, // Pointer to transmit data
    };
    ret = spi_device_transmit(ad7718_spi, &trans); // Perform SPI transaction
    assert(ret == ESP_OK); // make sure transaction is successful
}

// Reads 3 bytes (24-bit value) from a specified register and returns a 24-bit value.
uint32_t ad7718_read(uint8_t address)
{
    esp_err_t ret;
    uint8_t tx_data = address | 0x40;  // Set up read command by ORing address with '0x40'
    uint8_t rx_data[3] = {0}; // Buffer to store received data (3 bytes)

    // First transaction - send the read command
    spi_transaction_t trans = {
        .length = 8,  // Send 1 byte (8-bit register address)
        .tx_buffer = &tx_data, // Pointer to the transmit buffer
        .flags = SPI_TRANS_USE_RXDATA, // Enable RX buffer usage
    };

    gpio_set_level(AD7718_CS, 0);
    ret = spi_device_transmit(ad7718_spi, &trans); // send read command
    gpio_set_level(AD7718_CS, 1);

    assert(ret == ESP_OK); // Stop excecution if transaction fails

    vTaskDelay(pdMS_TO_TICKS(1)); // Small delay before reading data

    // Second transaction - read 3 bytes from AD7718
    trans.length = 24;  // Read 3 bytes (24-bit ADC data)
    trans.tx_buffer = NULL; // No sending data in this transaction
    trans.rx_buffer = rx_data; // Store recieved data in rx_data buffer
    trans.flags = 0;

    gpio_set_level(AD7718_CS, 0);
    ret = spi_device_transmit(ad7718_spi, &trans); // Read the ADC conversion result
    gpio_set_level(AD7718_CS, 1);
    assert(ret == ESP_OK); // Stop excecution if transaction fails

    // Convert the bytes into a single 24-bit value
    uint32_t adc_value = ((uint32_t)rx_data[0] << 16) | ((uint32_t)rx_data[1] << 8) | rx_data[2];

    // // **LOG OUTPUT**
    // ESP_LOGI("AD7718", "Raw ADC Data: 0x%08", adc_value);
    ESP_LOGI("AD7718", "ADC Decimal Value: %lu", adc_value);

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
        .mode = 3,                 // SPI mode 1 per AD7718 datasheet               /////// Change this to 3
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

// // Converts the raw ADC code into a sensor value using the provided formula.
// float ad7718_convert(int32_t adc_code)
// {
//     float adc_voltage = 2 * ((((float)adc_code / pow(2, 23)) - 1) * 2.5 + 0.0278177268);
//     ESP_LOGI(TAG, "ADC Value: %lu", adc_voltage);

//     return adc_voltage;
// }

void ad7718_task(void *pvParameters)
{
    // Configure GPIO
    gpio_set_direction(AD7718_RESET, GPIO_MODE_OUTPUT);
    gpio_set_direction(AD7718_RDY, GPIO_MODE_INPUT);

    // Initialize SPI and Reset AD7718
    ad7718_init_spi();
    ad7718_reset();

    // AD7718 Configuration
    ESP_LOGI("AD7718", "Setting ADC Mode to Continuous");
    ad7718_write(AD7718_SETUP_REG, 0x33); // Set ADC Mode to continuous


    ESP_LOGI("AD7718", "Setting ADC Filter Register");
    ad7718_write(0x03, 0x17); // Set ADC filter

    ESP_LOGI("AD7718", "Configuring input channels...");
    for (int channel = 0; channel < 5; channel++) 
    {
        uint8_t reg_value = 0x07 | (channel << 4);
        ESP_LOGI("AD7718", "Writing 0x%02X to Control Register (Channel %d)", reg_value, channel);
        ad7718_write(0x02, reg_value);
    }

    // Main loop to read ADC values
    while(1) {
        if (gpio_get_level(AD7718_RDY) == 0) { // Check if data is ready

            ESP_LOGI("AD7718", "Data Ready - Reading ADC Value...");

            // Read raw ADC value
            uint8_t raw_data[3] = {0};
            uint32_t adc_value = ad7718_read(AD7718_DATA_REG);

            ESP_LOGI("AD7718", "Raw ADC Data: 0x%08" PRIX32, adc_value);
            ESP_LOGI("AD7718", "ADC Decimal Value: %" PRIu32, adc_value);

            // Convert to voltage (assuming 2.5V reference and 24-bit ADC)
            float voltage = ((((float)adc_value / pow(2, 23)) - 1) * 2.5 + 0.0278177268);
            ESP_LOGI("AD7718", "Converted Voltage: %.6f V", voltage);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // read every second
    }
}
// ******************************* AD7718 *******************************


// Main application entry point
void app_main() {

    esp_err_t ret;

    // Set logging level for debugging
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    // Initialize Non-Volatile Storage for esp32
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); // Erase NVS if theres an error with free pages or version mismatch
        ret = nvs_flash_init(); // Re-initialize NVS
    }
    ESP_ERROR_CHECK(ret);

    // Create queues for MQTT & sensor data
    mqtt_aws_queue = xQueueCreate(10, sizeof(uint32_t));
    sensor_queue = xQueueCreate(5, sizeof(float[2])); // Only stores temp & humidity

    // Initialize Wi-Fi station mode
    wifi_init();
    if (wifi_connect() == ESP_OK)
    {
        ESP_LOGI(TAG, "Wi-Fi connection successful!");
    } else {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi after 10 tries.");
    }

    // Initialize AWS IoT MQTT Connection
    aws_init();
    int tlsErr = xTlsConnect(&network_context); // TLS handshake
    if (tlsErr != 0)
    {
        ESP_LOGE(TAG, "TLS connection failed with error: %d", tlsErr);
    } else {
        ESP_LOGI(TAG, "TLS connection successful!");
    }
    int mqttErr = MQTT_Connect(&mqtt_context, &connect_info, NULL, 1000, &ses_present); // Connect to MQTT Broker
    if (mqttErr != MQTTSuccess)
    {
        ESP_LOGE(TAG, "MQTT connection failed with error: %d", mqttErr);
    } else {
        ESP_LOGI(TAG, "MQTT connection successful!");
    }

    // esp_log_level_set("i2c", ESP_LOG_DEBUG);
    // esp_log_level_set("I2C", ESP_LOG_DEBUG);  // Some components use uppercase

    // ESP_LOGI("I2C", "Initializing I2C driver...");
    // ESP_ERROR_CHECK(i2cdev_init());

    // // Start sensor task (reads data and sends to MQTT)
    // xTaskCreatePinnedToCore(hdc1080_task, "hdc1080_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

    // // // Start sensor task (reads data and sends to MQTT)
    // xTaskCreatePinnedToCore(scd41_task, "scd41_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

    // scan_i2c_bus(); 
    // Start the AD7718 SPI sensor task.
    xTaskCreate(ad7718_task, "ad7718_task", 4096, NULL, 10, NULL);

    // Start MQTT processing task (Handles connection & publishing)
    xTaskCreate(mqtt_process_task, "mqtt_process_task", 4096, NULL, 9, &mqtt_task_handle);

    ESP_LOGI(TAG, "Application successfully started.");
}