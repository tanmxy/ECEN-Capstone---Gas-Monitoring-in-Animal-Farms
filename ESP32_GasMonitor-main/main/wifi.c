#include "wifi.h"
#include <string.h>
#include <stdio.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

static const char *TAG = "wifi";

// Wi-Fi event group and retry count
static EventGroupHandle_t wifi_eg;
#define WIFI_CONNECTED_BIT          BIT0
#define WIFI_FAIL_BIT               BIT1
#define WIFI_MAXIMUM_RETRY          10
static int s_retry_num = 0;

#define WIFI_SSID_1 "TAMU_IoT" // TAMU_IoT Wi-Fi SSID [school]
#define WIFI_SSID_2 "FoundingFathers" // Home Wi-Fi SSID [home]
#define WIFI_SSID_3 "Siboney" // Home Wi-Fi SSID [home]

#define WIFI_PASS_1 "" // TAMU Wi-Fi PASS [school]
#define WIFI_PASS_2 "chrisavila" // Home Wi-Fi PASS [home]
#define WIFI_PASS_3 "7gqqjsmyew" // Home Wi-Fi PASS [home]

// Wi-Fi event handler function
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                return;

            case WIFI_EVENT_STA_DISCONNECTED:
                if (s_retry_num < WIFI_MAXIMUM_RETRY) {
                    esp_wifi_connect();
                    s_retry_num++;
                    ESP_LOGI(TAG, "Retrying connection... (attempt %d)", s_retry_num);
                } else {
                    xEventGroupSetBits(wifi_eg, WIFI_FAIL_BIT);
                }
                ESP_LOGE(TAG, "Failed to connect to AP");
                break;
            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* evt = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&evt->ip_info.ip));
            s_retry_num = 0;
            xEventGroupSetBits(wifi_eg, WIFI_CONNECTED_BIT);
        } else {
            ESP_LOGI(TAG, "Unhandled IP event_id=%ld", (long) event_id);
        }
    }
}

// Function to wait for Wi-Fi connection result
esp_err_t wifi_connect(void)
{
    EventBits_t bits = xEventGroupWaitBits(wifi_eg, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                             pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP");
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to AP");
        return ESP_FAIL;
    }
    return ESP_ERR_INVALID_STATE;
}

// Function to initialize Wi-Fi and configure SSID/password for two networks
void wifi_init(void)
{
    wifi_eg = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                            ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                            IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Configure first network (SSID 1)
    wifi_config_t wifi_config_1 = {
        .sta = {
            .ssid = WIFI_SSID_1,
            .password = WIFI_PASS_1,
            .threshold.authmode = WIFI_AUTH_OPEN,
            .listen_interval = 0,
        },
    };

    // Configure second network (SSID 2)
    wifi_config_t wifi_config_2 = {
        .sta = {
            .ssid = WIFI_SSID_3,
            .password = WIFI_PASS_3,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    // Try connecting to the first network (SSID 1)
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config_1));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // Disable Wi-Fi power save mode

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Attempting to connect to: %s", WIFI_SSID_1);

    if (wifi_connect() != ESP_OK) {
        ESP_LOGI(TAG, "Failed to connect to the first Wi-Fi, trying the second SSID...");
        // Disconnect and stop Wi-Fi before switching
        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_stop());
        // Clear event group bits and reset retry counter
        xEventGroupClearBits(wifi_eg, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
        s_retry_num = 0;

        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config_2));
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // Disable Wi-Fi power save mode
        
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(TAG, "Attempting to connect to Wi-Fi SSID: %s", WIFI_SSID_2);
        wifi_connect();
    }
}