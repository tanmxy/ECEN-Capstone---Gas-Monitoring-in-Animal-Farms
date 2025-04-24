idf_component_register(SRCS "Cap_GasMonitor.c" "wifi.c" "hdc1000.c" "scd4x.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_wifi coreMQTT coreMQTT-Agent driver backoffAlgorithm json esp_http_client freertos esp_timer)

