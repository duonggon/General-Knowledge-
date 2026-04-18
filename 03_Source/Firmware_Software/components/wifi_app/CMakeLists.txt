idf_component_register(SRCS "src/wifi_app.c"
                    INCLUDE_DIRS "include"
                    REQUIRES    
                    PRIV_REQUIRES esp_wifi esp_event nvs_flash esp_netif)