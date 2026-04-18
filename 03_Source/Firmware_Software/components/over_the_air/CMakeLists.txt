idf_component_register(SRCS "src/over_the_air.c"
                    INCLUDE_DIRS "include"
                    REQUIRES    
                    PRIV_REQUIRES esp_http_server app_update driver)