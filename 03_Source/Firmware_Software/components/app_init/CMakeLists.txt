idf_component_register(SRCS "src/app.c"
                    INCLUDE_DIRS "include"
                    REQUIRES    
                    PRIV_REQUIRES wifi_app over_the_air arm globals udp_connect type_data robot_delta)