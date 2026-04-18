idf_component_register(SRCS "src/trajectory.c"
                    INCLUDE_DIRS "include"
                    REQUIRES type_data robot_delta  
                    PRIV_REQUIRES)