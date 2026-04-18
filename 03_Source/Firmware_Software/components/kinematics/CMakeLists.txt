idf_component_register(SRCS "src/kinematics.c"
                    INCLUDE_DIRS "include"
                    REQUIRES globals robot_delta type_data
                    PRIV_REQUIRES math_until)