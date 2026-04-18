idf_component_register(SRCS "src/robot_delta.c"
                            "src/task_kinematics.c"
                            "src/task_motor_control.c"
                            "src/task_planner.c"
                    INCLUDE_DIRS "include"
                    REQUIRES type_data arm
                    PRIV_REQUIRES kinematics math_until trajectory globals log)