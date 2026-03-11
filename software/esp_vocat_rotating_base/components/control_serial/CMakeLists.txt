idf_component_register(SRCS "control_serial.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES driver stepper_motor magnetic_slide_switch)
