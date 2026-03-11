idf_component_register(SRCS "magnetic_slide_switch.c"
                    INCLUDE_DIRS "include"
                    REQUIRES espressif__i2c_bus
                    PRIV_REQUIRES esp_adc control_serial nvs_flash)
