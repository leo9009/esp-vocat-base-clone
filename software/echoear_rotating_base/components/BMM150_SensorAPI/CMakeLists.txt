idf_component_register(
    SRCS "bmm150_aux_adapter.c" "bmm150.c" "bmm150_aux_example.c"
    INCLUDE_DIRS "."
    REQUIRES bmi270 espressif__i2c_bus
) 