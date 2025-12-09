idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "."
    REQUIRES BMM150_SensorAPI espressif__i2c_bus
    )

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
