idf_component_register(
    SRCS
        "ADS131M0x.cpp"
        "adc_proc.cpp"
        "ble_ota.cpp"
        "ble_proc.cpp"
        "board_id.cpp"
        "i2c_proc.cpp"
        "main.cpp"
        "runtime_stats.cpp"
        "user_kvs.cpp"
    INCLUDE_DIRS
        "."
        "${CMAKE_CURRENT_BINARY_DIR}" # Allows #include "build_metadata.h" from the build folder
    REQUIRES
        app_update
        bt
        esp_driver_gpio
        esp_driver_gptimer
        esp_driver_i2c
        esp_driver_spi
        nvs_flash
)

# Autogenerate build_metadata.h on every build.
# (add_custom_target is always considered out-of-date, so it runs every time you build).
set(build_metadata_filepath "${CMAKE_CURRENT_BINARY_DIR}/build_metadata.h")

add_custom_target(
    build_metadata_target
    COMMAND ${python} "${CMAKE_CURRENT_SOURCE_DIR}/generate_build_metadata.py"
        "--sdkconfig-debug=${CONFIG_COMPILER_OPTIMIZATION_DEBUG}"
        "--idf-version=${IDF_VER}"
        > "${build_metadata_filepath}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Generating build_metadata.h..."
    BYPRODUCTS "${build_metadata_filepath}"
)

# Force build_metadata_target to run BEFORE compiling the C++ files above.
add_dependencies(${COMPONENT_TARGET} build_metadata_target)
