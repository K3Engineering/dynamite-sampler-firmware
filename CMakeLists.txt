# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# By default esp idf builds all managed components, even though they aren't used.
# Build only necessary components by specifying only main.
# Components added via 'REQUIRES' are added to this list.
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(dynamite-sampler-firmware)

set(CMAKE_CXX_STANDARD 20)

# Generate an NVS image holding the Factory namespace (currently just the board
# model) and flash it to the DynaPersistent partition on every `idf.py flash`.
# WARNING: this rewrites the whole partition, erasing the User namespace too.
partition_table_get_partition_info(DYNA_PERSIST_PARTITION_SIZE
    "--partition-name DynaPersistent" "size")
set(factory_nvs_bin ${CMAKE_CURRENT_BINARY_DIR}/factory_nvs.bin)

# A custom target is always out of date. This forces cmake to run the COMMAND on every build.
add_custom_target(
    factory_nvs_bin_target
    COMMAND ${PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/generate_factory_nvs.py
            --sdkconfig ${SDKCONFIG}
            --out ${factory_nvs_bin}
            --size ${DYNA_PERSIST_PARTITION_SIZE}
    BYPRODUCTS ${factory_nvs_bin}
    COMMENT "Generating factory NVS image (board model)"
    VERBATIM
)

esptool_py_flash_to_partition(flash "DynaPersistent" "${factory_nvs_bin}")
add_dependencies(flash factory_nvs_bin_target)

# TODO this doesn't work
# Set the env variable for github workflows
# if(DEFINED ENV{GITHUB_ENV})
#   file(APPEND "$ENV{GITHUB_ENV}" "PROJECT_NAME=dynamite-sampler-firmware\n")
# endif()
