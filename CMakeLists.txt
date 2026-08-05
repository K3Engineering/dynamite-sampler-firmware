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

# Generate a csv holding the Factory namespace (board model and model-derived
# nominal values) and flash it to the DynaPersistent partition as an NVS image.
# WARNING: flashing rewrites the whole partition, erasing the User namespace too.
set(factory_nvs_csv ${CMAKE_CURRENT_BINARY_DIR}/factory_nvs.csv)
execute_process(
    COMMAND ${PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/generate_factory_nvs.py
            --sdkconfig ${SDKCONFIG} --out ${factory_nvs_csv}
    RESULT_VARIABLE factory_nvs_csv_result
)
if(NOT factory_nvs_csv_result EQUAL 0)
    message(FATAL_ERROR "Failed to generate the factory NVS csv")
endif()

# When ON (default), `idf.py flash` also writes the factory NVS image.
# When OFF, the image is written only on demand via `idf.py DynaPersistent-flash`.
option(DYNAMITE_FLASH_FACTORY_NVS "Flash factory NVS data on every idf.py flash" ON)
if(DYNAMITE_FLASH_FACTORY_NVS)
    set(factory_nvs_flash_arg FLASH_IN_PROJECT)
endif()
nvs_create_partition_image("DynaPersistent" "${factory_nvs_csv}" ${factory_nvs_flash_arg})

# TODO this doesn't work
# Set the env variable for github workflows
# if(DEFINED ENV{GITHUB_ENV})
#   file(APPEND "$ENV{GITHUB_ENV}" "PROJECT_NAME=dynamite-sampler-firmware\n")
# endif()
