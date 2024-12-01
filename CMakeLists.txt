# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# By default esp idf builds all managed components, even though they aren't used.
# Components added via 'REQUIRES' are added to this list.
set(COMPONENTS )

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Prevents arduino from releasing BLE memory
add_compile_definitions(ARDUINO_ARCH_ESP32=1)

project(dynamite-sampler-firmware)

# TODO this doesn't work
# Set the env variable for github workflows
# if(DEFINED ENV{GITHUB_ENV})
#   file(APPEND "$ENV{GITHUB_ENV}" "PROJECT_NAME=dynamite-sampler-firmware\n")
# endif()
