# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Prevents arduino from releasing BLE memory
add_compile_definitions(ARDUINO_ARCH_ESP32=1)

# Set the env variable for github workflows
file(APPEND "$ENV{GITHUB_ENV}" "PROJECT_NAME=dynamite-sampler-firmware\n")
# if(DEFINED ENV{GITHUB_ENV})
#   message(FATAL_ERROR "Setting GITHUB_ENV for CI workflows")
#   file(APPEND "$ENV{GITHUB_ENV}" "PROJECT_NAME=dynamite-sampler-firmware\n")
# endif()


project(dynamite-sampler-firmware)
# message(FATAL_ERROR "after project set")
