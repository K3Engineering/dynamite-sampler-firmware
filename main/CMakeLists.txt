idf_component_register(
    SRCS "main.cpp" "adc_proc.cpp" "ble_proc.cpp" "ADS131M0x.cpp"
    INCLUDE_DIRS ""
)

target_compile_definitions(${COMPONENT_TARGET} PUBLIC -DGIT_REVISION="${GIT_REVISION}")
