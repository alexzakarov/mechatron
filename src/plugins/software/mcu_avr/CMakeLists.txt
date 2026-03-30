# AVR MCU Plugin (Arduino Uno/Mega, ATtiny - QEMU backend)

add_library(soft_mcu_avr_lib
    MCUMcuAvrPlugin.cpp
    MCUMcuAvrPlugin.hpp
)

target_link_libraries(soft_mcu_avr_lib
    PUBLIC
    mechatron_core
    mechatron_mcu
)

target_include_directories(soft_mcu_avr_lib PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_library(mechatron::soft_mcu_avr ALIAS soft_mcu_avr_lib)
