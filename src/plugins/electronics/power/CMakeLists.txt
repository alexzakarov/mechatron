# Power Electronics Plugin (H-Bridge, Converters, Motor Drivers)

add_library(elec_power_lib
    ElecPowerPlugin.cpp
    ElecPowerPlugin.hpp
)

target_link_libraries(elec_power_lib
    PUBLIC
    mechatron_core
    mechatron_electronics
)

target_include_directories(elec_power_lib PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_library(mechatron::elec_power ALIAS elec_power_lib)
