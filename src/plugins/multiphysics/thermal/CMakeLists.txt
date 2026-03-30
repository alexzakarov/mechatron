# Thermal Physics Plugin (Heat Source, Heat Sink, Convection, Radiation)

add_library(multi_thermal_lib
    MultiThermalPlugin.cpp
    MultiThermalPlugin.hpp
)

target_link_libraries(multi_thermal_lib
    PUBLIC
    mechatron_core
)

target_include_directories(multi_thermal_lib PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_library(mechatron::multi_thermal ALIAS multi_thermal_lib)
