# Semiconductor Electronics Plugin (Diode, BJT, MOSFET, IGBT, etc.)

add_library(elec_semiconductor_lib
    ElecSemiconductorPlugin.cpp
    ElecSemiconductorPlugin.hpp
)

target_link_libraries(elec_semiconductor_lib
    PUBLIC
    mechatron_core
    mechatron_electronics
)

target_include_directories(elec_semiconductor_lib PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_library(mechatron::elec_semiconductor ALIAS elec_semiconductor_lib)
