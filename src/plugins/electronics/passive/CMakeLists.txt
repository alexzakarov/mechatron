# Passive Electronics Plugin (R, C, L, Transformer)

add_library(elec_passive_lib
    ElecPassivePlugin.cpp
    ElecPassivePlugin.hpp
)

target_link_libraries(elec_passive_lib
    PUBLIC
    mechatron_core
    mechatron_electronics
)

target_include_directories(elec_passive_lib PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_library(mechatron::elec_passive ALIAS elec_passive_lib)
