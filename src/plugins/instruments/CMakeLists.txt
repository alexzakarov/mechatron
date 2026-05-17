# Instrument Plugin (Oscilloscope, etc.)

add_library(instrument_plugin_lib
    InstrumentPlugin.cpp
    InstrumentPlugin.hpp
)

target_link_libraries(instrument_plugin_lib
    PUBLIC
    mechatron_core
)

target_include_directories(instrument_plugin_lib PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_library(mechatron::instrument ALIAS instrument_plugin_lib)
