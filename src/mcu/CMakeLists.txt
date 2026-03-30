add_library(mechatron_mcu STATIC
    QEMUInterface.cpp
    QEMUInterface.hpp
    FirmwareLoader.cpp
    FirmwareLoader.hpp
    ATmegaInterpreter.cpp
    ATmegaInterpreter.hpp
)

target_include_directories(mechatron_mcu
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(mechatron_mcu
    PUBLIC
        mechatron_core
        spdlog::spdlog
)

# Platform-specific linking
if(WIN32)
    target_link_libraries(mechatron_mcu PUBLIC ws2_32)
endif()
