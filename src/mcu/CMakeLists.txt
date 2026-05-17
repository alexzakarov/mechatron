add_library(mechatron_mcu STATIC
    QEMUInterface.cpp
    QEMUInterface.hpp
    FirmwareLoader.cpp
    FirmwareLoader.hpp
    ATmegaInterpreter.cpp
    ATmegaInterpreter.hpp
    PhysicalMCUProtocol.cpp
    PhysicalMCUProtocol.hpp
    PhysicalMCUBridge.cpp
    PhysicalMCUBridge.hpp
)

target_include_directories(mechatron_mcu
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(mechatron_mcu
    PUBLIC
        mechatron_core
        mechatron_io
        spdlog::spdlog
)

# Platform-specific linking
if(WIN32)
    target_link_libraries(mechatron_mcu PUBLIC ws2_32)
endif()
