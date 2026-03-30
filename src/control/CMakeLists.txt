# Control Algorithms Module
# Provides PID, state-space, MPC, and other advanced control algorithms

target_sources(mechatron_core
    PRIVATE
        ControlAlgorithms.cpp
)

target_include_directories(mechatron_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

message(STATUS "Control algorithms module enabled")
