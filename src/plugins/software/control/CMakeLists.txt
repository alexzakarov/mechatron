# Control Algorithms Plugin (PID, State Machine, Kalman Filter, etc.)

add_library(soft_control_lib
    SoftControlPlugin.cpp
    SoftControlPlugin.hpp
)

target_link_libraries(soft_control_lib
    PUBLIC
    mechatron_core
)

target_include_directories(soft_control_lib PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_library(mechatron::soft_control ALIAS soft_control_lib)
