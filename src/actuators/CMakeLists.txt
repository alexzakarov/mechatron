add_library(mechatron_actuators STATIC
    Actuator.cpp
    Actuator.hpp
)

target_include_directories(mechatron_actuators
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(mechatron_actuators
    PUBLIC
        mechatron_core
        mechatron_physics
        nlohmann_json::nlohmann_json
        spdlog::spdlog
)
