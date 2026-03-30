add_library(mechatron_sensors STATIC
    Sensor.cpp
    Sensor.hpp
)

target_include_directories(mechatron_sensors
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(mechatron_sensors
    PUBLIC
        mechatron_core
        nlohmann_json::nlohmann_json
        spdlog::spdlog
)
