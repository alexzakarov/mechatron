add_library(mechatron_physics STATIC
    PhysicsWorld.cpp
    PhysicsWorld.hpp
)

target_include_directories(mechatron_physics
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(mechatron_physics
    PUBLIC
        mechatron_core
        spdlog::spdlog
)
