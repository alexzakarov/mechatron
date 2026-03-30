add_library(mechatron_core STATIC
    TimeManager.cpp
    EventBus.cpp
    PluginHost.cpp
    SimulationOrchestrator.cpp
    Subsystem.cpp
    ProjectFile.cpp
    MechatronProject.cpp
    CircuitPhysicsBridge.cpp
)

target_include_directories(mechatron_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(mechatron_core PUBLIC
    nlohmann_json::nlohmann_json
    spdlog::spdlog
)

# Physics dependency
add_dependencies(mechatron_core mechatron_physics)
target_link_libraries(mechatron_core PUBLIC mechatron_physics)

# Electronics dependency (for CircuitSimulator)
add_dependencies(mechatron_core mechatron_electronics)
target_link_libraries(mechatron_core PUBLIC mechatron_electronics)

# Actuators dependency
add_dependencies(mechatron_core mechatron_actuators)
target_link_libraries(mechatron_core PUBLIC mechatron_actuators)

target_compile_features(mechatron_core PUBLIC cxx_std_20)
