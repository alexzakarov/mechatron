#pragma once

#include "TimeManager.hpp"
#include "EventBus.hpp"
#include "PluginHost.hpp"
#include "Registry.hpp"
#include "Subsystem.hpp"
#include "CircuitPhysicsBridge.hpp"
#include <vector>
#include <memory>
#include <functional>

namespace mechatron {

class PhysicsWorld;

class SimulationOrchestrator {
public:
    SimulationOrchestrator();

    TimeManager& time_manager() { return m_time; }
    EventBus& event_bus() { return m_events; }
    PluginHost& plugin_host() { return m_plugins; }
    Registry& registry() { return m_registry; }
    PhysicsWorld& physics_world();

    // Circuit-Physics bridge
    CircuitPhysicsBridge& circuit_bridge() { return m_circuit_bridge; }

    // Subsystem management
    Subsystem* add_subsystem(std::string id);
    Subsystem* get_subsystem(std::string_view id);
    void remove_subsystem(std::string_view id);

    // Simulation lifecycle
    void start();
    void pause();
    void resume();
    void stop();
    void step();

    // Main update (called every frame)
    void update();

    // Component creation via plugin system
    Component* create_component(std::string_view plugin_name, std::string_view type, std::string id);

    // Plugin loading
    void load_all_plugins();
    void register_plugin(std::unique_ptr<IMechatronPlugin> plugin);

    // Connection management
    std::unique_ptr<Connection> connect(Port* source, Port* target);
    void disconnect(Connection* conn);

    // Selection
    void set_selected_component(const std::string& id) { m_selected_component = id; }
    const std::string& get_selected_component() const { return m_selected_component; }

    // Remove component (also cleans up physics body)
    void remove_component(std::string_view id);

private:
    TimeManager m_time;
    EventBus m_events;
    PluginHost m_plugins;
    Registry m_registry;
    std::unordered_map<std::string, std::unique_ptr<Subsystem>> m_subsystems;
    std::vector<std::unique_ptr<Connection>> m_connections;

    CircuitPhysicsBridge m_circuit_bridge;

    std::string m_selected_component;

    // Physics world (lazy initialization)
    // Forward declared - use unique_ptr with deleter in cpp
    struct PhysicsWorldDeleter {
        void operator()(class PhysicsWorld* p);
    };
    std::unique_ptr<PhysicsWorld, PhysicsWorldDeleter> m_physics;
};

} // namespace mechatron
