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
#include <unordered_map>

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
    Connection* connect(Port* source, Port* target, const std::string& uid = "");
    void disconnect(Connection* conn);
    const std::vector<std::unique_ptr<Connection>>& get_connections() const { return m_connections; }

    // Net-based voltage propagation
    void propagate_nets();

    // Selection
    void set_selected_component(const std::string& id) { m_selected_component = id; }
    const std::string& get_selected_component() const { return m_selected_component; }

    // Remove component (also cleans up physics body)
    void remove_component(std::string_view id);

    // Circuit simulation
    void step_circuit(double dt);
    bool has_circuit() const { return m_circuit_simulator != nullptr; }
    void mark_circuit_topology_dirty() { m_circuit_topology_dirty = true; }

    // Actuator voltage propagation
    void propagate_voltages_to_actuators();

    // Add equivalent circuits for actuators
    void add_actuator_equivalent_circuits();
    void add_mcu_equivalent_circuits();
    void update_mcu_equivalent_sources();
    void sync_mcu_power_from_ngspice();
    void sync_mcu_inputs_from_ngspice();

    bool get_actuator_terminal_current(std::string_view component_id,
                                       std::string_view pin_name,
                                       float& current) const;

private:
    TimeManager m_time;
    EventBus m_events;
    PluginHost m_plugins;
    Registry m_registry;
    std::unordered_map<std::string, std::unique_ptr<Subsystem>> m_subsystems;
    std::vector<std::unique_ptr<Connection>> m_connections;
    int m_next_connection_uid = 0;  // Connection UID generator

    CircuitPhysicsBridge m_circuit_bridge;

    // Circuit simulator instance
    class CircuitSimulator* m_circuit_simulator = nullptr;
    bool m_circuit_topology_dirty = true;

    std::string m_selected_component;

    // Physics world (lazy initialization)
    // Forward declared - use unique_ptr with deleter in cpp
    struct PhysicsWorldDeleter {
        void operator()(class PhysicsWorld* p);
    };
    std::unique_ptr<PhysicsWorld, PhysicsWorldDeleter> m_physics;

    // Storage for equivalent circuit components (e.g., motor resistors)
    // These are owned by the orchestrator and registered with the circuit simulator
    // Forward declared - use unique_ptr with deleter in cpp
    struct CircuitComponentDeleter {
        void operator()(class CircuitComponent* p);
    };
    std::vector<std::unique_ptr<class CircuitComponent, CircuitComponentDeleter>> m_equivalent_components;
    std::unordered_map<std::string, std::string> m_actuator_equivalent_map;
    std::unordered_map<std::string, std::string> m_mcu_equivalent_source_map;
};

} // namespace mechatron
