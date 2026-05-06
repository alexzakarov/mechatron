#include "SimulationOrchestrator.hpp"
#include "physics/PhysicsWorld.hpp"
#include "electronics/CircuitSimulator.hpp"
#include "electronics/CircuitComponentAdapter.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

// Plugin includes (conditionally compiled)
#ifdef MECH_MACHINE_ELEMENTS_ENABLED
#include "plugins/mechanics/machine_elements/MechMachineElementsPlugin.hpp"
#endif
#ifdef ELEC_PASSIVE_ENABLED
#include "plugins/electronics/passive/ElecPassivePlugin.hpp"
#endif
#ifdef ELEC_SEMICONDUCTOR_ENABLED
#include "plugins/electronics/semiconductor/ElecSemiconductorPlugin.hpp"
#endif
#ifdef ELEC_POWER_ENABLED
#include "plugins/electronics/power/ElecPowerPlugin.hpp"
#endif
#ifdef SOFT_MCU_AVR_ENABLED
#include "plugins/software/mcu_avr/MCUMcuAvrPlugin.hpp"
#endif
#ifdef SOFT_CONTROL_ENABLED
#include "plugins/software/control/SoftControlPlugin.hpp"
#endif
#ifdef MULTI_THERMAL_ENABLED
#include "plugins/multiphysics/thermal/MultiThermalPlugin.hpp"
#endif
#ifdef MULTI_MAGNETIC_ENABLED
#include "plugins/multiphysics/magnetic/MultiMagneticPlugin.hpp"
#endif

namespace mechatron {

SimulationOrchestrator::SimulationOrchestrator() {
    // Initialize circuit bridge with registry
    m_circuit_bridge.set_registry(&m_registry);
}

Subsystem* SimulationOrchestrator::add_subsystem(std::string id) {
    auto sub = std::make_unique<Subsystem>(id);
    Subsystem* ptr = sub.get();
    m_subsystems[id] = std::move(sub);
    spdlog::info("Subsystem '{}' created", id);
    return ptr;
}

Subsystem* SimulationOrchestrator::get_subsystem(std::string_view id) {
    auto it = m_subsystems.find(std::string(id));
    return it != m_subsystems.end() ? it->second.get() : nullptr;
}

void SimulationOrchestrator::remove_subsystem(std::string_view id) {
    m_subsystems.erase(std::string(id));
}

void SimulationOrchestrator::start() {
    // Initialize physics world if not already
    if (!m_physics) {
        m_physics.reset(new PhysicsWorld());
        spdlog::info("Physics world initialized");
    }

    // Create physics bodies for existing components that don't have one
    m_registry.for_each([this](Component& comp) {
        if (comp.physics_body()) return; // Already has physics body

        auto& t = comp.transform();
        CollisionShapeDef shape;
        shape.type = CollisionShape::Box;
        shape.box_extents = {t.scale.x * 0.5f, t.scale.y * 0.5f, t.scale.z * 0.5f};

        PhysicsBody* body = m_physics->create_body(comp.id(), shape, t.position);
        if (body) {
            comp.attach_physics_body(body);
            spdlog::debug("Created physics body for: {}", comp.id());
        }
    });

    spdlog::info("Simulation starting");
    m_time.start();
    m_events.publish(Event{
        EventType::SimStart, EventDomain::System,
        "", "", std::monostate{}, m_time.current_tick()
    });
}

void SimulationOrchestrator::pause() {
    m_time.pause();
    m_events.publish(Event{
        EventType::SimPause, EventDomain::System,
        "", "", std::monostate{}, m_time.current_tick()
    });
}

void SimulationOrchestrator::resume() {
    m_time.resume();
    m_events.publish(Event{
        EventType::SimResume, EventDomain::System,
        "", "", std::monostate{}, m_time.current_tick()
    });
}

void SimulationOrchestrator::stop() {
    m_time.stop();
    m_events.publish(Event{
        EventType::SimStop, EventDomain::System,
        "", "", std::monostate{}, m_time.current_tick()
    });
    spdlog::info("Simulation stopped");
}

void SimulationOrchestrator::step() {
    m_time.step();
}

void SimulationOrchestrator::update() {
    m_time.update();

    double dt = m_time.physics_step_size();

    // Step 0: Update all components (sources set output port voltages)
    m_registry.for_each([dt](Component& comp) {
        comp.update(dt);
    });

    // Step 1: Net-based voltage propagation
    // Identifies electrically connected nets via Union-Find, then propagates
    // voltages within each net. This replaces naive one-way propagation.
    propagate_nets();

    // Step 1.5: Circuit simulation (MNA solver)
    // Run MNA solver to calculate currents in circuit components
    step_circuit(dt);

    // Step 2: Circuit-physics bridge update (voltages → actuator inputs)
    m_circuit_bridge.update(dt);

    // Only run physics when not stopped
    auto state = m_time.state();
    if (state == SimulationState::Stopped) return;

    // Step 4: Component → Physics: apply forces/torques to physics bodies
    if (m_physics) {
        m_registry.for_each([this](Component& comp) {
            PhysicsBody* body = comp.physics_body();
            if (!body || body->is_static) return;

            // Sync component transform position to physics body
            body->position = comp.transform().position;
        });

        // Step 5: Physics step
        m_physics->step(dt);

        // Step 6: Physics → Component: write back positions
        m_registry.for_each([dt](Component& comp) {
            PhysicsBody* body = comp.physics_body();
            if (!body) return;

            // Sync physics body position back to component transform
            comp.transform().position = body->position;
            comp.on_physics_update(dt);
        });
    }
}

Component* SimulationOrchestrator::create_component(
    std::string_view plugin_name,
    std::string_view type,
    std::string id)
{
    auto comp = m_plugins.create_component(plugin_name, type);
    if (!comp) return nullptr;

    Component* ptr = m_registry.add(std::move(comp), std::move(id));

    // Auto-create physics body for components with non-zero position or actuators
    if (ptr && m_physics) {
        auto& t = ptr->transform();
        CollisionShapeDef shape;
        shape.type = CollisionShape::Box;
        shape.box_extents = {t.scale.x * 0.5f, t.scale.y * 0.5f, t.scale.z * 0.5f};

        PhysicsBody* body = m_physics->create_body(ptr->id(), shape, t.position);
        if (body) {
            ptr->attach_physics_body(body);
            spdlog::debug("Created physics body for component: {}", ptr->id());
        }
    }

    return ptr;
}

Connection* SimulationOrchestrator::connect(Port* source, Port* target, const std::string& uid) {
    if (!source || !target) {
        spdlog::warn("connect() called with null port");
        return nullptr;
    }

    std::string conn_uid = uid.empty() ? "conn_" + std::to_string(m_next_connection_uid++) : uid;
    auto conn = std::make_unique<Connection>(source, target, conn_uid);
    Connection* ptr = conn.get();
    m_connections.push_back(std::move(conn));
    spdlog::debug("Connected port '{}' to '{}' (UID: {})", source->name(), target->name(), conn_uid);
    return ptr;
}

void SimulationOrchestrator::disconnect(Connection* conn) {
    // Find and remove the connection
    for (auto it = m_connections.begin(); it != m_connections.end(); ++it) {
        if (it->get() == conn) {
            // Reset target port value to stop voltage flow
            if (conn->target) {
                conn->target->set_value(0.0f);
            }

            // Remove from Port's connection list so connections().empty() works
            if (conn->source) {
                auto& src_conns = const_cast<std::vector<Connection*>&>(conn->source->connections());
                auto src_it = std::find(src_conns.begin(), src_conns.end(), conn);
                if (src_it != src_conns.end()) {
                    src_conns.erase(src_it);
                }
            }
            if (conn->target) {
                auto& tgt_conns = const_cast<std::vector<Connection*>&>(conn->target->connections());
                auto tgt_it = std::find(tgt_conns.begin(), tgt_conns.end(), conn);
                if (tgt_it != tgt_conns.end()) {
                    tgt_conns.erase(tgt_it);
                }
            }

            // Remove from orchestrator
            m_connections.erase(it);
            spdlog::info("Disconnected: {}",
                         conn->source ? conn->source->name() : "(null)");
            return;
        }
    }
}

PhysicsWorld& SimulationOrchestrator::physics_world() {
    if (!m_physics) {
        m_physics.reset(new PhysicsWorld());
        spdlog::info("Physics world initialized");
    }
    return *m_physics;
}

void SimulationOrchestrator::PhysicsWorldDeleter::operator()(PhysicsWorld* p) {
    delete p;
}

void SimulationOrchestrator::load_all_plugins() {
    spdlog::info("Loading all plugins...");

    // Mechanics plugins
    #ifdef MECH_MACHINE_ELEMENTS_ENABLED
    register_plugin(std::make_unique<MechMachineElementsPlugin>());
    #endif

    // Electronics plugins
    #ifdef ELEC_PASSIVE_ENABLED
    register_plugin(std::make_unique<ElecPassivePlugin>());
    #endif
    #ifdef ELEC_SEMICONDUCTOR_ENABLED
    register_plugin(std::make_unique<ElecSemiconductorPlugin>());
    #endif
    #ifdef ELEC_POWER_ENABLED
    register_plugin(std::make_unique<ElecPowerPlugin>());
    #endif

    // Software plugins
    #ifdef SOFT_MCU_AVR_ENABLED
    register_plugin(std::make_unique<MCUMcuAvrPlugin>());
    #endif
    #ifdef SOFT_CONTROL_ENABLED
    register_plugin(std::make_unique<SoftControlPlugin>());
    #endif

    // Multiphysics plugins
    #ifdef MULTI_THERMAL_ENABLED
    register_plugin(std::make_unique<MultiThermalPlugin>());
    #endif
    #ifdef MULTI_MAGNETIC_ENABLED
    register_plugin(std::make_unique<MultiMagneticPlugin>());
    #endif

    spdlog::info("All plugins loaded successfully");
}

void SimulationOrchestrator::register_plugin(std::unique_ptr<IMechatronPlugin> plugin) {
    std::string plugin_name(plugin->name());
    if (m_plugins.register_plugin(std::move(plugin))) {
        spdlog::info("Plugin '{}' registered successfully", plugin_name);
    } else {
        spdlog::warn("Failed to register plugin '{}'", plugin_name);
    }
}

void SimulationOrchestrator::remove_component(std::string_view id) {
    // Clean up physics body if it exists
    if (m_physics) {
        m_physics->remove_body(id);
    }

    // Clear selection if this component was selected
    if (m_selected_component == id) {
        m_selected_component.clear();
    }

    // Remove from registry
    m_registry.remove(id);
}

// ============================================================================
// Net-Based Voltage Propagation
// ============================================================================

void SimulationOrchestrator::propagate_nets() {
    if (m_connections.empty()) return;

    // Union-Find: map each Port* to an integer index
    std::unordered_map<Port*, int> port_to_idx;
    int idx_counter = 0;
    for (const auto& conn : m_connections) {
        if (conn->source && port_to_idx.find(conn->source) == port_to_idx.end()) {
            port_to_idx[conn->source] = idx_counter++;
        }
        if (conn->target && port_to_idx.find(conn->target) == port_to_idx.end()) {
            port_to_idx[conn->target] = idx_counter++;
        }
    }

    // Union-Find data structures
    std::vector<int> parent(idx_counter);
    for (int i = 0; i < idx_counter; i++) parent[i] = i;

    auto find_root = [&parent](int x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];  // Path compression
            x = parent[x];
        }
        return x;
    };

    auto union_ports = [&parent, &find_root](int a, int b) {
        int ra = find_root(a), rb = find_root(b);
        if (ra != rb) parent[ra] = rb;
    };

    // Build nets: union all connected ports
    for (const auto& conn : m_connections) {
        auto it_s = port_to_idx.find(conn->source);
        auto it_t = port_to_idx.find(conn->target);
        if (it_s != port_to_idx.end() && it_t != port_to_idx.end()) {
            union_ports(it_s->second, it_t->second);
        }
    }

    // For each net, find the driven voltage (from Output/driver ports)
    // and propagate to all ports in the net
    std::unordered_map<int, float> net_voltage;    // root -> voltage value
    std::unordered_map<int, bool> net_has_voltage; // root -> has a driver

    // First pass: find driven voltages in each net
    for (const auto& [port, idx] : port_to_idx) {
        int root = find_root(idx);

        // Check if this port is driven (Output direction and has a valid value)
        if (port->direction() == PortDirection::Output) {
            if (const float* val = port->get_value<float>()) {
                net_voltage[root] = *val;
                net_has_voltage[root] = true;
            }
        }
    }

    // Second pass: propagate net voltage to all ports in the net
    for (const auto& [port, idx] : port_to_idx) {
        int root = find_root(idx);
        if (net_has_voltage.count(root) && net_has_voltage[root]) {
            port->set_value(net_voltage[root]);
        }
    }
}

void SimulationOrchestrator::step_circuit(double dt) {
    // Lazy initialization of circuit simulator
    if (!m_circuit_simulator) {
        m_circuit_simulator = new CircuitSimulator();
        spdlog::info("[CIRCUIT] Circuit simulator initialized");
    }

    // Check if we have any circuit components to simulate
    bool has_circuit = false;
    m_registry.for_each([&has_circuit](Component& comp) {
        std::string_view cat = comp.category();
        if (cat == "passive" || cat == "semiconductor" || cat == "power" ||
            cat == "electronic" || cat == "optoelectronic") {
            has_circuit = true;
        }
    });

    if (!has_circuit) {
        return;
    }

    // Sync circuit components from adapters to simulator
    // IMPORTANT: We use the EXISTING circuit component instances from adapters
    // instead of creating duplicates. This ensures that current calculations
    // are reflected in the UI without needing to copy values back.
    m_registry.for_each([this](Component& comp) {
        std::string_view cat = comp.category();
        std::string_view ctype = comp.component_type();

        // Only process circuit components
        bool is_circuit = (cat == "electronic" || cat == "passive" ||
                          cat == "semiconductor" || cat == "power" ||
                          cat == "optoelectronic");

        if (!is_circuit) return;

        // Check if already added to simulator
        if (m_circuit_simulator->get_component(comp.id())) {
            return;
        }

        // Get the circuit component from adapter and add it to simulator
        // This uses the SAME instance, not a copy
        #define ADD_CIRCOMP_FROM_ADAPTER(AdapterType) \
            if (auto* adapted = dynamic_cast<AdapterType*>(&comp)) { \
                auto* circuit_comp = adapted->circuit_component(); \
                if (circuit_comp) { \
                    m_circuit_simulator->add_component_external(comp.id(), circuit_comp); \
                    spdlog::info("[CIRCUIT] Added {} ({}) to simulator (external)", comp.id(), ctype); \
                } \
            }

        // Passive components
        if (cat == "passive") {
            ADD_CIRCOMP_FROM_ADAPTER(ResistorComponent);
            ADD_CIRCOMP_FROM_ADAPTER(CapacitorComponent);
            ADD_CIRCOMP_FROM_ADAPTER(InductorComponent);
        }
        // Semiconductor components
        else if (cat == "semiconductor") {
            ADD_CIRCOMP_FROM_ADAPTER(DiodeComponent);
            ADD_CIRCOMP_FROM_ADAPTER(ZenerDiodeComponent);
            ADD_CIRCOMP_FROM_ADAPTER(LEDComponent);
            ADD_CIRCOMP_FROM_ADAPTER(BJTComponent);
            ADD_CIRCOMP_FROM_ADAPTER(MOSFETComponent);
        }
        // Power components
        else if (cat == "power") {
            ADD_CIRCOMP_FROM_ADAPTER(DCVoltageComponent);
            ADD_CIRCOMP_FROM_ADAPTER(GroundComponent);
            ADD_CIRCOMP_FROM_ADAPTER(HBridgeComponent);
            ADD_CIRCOMP_FROM_ADAPTER(BuckConverterComponent);
            ADD_CIRCOMP_FROM_ADAPTER(BoostConverterComponent);
            ADD_CIRCOMP_FROM_ADAPTER(MotorDriverComponent);
        }

        #undef ADD_CIRCOMP_FROM_ADAPTER
    });

    // Sync wire connections from orchestrator to circuit simulator
    for (const auto& conn : m_connections) {
        Port* src_port = conn->source;
        Port* tgt_port = conn->target;

        if (!src_port || !tgt_port) continue;

        Component* src_comp = src_port->owner();
        Component* tgt_comp = tgt_port->owner();

        if (!src_comp || !tgt_comp) continue;

        std::string wire_id = conn->uid.empty() ?
            ("wire_" + std::string(src_comp->id()) + "_" + std::string(tgt_comp->id())) :
            conn->uid;

        m_circuit_simulator->connect(
            wire_id,
            std::string(src_comp->id()),
            std::string(src_port->name()),
            std::string(tgt_comp->id()),
            std::string(tgt_port->name())
        );
    }

    // Step the circuit simulator (MNA solver + Newton-Raphson)
    // This will update voltages and currents directly in the adapter components
    m_circuit_simulator->step(dt);
}

} // namespace mechatron
