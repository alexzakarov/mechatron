#include "SimulationOrchestrator.hpp"
#include "physics/PhysicsWorld.hpp"
#include "electronics/CircuitSimulator.hpp"
#include "electronics/CircuitComponentAdapter.hpp"
#include "actuators/Actuator.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace mechatron { class CircuitComponent; }

namespace {
float pin_voltage_or_zero(mechatron::CircuitComponent* component, std::string_view pin_id) {
    if (!component) return 0.0f;
    for (auto* pin : component->get_pins()) {
        if (pin && pin->id == pin_id) return pin->voltage;
    }
    return 0.0f;
}

}

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
#ifdef INSTRUMENT_ENABLED
#include "plugins/instruments/InstrumentPlugin.hpp"
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

    // Prime electrical nets before the first running tick so MCUs see rails
    // immediately on Start (otherwise they may observe VCC/GND=0 for one frame
    // and refuse to run due to power gating).
    m_registry.for_each([](Component& comp) {
        if (comp.category() == "mcu" || comp.category() == "actuator" || comp.category() == "instrument") {
            return;
        }
        comp.update(0.0);
    });
    propagate_nets();

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
    bool single_step = m_time.consume_step_request();
    m_time.update();

    double dt = m_time.physics_step_size();
    auto state = m_time.state();
    bool active_step = (state == SimulationState::Running) || single_step;

    // Step 0: Update non-actuator, non-instrument components first.
    // Actuators consume solved NGSpice terminal voltages, so they are
    // advanced after step_circuit(). Instruments (oscilloscope) also need
    // to read post-circuit voltages, so they are deferred too.
    m_registry.for_each([dt, active_step](Component& comp) {
        if (comp.category() == "actuator" || comp.category() == "instrument") {
            return;
        }
        if (comp.category() == "mcu" && !active_step) {
            return;
        }
        comp.update(dt);
    });

    // Step 1.5: Circuit simulation (ngspice)
    // Run circuit simulation to calculate currents in circuit components
    // Only run when simulation is actually running (not paused/stopped)
    if (active_step) {
        step_circuit(dt);
    }

    // Step 1.6: Net propagation. Always run so that voltages from
    // non-ngspice components (potentiometers, sensors, MCU output pins)
    // propagate to instrument input ports and other consumers.
    propagate_nets();

    // Step 2: Circuit-physics bridge update (voltages → actuator inputs)
    if (active_step) {
        m_circuit_bridge.update(dt);
    }

    if (active_step) {
        m_registry.for_each([dt](Component& comp) {
            if (comp.category() == "actuator") {
                comp.update(dt);
            }
        });
    }

    // Step 3: Update instrument components (oscilloscope, etc.) after circuit
    // simulation so they can read solved voltages from MCU and other ports.
    // Instruments read directly from connected peer ports (not via propagate_nets).
    m_registry.for_each([dt](Component& comp) {
        if (comp.category() == "instrument") {
            comp.update(dt);
        }
    });

    // Only advance physics while the timeline is actively running, or while a
    // single-step request is being consumed.
    if (!active_step) return;

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
    m_circuit_topology_dirty = true;

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
    m_circuit_topology_dirty = true;
    spdlog::debug("Connected port '{}' to '{}' (UID: {})", source->name(), target->name(), conn_uid);
    return ptr;
}

void SimulationOrchestrator::disconnect(Connection* conn) {
    // Find and remove the connection
    for (auto it = m_connections.begin(); it != m_connections.end(); ++it) {
        if (it->get() == conn) {
            std::string source_name = conn->source ? std::string(conn->source->name()) : "(null)";

            // Reset target port value to stop voltage flow
            if (conn->target) {
                conn->target->set_value(0.0f);
            }

            // Remove from Port's connection list so connections().empty() works
            if (m_circuit_simulator) {
                m_circuit_simulator->disconnect(conn->uid);
            }

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
            m_circuit_topology_dirty = true;
            spdlog::info("Disconnected: {}", source_name);
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

void SimulationOrchestrator::CircuitComponentDeleter::operator()(CircuitComponent* p) {
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

    // Instrument plugins
    #ifdef INSTRUMENT_ENABLED
    register_plugin(std::make_unique<InstrumentPlugin>());
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
    // Clean up wire connections to/from this component
    std::vector<Connection*> connections_to_remove;
    for (auto& conn : m_connections) {
        bool involves_component = false;
        if (conn->source && conn->source->owner() &&
            std::string(conn->source->owner()->id()) == id) {
            involves_component = true;
        }
        if (conn->target && conn->target->owner() &&
            std::string(conn->target->owner()->id()) == id) {
            involves_component = true;
        }
        if (involves_component) {
            connections_to_remove.push_back(conn.get());
        }
    }

    // Remove all connections involving this component
    for (auto* conn : connections_to_remove) {
        disconnect(conn);
    }

    // Clean up physics body if it exists
    if (m_physics) {
        m_physics->remove_body(id);
    }

    // Clear selection if this component was selected
    if (m_selected_component == id) {
        m_selected_component.clear();
    }

    // Remove from circuit simulator if present
    if (m_circuit_simulator) {
        m_circuit_simulator->remove_component(id);
    }
    m_circuit_topology_dirty = true;

    // Remove from registry
    m_registry.remove(id);

    spdlog::info("Component '{}' removed with {} connections", id, connections_to_remove.size());
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
    std::unordered_map<int, bool> net_forced_ground; // root -> has explicit ground driver

    // First pass: find driven voltages in each net
    for (const auto& [port, idx] : port_to_idx) {
        int root = find_root(idx);

        // Check if this port is driven (Output or Bidirectional direction
        // and has a valid value). Bidirectional ports (e.g. MCU analog pins)
        // can drive a net when they hold a voltage.
        if (port->direction() == PortDirection::Output ||
            port->direction() == PortDirection::Bidirectional) {
            if (const float* val = port->get_value<float>()) {
                bool is_explicit_ground = false;
                if (auto* owner = port->owner()) {
                    is_explicit_ground = (owner->component_type() == "ground" && port->name() == "GND");
                }

                // Explicit ground source wins over any other driver on the net.
                if (is_explicit_ground) {
                    net_voltage[root] = 0.0f;
                    net_has_voltage[root] = true;
                    net_forced_ground[root] = true;
                    continue;
                }

                if (net_forced_ground[root]) {
                    continue;
                }

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
    static thread_local bool rebuilding_after_mcu_source_change = false;

    // Lazy initialization of circuit simulator
    if (!m_circuit_simulator) {
        m_circuit_simulator = new CircuitSimulator();
        spdlog::debug("[CIRCUIT] Circuit simulator initialized");
    }

    // Check if we have any electrical participants to simulate. MCU pins and
    // actuators are translated into equivalent circuit components below, so
    // they must also wake up the ngspice path even when no passive/power
    // component exists in the scene.
    bool has_circuit = false;
    m_registry.for_each([&has_circuit](Component& comp) {
        std::string_view cat = comp.category();
        if (cat == "passive" || cat == "semiconductor" || cat == "power" ||
            cat == "electronic" || cat == "optoelectronic" ||
            cat == "actuator" || cat == "mcu") {
            has_circuit = true;
        }
    });

    if (!has_circuit) {
        return;
    }

    if (m_circuit_topology_dirty) {
        delete m_circuit_simulator;
        m_circuit_simulator = new CircuitSimulator();
        m_equivalent_components.clear();
        m_actuator_equivalent_map.clear();
        m_mcu_equivalent_source_map.clear();
        spdlog::debug("[CIRCUIT] Rebuilt circuit simulator for dirty topology");

        m_registry.for_each([this](Component& comp) {
            std::string_view cat = comp.category();
            std::string_view ctype = comp.component_type();

            bool is_circuit = (cat == "electronic" || cat == "passive" ||
                              cat == "semiconductor" || cat == "power" ||
                              cat == "optoelectronic");

            if (!is_circuit || m_circuit_simulator->get_component(comp.id())) {
                return;
            }

            if (auto* adapted = dynamic_cast<ICircuitComponentAdapter*>(&comp)) {
                auto* circuit_comp = adapted->circuit_component_base();
                if (circuit_comp) {
                    m_circuit_simulator->add_component_external(comp.id(), circuit_comp);
                    spdlog::debug("[CIRCUIT] Added {} ({}) to simulator (external)", comp.id(), ctype);
                }
            }
        });

        spdlog::debug("[CIRCUIT] Syncing {} wire connections", m_connections.size());
        for (const auto& conn : m_connections) {
            Port* src_port = conn->source;
            Port* tgt_port = conn->target;

            if (!src_port || !tgt_port) {
                spdlog::debug("[CIRCUIT] Skipping connection - null port");
                continue;
            }

            Component* src_comp = src_port->owner();
            Component* tgt_comp = tgt_port->owner();

            if (!src_comp || !tgt_comp) {
                spdlog::debug("[CIRCUIT] Skipping connection - null component owner");
                continue;
            }

            // Actuators are represented in ngspice by equivalent circuit
            // components below. Their UI wires are translated there, so trying
            // to connect the actuator Component itself here only creates
            // missing-component failures and order-dependent intermediate
            // state.
            if (src_comp->category() == "actuator" || tgt_comp->category() == "actuator" ||
                src_comp->category() == "mcu" || tgt_comp->category() == "mcu") {
                continue;
            }

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

        // Ensure actuator loads are represented in the electrical solve so
        // actuator terminal voltages depend on circuit currents/voltage drops.
        add_actuator_equivalent_circuits();
        add_mcu_equivalent_circuits();
        m_circuit_topology_dirty = false;
    }

    // Step the circuit simulator. Actuator equivalent loads are included in
    // the solve, and actuator terminal ports are synchronized from those
    // solved equivalent pins below.
    update_mcu_equivalent_sources();
    if (m_circuit_topology_dirty && !rebuilding_after_mcu_source_change) {
        rebuilding_after_mcu_source_change = true;
        step_circuit(dt);
        rebuilding_after_mcu_source_change = false;
        return;
    }
    m_circuit_simulator->step(dt);

    m_registry.for_each([](Component& comp) {
        if (auto* adapted = dynamic_cast<ICircuitComponentAdapter*>(&comp)) {
            adapted->sync_circuit_pins_to_ports();
        }
    });

    auto sync_actuator_terminals_from_ngspice = [this]() {
        m_registry.for_each([this](Component& comp) {
        if (comp.category() != "actuator") return;

        Port* v_plus_port = nullptr;
        Port* gnd_port = nullptr;
        for (Port* port : comp.get_ports()) {
            if (port->name() == "V+") v_plus_port = port;
            else if (port->name() == "GND") gnd_port = port;
        }
        if (!v_plus_port || !gnd_port) return;

        auto map_it = m_actuator_equivalent_map.find(comp.id());
        if (map_it == m_actuator_equivalent_map.end()) return;

        CircuitComponent* eq = m_circuit_simulator->get_component(map_it->second);
        if (!eq) return;

        v_plus_port->set_value(pin_voltage_or_zero(eq, "1"));
        gnd_port->set_value(pin_voltage_or_zero(eq, "2"));
    });
    };
    sync_actuator_terminals_from_ngspice();

    // During active simulation, NGSpice is the single source of truth for
    // electrical state. Avoid generic net propagation here because it can
    // overwrite solved actuator terminal voltages with stale/default values.
    // Ensure actuator terminals stay anchored to NGSpice solved nodes.
    sync_actuator_terminals_from_ngspice();
    sync_mcu_power_from_ngspice();
    sync_mcu_inputs_from_ngspice();
}

void SimulationOrchestrator::add_actuator_equivalent_circuits() {
    // This function adds equivalent resistors for actuators to the circuit simulator
    // DC motors are modeled as resistors (their internal resistance)
    // This allows them to participate in circuit simulation

    m_registry.for_each([this](Component& comp) {
        // Only process DC motors
        if (comp.component_type() != "dc_motor") return;

        // Try to cast to DCMotor
        auto* motor = dynamic_cast<DCMotor*>(&comp);
        if (!motor) return;

        // Check if we already added an equivalent resistor for this motor
        std::string equiv_resistor_id = "rmotor_" + std::string(comp.id());
        if (m_circuit_simulator->get_component(equiv_resistor_id)) {
            m_actuator_equivalent_map[comp.id()] = equiv_resistor_id;
            spdlog::debug("[CIRCUIT] Motor {} already has equivalent resistor", comp.id());
            return;
        }

        // Use a rated-load equivalent winding resistance. The UI exposes the
        // default DC motor as a 12 ohm load; deriving this from the voltage
        // rating keeps the electrical model consistent with that presentation
        // and avoids the old hardcoded 1 ohm stall-like load.
        float voltage_rating = motor->get_voltage_rating();
        float motor_resistance = std::max(0.1f, voltage_rating);

        spdlog::debug("[CIRCUIT] Adding equivalent resistor {} for motor {} (R={} ohm)",
                    equiv_resistor_id, comp.id(), motor_resistance);

        // Create a Resistor circuit component (using constructor to set resistance)
        auto resistor = std::make_unique<Resistor>(motor_resistance);
        // Note: m_id will be set by add_component_external

        // Get motor's port connections
        auto ports = comp.get_ports();
        Port* v_plus_port = nullptr;
        Port* gnd_port = nullptr;

        for (Port* port : ports) {
            std::string_view port_name = port->name();
            if (port_name == "V+") {
                v_plus_port = port;
            } else if (port_name == "GND") {
                gnd_port = port;
            }
        }

        if (!v_plus_port || !gnd_port) {
            spdlog::warn("[CIRCUIT] Motor {} missing V+ or GND port, cannot add equivalent resistor", comp.id());
            return;
        }

        // Add resistor to circuit simulator
        m_circuit_simulator->add_component_external(equiv_resistor_id, resistor.get());

        // Track which resistor pin connects to which motor port
        // Resistor pin 1 (m_pin1) will connect to motor's V+
        // Resistor pin 2 (m_pin2) will connect to motor's GND
        struct ResistorConnection {
            std::string circuit_component_id;
            std::string circuit_pin;
            std::string resistor_pin;  // "1" for V+ side, "2" for GND side
        };

        std::vector<ResistorConnection> resistor_connections;

        auto collect_net_ports = [this](Port* start) {
            std::vector<Port*> result;
            if (!start) return result;

            std::vector<Port*> stack{start};
            std::unordered_set<Port*> visited;
            visited.insert(start);

            while (!stack.empty()) {
                Port* current = stack.back();
                stack.pop_back();
                result.push_back(current);

                for (const auto& conn : m_connections) {
                    if (!conn->source || !conn->target) continue;

                    Port* next = nullptr;
                    if (conn->source == current) {
                        next = conn->target;
                    } else if (conn->target == current) {
                        next = conn->source;
                    }

                    if (next && visited.insert(next).second) {
                        stack.push_back(next);
                    }
                }
            }

            return result;
        };

        std::unordered_set<std::string> added_connections;
        auto add_motor_port_connections = [&](Port* motor_port) {
            if (!motor_port) return;

            std::string_view motor_port_name = motor_port->name();
            std::string resistor_pin;

            if (motor_port_name == "V+") {
                resistor_pin = "1";  // Resistor pin 1 connects to V+ net
            } else if (motor_port_name == "GND") {
                resistor_pin = "2";  // Resistor pin 2 connects to GND net
            } else {
                spdlog::warn("[CIRCUIT] Motor {} has unknown port '{}', skipping", comp.id(), motor_port_name);
                return;
            }

            for (Port* net_port : collect_net_ports(motor_port)) {
                if (!net_port || net_port == motor_port) continue;

                Component* circuit_comp = net_port->owner();
                if (!circuit_comp || circuit_comp == &comp) continue;

                if (circuit_comp->category() != "power" &&
                    circuit_comp->category() != "passive" &&
                    circuit_comp->category() != "semiconductor" &&
                    circuit_comp->category() != "electronic" &&
                    circuit_comp->category() != "optoelectronic") {
                    continue;
                }

                std::string key = std::string(circuit_comp->id()) + "." +
                                  std::string(net_port->name()) + "." +
                                  resistor_pin;
                if (!added_connections.insert(key).second) continue;

                resistor_connections.push_back({
                    std::string(circuit_comp->id()),
                    std::string(net_port->name()),
                    resistor_pin
                });

                spdlog::debug("[CIRCUIT] Motor {} port {} connected through net to {}:{}, will wire resistor pin {} to same",
                             comp.id(), motor_port_name, circuit_comp->id(), net_port->name(), resistor_pin);
            }
        };

        add_motor_port_connections(v_plus_port);
        add_motor_port_connections(gnd_port);

        // Now create the actual wire connections in the circuit simulator
        int wire_index = 0;
        for (const auto& res_conn : resistor_connections) {
            std::string wire_id = "wire_" + equiv_resistor_id + "_" + res_conn.circuit_component_id + "_" + std::to_string(wire_index++);

            // Create wire: equiv_resistor.pin -> circuit_component.pin
            // The connect method signature is:
            // connect(wire_id, source_comp, source_pin, target_comp, target_pin)
            bool success = m_circuit_simulator->connect(
                wire_id,
                equiv_resistor_id,           // Source: equivalent resistor
                res_conn.resistor_pin,       // Source pin: "1" or "2"
                res_conn.circuit_component_id, // Target: the circuit component motor is connected to
                res_conn.circuit_pin         // Target pin: the specific pin on that component
            );

            if (success) {
                spdlog::debug("[CIRCUIT] Wired equivalent resistor: {}.{} -> {}.{}",
                           equiv_resistor_id, res_conn.resistor_pin,
                           res_conn.circuit_component_id, res_conn.circuit_pin);
            } else {
                spdlog::error("[CIRCUIT] Failed to wire equivalent resistor: {}.{} -> {}.{}",
                            equiv_resistor_id, res_conn.resistor_pin,
                            res_conn.circuit_component_id, res_conn.circuit_pin);
            }
        }

        // Keep the resistor alive (stored in orchestrator's equivalent components)
        // Convert unique_ptr<Resistor> to unique_ptr<CircuitComponent, CircuitComponentDeleter>
        m_equivalent_components.push_back(
            std::unique_ptr<CircuitComponent, CircuitComponentDeleter>(
                resistor.release(),
                CircuitComponentDeleter()
            )
        );
        m_actuator_equivalent_map[comp.id()] = equiv_resistor_id;

        if (resistor_connections.empty()) {
            spdlog::warn("[CIRCUIT] Motor {} has no circuit connections, equivalent resistor {} is unconnected",
                        comp.id(), equiv_resistor_id);
        } else {
            spdlog::debug("[CIRCUIT] Motor {} equivalent resistor {} wired with {} connections",
                        comp.id(), equiv_resistor_id, resistor_connections.size());
        }
    });
}

void SimulationOrchestrator::add_mcu_equivalent_circuits() {
    m_registry.for_each([this](Component& comp) {
        if (comp.category() != "mcu") return;

        auto collect_net_ports = [this](Port* start) {
            std::vector<Port*> result;
            if (!start) return result;

            std::vector<Port*> stack{start};
            std::unordered_set<Port*> visited;
            visited.insert(start);

            while (!stack.empty()) {
                Port* current = stack.back();
                stack.pop_back();
                result.push_back(current);

                for (const auto& conn : m_connections) {
                    if (!conn->source || !conn->target) continue;

                    Port* next = nullptr;
                    if (conn->source == current) {
                        next = conn->target;
                    } else if (conn->target == current) {
                        next = conn->source;
                    }

                    if (next && visited.insert(next).second) {
                        stack.push_back(next);
                    }
                }
            }

            return result;
        };

        auto resolve_circuit_endpoint = [this](Component* target_comp, Port* target_port,
                                               std::string& component_id,
                                               std::string& port_name) {
            if (!target_comp || !target_port) return false;

            const std::string_view category = target_comp->category();
            if (category == "power" ||
                category == "passive" ||
                category == "semiconductor" ||
                category == "electronic" ||
                category == "optoelectronic") {
                component_id = target_comp->id();
                port_name = std::string(target_port->name());
                return true;
            }

            if (category == "actuator") {
                auto map_it = m_actuator_equivalent_map.find(target_comp->id());
                if (map_it == m_actuator_equivalent_map.end()) return false;

                if (target_port->name() == "V+") {
                    component_id = map_it->second;
                    port_name = "1";
                    return true;
                }
                if (target_port->name() == "GND") {
                    component_id = map_it->second;
                    port_name = "2";
                    return true;
                }
            }

            return false;
        };

        auto ports = comp.get_ports();
        Port* mcu_gnd = nullptr;
        for (Port* port : ports) {
            if (port && port->name() == "GND") {
                mcu_gnd = port;
                break;
            }
        }
        if (!mcu_gnd) return;

        std::vector<std::pair<Component*, Port*>> gnd_targets;
        for (Port* net_port : collect_net_ports(mcu_gnd)) {
            if (!net_port || net_port == mcu_gnd || !net_port->owner()) continue;
            gnd_targets.push_back({net_port->owner(), net_port});
        }
        if (gnd_targets.empty()) {
            spdlog::debug("[CIRCUIT][MCU] {} has no GND reference in circuit, skipping output sources", comp.id());
            return;
        }

        for (Port* mcu_port : ports) {
            if (!mcu_port) continue;

            std::string pin_name(mcu_port->name());
            if (pin_name.size() < 2 || (pin_name[0] != 'D' && pin_name[0] != 'A')) {
                continue;
            }

            float voltage = 0.0f;
            bool is_output = comp.get_mcu_pin_output_voltage(pin_name, voltage);

            std::vector<std::pair<Component*, Port*>> pin_targets;
            for (Port* net_port : collect_net_ports(mcu_port)) {
                if (!net_port || net_port == mcu_port || !net_port->owner()) continue;
                pin_targets.push_back({net_port->owner(), net_port});
            }
            if (pin_targets.empty()) continue;

            if (is_output) {
                // Output pin: drive the net with a voltage source
                std::string source_id = "vmcu_" + std::string(comp.id()) + "_" + pin_name;
                auto source = std::make_unique<DCVoltageSource>(voltage);
                m_circuit_simulator->add_component_external(source_id, source.get());
                m_mcu_equivalent_source_map[std::string(comp.id()) + "." + pin_name] = source_id;

                int wire_index = 0;
                for (const auto& [target_comp, target_port] : pin_targets) {
                    if (!target_comp || !target_port || target_comp == &comp) continue;
                    std::string endpoint_component;
                    std::string endpoint_port;
                    if (!resolve_circuit_endpoint(target_comp, target_port, endpoint_component, endpoint_port)) {
                        continue;
                    }

                    m_circuit_simulator->connect(
                        "wire_" + source_id + "_out_" + std::to_string(wire_index++),
                        source_id,
                        "V+",
                        endpoint_component,
                        endpoint_port
                    );
                }

                wire_index = 0;
                for (const auto& [target_comp, target_port] : gnd_targets) {
                    if (!target_comp || !target_port || target_comp == &comp) continue;
                    std::string endpoint_component;
                    std::string endpoint_port;
                    if (!resolve_circuit_endpoint(target_comp, target_port, endpoint_component, endpoint_port)) {
                        continue;
                    }

                    m_circuit_simulator->connect(
                        "wire_" + source_id + "_gnd_" + std::to_string(wire_index++),
                        source_id,
                        "GND",
                        endpoint_component,
                        endpoint_port
                    );
                }

                m_equivalent_components.push_back(
                    std::unique_ptr<CircuitComponent, CircuitComponentDeleter>(
                        source.release(),
                        CircuitComponentDeleter()
                    )
                );
                spdlog::debug("[CIRCUIT][MCU] Added output source {} for {}.{} = {}V",
                             source_id, comp.id(), pin_name, voltage);
            } else if (pin_name[0] == 'A') {
                // Analog input pin (A0-A5): add a high-impedance resistor (1GΩ)
                // to GND so ngspice includes this node in the solve. This allows
                // the pin to read voltages from connected circuit components
                // (e.g. potentiometer wiper, voltage dividers).
                std::string probe_id = "rmcu_" + std::string(comp.id()) + "_" + pin_name;
                auto probe = std::make_unique<Resistor>(1e9f); // 1 GΩ
                m_circuit_simulator->add_component_external(probe_id, probe.get());

                // Connect probe pin 1 to circuit targets
                int wire_index = 0;
                for (const auto& [target_comp, target_port] : pin_targets) {
                    if (!target_comp || !target_port || target_comp == &comp) continue;
                    std::string endpoint_component;
                    std::string endpoint_port;
                    if (!resolve_circuit_endpoint(target_comp, target_port, endpoint_component, endpoint_port)) {
                        continue;
                    }

                    m_circuit_simulator->connect(
                        "wire_" + probe_id + "_sig_" + std::to_string(wire_index++),
                        probe_id,
                        "1",
                        endpoint_component,
                        endpoint_port
                    );
                }

                // Connect probe pin 2 to GND
                wire_index = 0;
                for (const auto& [target_comp, target_port] : gnd_targets) {
                    if (!target_comp || !target_port || target_comp == &comp) continue;
                    std::string endpoint_component;
                    std::string endpoint_port;
                    if (!resolve_circuit_endpoint(target_comp, target_port, endpoint_component, endpoint_port)) {
                        continue;
                    }

                    m_circuit_simulator->connect(
                        "wire_" + probe_id + "_gnd_" + std::to_string(wire_index++),
                        probe_id,
                        "2",
                        endpoint_component,
                        endpoint_port
                    );
                }

                m_equivalent_components.push_back(
                    std::unique_ptr<CircuitComponent, CircuitComponentDeleter>(
                        probe.release(),
                        CircuitComponentDeleter()
                    )
                );
                spdlog::debug("[CIRCUIT][MCU] Added input probe {} for {}.{} (1GΩ to GND)",
                             probe_id, comp.id(), pin_name);
            }
        }
    });
}

void SimulationOrchestrator::update_mcu_equivalent_sources() {
    if (!m_circuit_simulator) return;

    m_registry.for_each([this](Component& comp) {
        if (comp.category() != "mcu") return;

        for (Port* port : comp.get_ports()) {
            if (!port) continue;
            std::string pin_name(port->name());
            if (pin_name.size() < 2 || (pin_name[0] != 'D' && pin_name[0] != 'A')) {
                continue;
            }

            float voltage = 0.0f;
            bool is_output = comp.get_mcu_pin_output_voltage(pin_name, voltage);
            std::string key = std::string(comp.id()) + "." + pin_name;
            auto map_it = m_mcu_equivalent_source_map.find(key);

            if (is_output && map_it == m_mcu_equivalent_source_map.end()) {
                if (port->connections().empty()) {
                    continue;
                }
                m_circuit_topology_dirty = true;
                return;
            }
            if (!is_output && map_it != m_mcu_equivalent_source_map.end()) {
                m_circuit_topology_dirty = true;
                return;
            }
            if (!is_output) continue;

            CircuitComponent* source = m_circuit_simulator->get_component(map_it->second);
            if (source) {
                source->set_parameter("voltage", voltage);
            }
        }
    });
}

void SimulationOrchestrator::sync_mcu_power_from_ngspice() {
    if (!m_circuit_simulator) return;

    auto collect_net_ports = [this](Port* start) {
        std::vector<Port*> result;
        if (!start) return result;

        std::vector<Port*> stack{start};
        std::unordered_set<Port*> visited;
        visited.insert(start);

        while (!stack.empty()) {
            Port* current = stack.back();
            stack.pop_back();
            result.push_back(current);

            for (const auto& conn : m_connections) {
                if (!conn->source || !conn->target) continue;

                Port* next = nullptr;
                if (conn->source == current) {
                    next = conn->target;
                } else if (conn->target == current) {
                    next = conn->source;
                }

                if (next && visited.insert(next).second) {
                    stack.push_back(next);
                }
            }
        }

        return result;
    };

    auto solved_port_voltage = [this](Component* other_comp, Port* other_port, float& voltage) {
        if (!other_comp || !other_port) return false;

        if (CircuitComponent* circuit_comp = m_circuit_simulator->get_component(other_comp->id())) {
            voltage = pin_voltage_or_zero(circuit_comp, other_port->name());
            return true;
        }

        if (other_comp->category() == "actuator") {
            auto map_it = m_actuator_equivalent_map.find(other_comp->id());
            if (map_it == m_actuator_equivalent_map.end()) return false;

            CircuitComponent* eq = m_circuit_simulator->get_component(map_it->second);
            if (!eq) return false;

            if (other_port->name() == "V+") {
                voltage = pin_voltage_or_zero(eq, "1");
                return true;
            }
            if (other_port->name() == "GND") {
                voltage = pin_voltage_or_zero(eq, "2");
                return true;
            }
        }

        return false;
    };

    m_registry.for_each([this, &collect_net_ports, &solved_port_voltage](Component& comp) {
        if (comp.category() != "mcu") return;

        for (Port* mcu_port : comp.get_ports()) {
            if (!mcu_port) continue;

            const std::string pin_name(mcu_port->name());
            if (pin_name != "VCC" && pin_name != "GND" && pin_name != "AREF") {
                continue;
            }

            for (Port* net_port : collect_net_ports(mcu_port)) {
                if (!net_port || net_port == mcu_port) continue;

                float voltage = 0.0f;
                if (!solved_port_voltage(net_port->owner(), net_port, voltage)) continue;

                mcu_port->set_value(voltage);
                break;
            }
        }
    });
}

void SimulationOrchestrator::sync_mcu_inputs_from_ngspice() {
    if (!m_circuit_simulator) return;

    auto collect_net_ports = [this](Port* start) {
        std::vector<Port*> result;
        if (!start) return result;

        std::vector<Port*> stack{start};
        std::unordered_set<Port*> visited;
        visited.insert(start);

        while (!stack.empty()) {
            Port* current = stack.back();
            stack.pop_back();
            result.push_back(current);

            for (const auto& conn : m_connections) {
                if (!conn->source || !conn->target) continue;

                Port* next = nullptr;
                if (conn->source == current) {
                    next = conn->target;
                } else if (conn->target == current) {
                    next = conn->source;
                }

                if (next && visited.insert(next).second) {
                    stack.push_back(next);
                }
            }
        }

        return result;
    };

    auto solved_port_voltage = [this](Component* other_comp, Port* other_port, float& voltage) {
        if (!other_comp || !other_port) return false;

        if (CircuitComponent* circuit_comp = m_circuit_simulator->get_component(other_comp->id())) {
            voltage = pin_voltage_or_zero(circuit_comp, other_port->name());
            return true;
        }

        if (other_comp->category() == "actuator") {
            auto map_it = m_actuator_equivalent_map.find(other_comp->id());
            if (map_it == m_actuator_equivalent_map.end()) return false;

            CircuitComponent* eq = m_circuit_simulator->get_component(map_it->second);
            if (!eq) return false;

            if (other_port->name() == "V+") {
                voltage = pin_voltage_or_zero(eq, "1");
                return true;
            }
            if (other_port->name() == "GND") {
                voltage = pin_voltage_or_zero(eq, "2");
                return true;
            }
        }

        return false;
    };

    m_registry.for_each([this, &collect_net_ports, &solved_port_voltage](Component& comp) {
        if (comp.category() != "mcu") return;

        for (Port* mcu_port : comp.get_ports()) {
            if (!mcu_port) continue;
            std::string pin_name(mcu_port->name());
            if (pin_name.size() < 2 || (pin_name[0] != 'D' && pin_name[0] != 'A')) {
                continue;
            }

            float output_voltage = 0.0f;
            if (comp.get_mcu_pin_output_voltage(pin_name, output_voltage)) {
                continue;
            }

            float gnd_voltage = 0.0f;
            for (Port* candidate : comp.get_ports()) {
                if (candidate && candidate->name() == "GND") {
                    if (const float* value = candidate->get_value<float>()) {
                        gnd_voltage = *value;
                    }
                    break;
                }
            }

            bool updated = false;
            for (Port* net_port : collect_net_ports(mcu_port)) {
                if (!net_port || net_port == mcu_port) continue;

                float voltage = 0.0f;
                if (!solved_port_voltage(net_port->owner(), net_port, voltage)) continue;

                comp.set_mcu_pin_input_voltage(pin_name, voltage);
                if (pin_name[0] == 'A') {
                    mcu_port->set_value(voltage);
                } else {
                    mcu_port->set_value((voltage - gnd_voltage) >= 2.5f);
                }
                updated = true;
                break;
            }

            if (!updated) {
                comp.set_mcu_pin_input_voltage(pin_name, 0.0f);
            }
        }
    });
}

bool SimulationOrchestrator::get_actuator_terminal_current(std::string_view component_id,
                                                           std::string_view pin_name,
                                                           float& current) const {
    if (!m_circuit_simulator) return false;

    auto map_it = m_actuator_equivalent_map.find(std::string(component_id));
    if (map_it == m_actuator_equivalent_map.end()) return false;

    CircuitComponent* eq = m_circuit_simulator->get_component(map_it->second);
    if (!eq) return false;

    std::string_view equivalent_pin;
    if (pin_name == "V+") {
        equivalent_pin = "1";
    } else if (pin_name == "GND") {
        equivalent_pin = "2";
    } else {
        return false;
    }

    for (auto* pin : eq->get_pins()) {
        if (pin && pin->id == equivalent_pin) {
            current = pin->current;
            return true;
        }
    }

    return false;
}

void SimulationOrchestrator::propagate_voltages_to_actuators() {
    // This function propagates voltages from circuit components (like voltage sources)
    // to actuators (like DC motors) that are connected via wires
    // Actuators are not in the circuit simulator but still need voltage to operate

    spdlog::debug("[ACTUATOR] ===== PROPAGATE VOLTAGES TO ACTUATORS START =====");
    spdlog::debug("[ACTUATOR] propagate_voltages_to_actuators() called - checking {} components", m_registry.size());
    spdlog::debug("[ACTUATOR] Total wire connections: {}", m_connections.size());
    int actuator_count = 0;
    int connection_count = 0;

    m_registry.for_each([this, &actuator_count, &connection_count](Component& comp) {
        // Only process actuators
        if (comp.category() != "actuator") return;

        // Try to cast to Actuator base class
        auto* actuator = dynamic_cast<Actuator*>(&comp);
        if (!actuator) return;

        actuator_count++;
        spdlog::debug("[ACTUATOR] Found actuator: {}, type: {}", comp.id(), comp.component_type());

        // Get actuator ports (V+, GND)
        auto ports = comp.get_ports();
        Port* v_plus_port = nullptr;
        Port* gnd_port = nullptr;

        for (Port* port : ports) {
            std::string_view port_name = port->name();
            if (port_name == "V+") {
                v_plus_port = port;
            } else if (port_name == "GND") {
                gnd_port = port;
            }
        }

        if (!v_plus_port || !gnd_port) {
            return;  // Actuator doesn't have V+/GND ports
        }

        // Find what these ports are connected to via wires
        float v_plus_voltage = 0.0f;
        float gnd_voltage = 0.0f;

        spdlog::debug("[ACTUATOR] Motor {} has {} wire connections to check", comp.id(), m_connections.size());

        // Search for wire connections in BOTH directions
        int checked_connections = 0;
        for (const auto& conn : m_connections) {
            spdlog::debug("[ACTUATOR] Checking connection {}/{}", checked_connections + 1, m_connections.size());
            checked_connections++;
            if (!conn->source || !conn->target) {
                spdlog::debug("[ACTUATOR] Connection {}/{} has null source or target", checked_connections, m_connections.size());
                continue;
            }

            spdlog::debug("[ACTUATOR] Connection {}/{} - source: {} (owner: {}), target: {} (owner: {})",
                        checked_connections, m_connections.size(),
                        conn->source->name(), conn->source->owner() ? conn->source->owner()->id() : "null",
                        conn->target->name(), conn->target->owner() ? conn->target->owner()->id() : "null");

            // CRITICAL FIX: Compare by ID, not by pointer!
            // The registry component might be a different instance than the one in connections
            // Also check if the connection ports belong to the motor (by checking port owner ID)
            std::string motor_id = comp.id();

            // Direction 1: conn->source -> conn->target
            bool motor_is_target = (conn->target->owner() && conn->target->owner()->id() == motor_id);
            bool motor_is_source = (conn->source->owner() && conn->source->owner()->id() == motor_id);

            // Also check if the ports belong to the motor (by comparing port ownership through component)
            // Sometimes the port's owner might be null, but the port itself belongs to the motor
            // We need to check if the port is one of the motor's ports
            bool target_port_is_motors = false;
            bool source_port_is_motors = false;

            // Check if target port is one of motor's ports
            for (Port* motor_port : comp.get_ports()) {
                if (conn->target == motor_port) {
                    target_port_is_motors = true;
                    break;
                }
            }

            // Check if source port is one of motor's ports
            for (Port* motor_port : comp.get_ports()) {
                if (conn->source == motor_port) {
                    source_port_is_motors = true;
                    break;
                }
            }

            spdlog::debug("[ACTUATOR] Connection {}/{} - motor_is_source: {}, motor_is_target: {}, source_port_is_motors: {}, target_port_is_motors: {}",
                        checked_connections, m_connections.size(),
                        motor_is_source, motor_is_target, source_port_is_motors, target_port_is_motors);

            // Check BOTH directions: source->target AND target->source
            Component* connected_comp = nullptr;
            Port* connected_port = nullptr;
            Port* actuator_port = nullptr;

            // Direction 1: conn->source -> conn->target
            if (motor_is_target || target_port_is_motors) {
                spdlog::debug("[ACTUATOR] Direction 1: motor is target");
                connected_comp = conn->source->owner();
                connected_port = conn->source;
                actuator_port = conn->target;
            }
            // Direction 2: conn->target -> conn->source (reverse)
            else if (motor_is_source || source_port_is_motors) {
                spdlog::debug("[ACTUATOR] Direction 2: motor is source");
                connected_comp = conn->target->owner();
                connected_port = conn->target;
                actuator_port = conn->source;
            } else {
                spdlog::debug("[ACTUATOR] Connection {}/{} - motor is neither source nor target, skipping",
                           checked_connections, m_connections.size());
                continue;
            }

            if (!connected_comp || !connected_port || !actuator_port) continue;

            // If connected to a voltage source or circuit component
            if (connected_comp->category() == "power" ||
                connected_comp->category() == "passive" ||
                connected_comp->category() == "semiconductor") {

                // Log the connection details
                spdlog::debug("[ACTUATOR] Checking connection: {}:{} -> {}:{} (category: {})",
                            comp.id(), actuator_port->name(),
                            connected_comp->id(), connected_port->name(),
                            connected_comp->category());

                // Try to get voltage from the connected component's port
                if (const float* val = connected_port->get_value<float>()) {
                    std::string_view actuator_port_name = actuator_port->name();

                    if (actuator_port_name == "V+") {
                        v_plus_voltage = *val;
                        spdlog::debug("[ACTUATOR] Propagating V+: {} -> {} = {}V",
                                     connected_comp->id(), comp.id(), *val);
                    } else if (actuator_port_name == "GND") {
                        gnd_voltage = *val;
                        spdlog::debug("[ACTUATOR] Propagating GND: {} -> {} = {}V",
                                     connected_comp->id(), comp.id(), *val);
                    }
                } else {
                    // Log when we can't get voltage value
                    std::string_view actuator_port_name = actuator_port->name();
                    spdlog::debug("[ACTUATOR] Could not get voltage value from {}:{} (connected to {}:{})",
                                connected_comp->id(), connected_port->name(),
                                comp.id(), actuator_port_name);

                    // Try to read the port value anyway to see what we get
                    if (connected_port) {
                        spdlog::debug("[ACTUATOR] Port {}:{} exists but get_value<float>() returned null",
                                    connected_comp->id(), connected_port->name());
                    }
                }
            } else {
                spdlog::debug("[ACTUATOR] Skipping connection to {} (category: {})",
                             connected_comp->id(), connected_comp->category());
            }
        }

        // Update actuator port values
        v_plus_port->set_value(v_plus_voltage);
        gnd_port->set_value(gnd_voltage);

        // Debug log when voltages change
        static std::unordered_map<std::string, float> last_v_plus;
        static std::unordered_map<std::string, float> last_gnd;

        if (last_v_plus[comp.id()] != v_plus_voltage || last_gnd[comp.id()] != gnd_voltage) {
            spdlog::debug("[ACTUATOR] Updated {} port voltages: V+ = {}V, GND = {}V",
                        comp.id(), v_plus_voltage, gnd_voltage);
            last_v_plus[comp.id()] = v_plus_voltage;
            last_gnd[comp.id()] = gnd_voltage;
        }
    });

    spdlog::debug("[ACTUATOR] ===== PROPAGATE VOLTAGES TO ACTUATORS END =====");
    spdlog::debug("[ACTUATOR] propagate_voltages_to_actuators() completed - found {} actuators", actuator_count);
}

} // namespace mechatron
