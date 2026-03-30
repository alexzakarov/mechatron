#include "SimulationOrchestrator.hpp"
#include "physics/PhysicsWorld.hpp"
#include <spdlog/spdlog.h>

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

    // Only run simulation steps when not stopped
    auto state = m_time.state();
    if (state == SimulationState::Stopped) return;

    double dt = m_time.physics_step_size();

    // Step 1: Circuit-physics bridge update (voltages → actuator inputs)
    m_circuit_bridge.update(dt);

    // Step 2: Update all components (actuator calculations, force generation, etc.)
    m_registry.for_each([dt](Component& comp) {
        comp.update(dt);
    });

    // Step 3: Component → Physics: apply forces/torques to physics bodies
    if (m_physics) {
        m_registry.for_each([this](Component& comp) {
            PhysicsBody* body = comp.physics_body();
            if (!body || body->is_static) return;

            // Sync component transform position to physics body
            body->position = comp.transform().position;
        });

        // Step 4: Physics step
        m_physics->step(dt);

        // Step 5: Physics → Component: write back positions
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

std::unique_ptr<Connection> SimulationOrchestrator::connect(Port* source, Port* target) {
    auto conn = std::make_unique<Connection>(source, target);
    Connection* ptr = conn.get();
    m_connections.push_back(std::move(conn));
    // Return ownership not needed - connections managed here
    // But we return raw for convenience
    return std::make_unique<Connection>(source, target);
}

void SimulationOrchestrator::disconnect(Connection* conn) {
    // Find and remove the connection
    for (auto it = m_connections.begin(); it != m_connections.end(); ++it) {
        if (it->get() == conn) {
            m_connections.erase(it);
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

} // namespace mechatron
