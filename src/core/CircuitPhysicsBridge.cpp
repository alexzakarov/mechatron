#include "CircuitPhysicsBridge.hpp"
#include "SimulationOrchestrator.hpp"
#include "Registry.hpp"
#include "actuators/Actuator.hpp"
#include "sensors/Sensor.hpp"
#include "electronics/CircuitSimulator.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>

namespace mechatron {

CircuitPhysicsBridge::CircuitPhysicsBridge() {
    spdlog::info("Circuit-Physics Bridge initialized");
}

void CircuitPhysicsBridge::add_mapping(const PinMapping& mapping) {
    m_mappings[mapping.circuit_pin_id] = mapping;
    spdlog::info("Added pin mapping: {} -> {} (type: {})",
        mapping.circuit_pin_id, mapping.target_component_id,
        static_cast<int>(mapping.type));
}

void CircuitPhysicsBridge::remove_mapping(const std::string& circuit_pin_id) {
    auto it = m_mappings.find(circuit_pin_id);
    if (it != m_mappings.end()) {
        spdlog::info("Removed pin mapping: {}", circuit_pin_id);
        m_mappings.erase(it);
    }
}

void CircuitPhysicsBridge::update(double dt) {
    if (!m_enabled || !m_circuit_sim || !m_registry) {
        return;
    }

    m_active_mappings = 0;

    // Process all mappings
    for (const auto& [pin_id, mapping] : m_mappings) {
        process_mapping(mapping);
        m_active_mappings++;
    }
}

void CircuitPhysicsBridge::process_mapping(const PinMapping& mapping) {
    // Find the circuit pin
    CircuitPin* pin = find_pin(mapping.circuit_pin_id);
    if (!pin) {
        return; // Pin not found, skip
    }

    // Get voltage from pin
    float voltage = pin->voltage;

    // Normalize voltage to output range
    float normalized_value = normalize_voltage(voltage, mapping);

    // Apply to target based on mapping type
    switch (mapping.type) {
        case PinMappingType::VoltageToActuatorInput:
        case PinMappingType::DigitalToEnable:
        case PinMappingType::PWMToSpeed:
            apply_to_actuator(mapping.target_component_id, normalized_value, mapping.type);
            break;

        case PinMappingType::SensorToDigitalPin:
        case PinMappingType::SensorToAnalogPin:
            apply_to_sensor(mapping.target_component_id, normalized_value, mapping.type);
            break;

        case PinMappingType::VoltageToForce:
        case PinMappingType::VoltageToTorque:
            apply_to_physics(mapping.target_component_id, normalized_value, mapping.type);
            break;
    }
}

float CircuitPhysicsBridge::normalize_voltage(float voltage, const PinMapping& mapping) const {
    // Clamp voltage to input range
    voltage = std::max(mapping.voltage_min, std::min(mapping.voltage_max, voltage));

    // Normalize to 0-1 range
    float normalized = (voltage - mapping.voltage_min) / (mapping.voltage_max - mapping.voltage_min);

    // Scale to output range
    return mapping.output_min + normalized * (mapping.output_max - mapping.output_min);
}

void CircuitPhysicsBridge::apply_to_actuator(const std::string& component_id, float value,
                                              PinMappingType type) {
    if (!m_registry) return;

    Component* comp = m_registry->get(component_id);
    if (!comp) {
        return; // Component not found
    }

    Actuator* actuator = dynamic_cast<Actuator*>(comp);
    if (!actuator) {
        spdlog::warn("Component {} is not an actuator", component_id);
        return;
    }

    switch (type) {
        case PinMappingType::VoltageToActuatorInput:
            // Direct voltage control (0-1)
            actuator->set_input(value);
            break;

        case PinMappingType::DigitalToEnable:
            // Digital enable (threshold at 0.5)
            actuator->set_enabled(value > 0.5f);
            break;

        case PinMappingType::PWMToSpeed:
            // PWM speed control
            actuator->set_input(value);
            break;

        default:
            break;
    }
}

void CircuitPhysicsBridge::apply_to_sensor(const std::string& component_id, float value,
                                            PinMappingType type) {
    // Sensor-to-circuit mapping is handled during sensor update
    // This is a placeholder for future implementation
}

void CircuitPhysicsBridge::apply_to_physics(const std::string& component_id, float value,
                                             PinMappingType type) {
    // Direct physics application
    // This would apply force/torque directly to physics bodies
    // For now, this is a placeholder
    spdlog::trace("Applying physics: {} value={} type={}", component_id, value, static_cast<int>(type));
}

CircuitPin* CircuitPhysicsBridge::find_pin(const std::string& pin_id) {
    if (!m_circuit_sim) return nullptr;

    // Parse pin_id format: "component_id.pin_name"
    size_t dot_pos = pin_id.find('.');
    if (dot_pos == std::string::npos) {
        return nullptr;
    }

    std::string comp_id = pin_id.substr(0, dot_pos);
    std::string pin_name = pin_id.substr(dot_pos + 1);

    // Get component from circuit simulator
    CircuitComponent* comp = m_circuit_sim->get_component(comp_id);
    if (!comp) {
        return nullptr;
    }

    // Get pins from component
    auto pins = comp->get_pins();
    for (CircuitPin* pin : pins) {
        if (pin->id == pin_name) {
            return pin;
        }
    }

    return nullptr;
}

} // namespace mechatron
