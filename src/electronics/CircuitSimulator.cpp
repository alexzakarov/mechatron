#include "CircuitSimulator.hpp"
#include <spdlog/spdlog.h>

namespace mechatron {

CircuitSimulator::CircuitSimulator() {
    spdlog::info("CircuitSimulator initialized");
}

bool CircuitSimulator::remove_component(std::string_view id) {
    std::string id_str(id);
    auto it = m_components.find(id_str);
    if (it == m_components.end()) return false;

    // Remove connected wires
    std::vector<std::string> wires_to_remove;
    for (auto& [wire_id, wire] : m_wires) {
        if (wire->source && wire->source->component_id == id_str ||
            wire->target && wire->target->component_id == id_str) {
            wires_to_remove.push_back(wire_id);
        }
    }
    for (const auto& wire_id : wires_to_remove) {
        m_wires.erase(wire_id);
    }

    m_components.erase(it);
    spdlog::debug("Removed circuit component: {}", id);
    return true;
}

CircuitComponent* CircuitSimulator::get_component(std::string_view id) {
    auto it = m_components.find(std::string(id));
    return it != m_components.end() ? it->second.get() : nullptr;
}

bool CircuitSimulator::connect(const std::string& wire_id,
                                const std::string& source_comp,
                                const std::string& source_pin,
                                const std::string& target_comp,
                                const std::string& target_pin) {
    // Find source pin
    auto* src_comp = get_component(source_comp);
    if (!src_comp) {
        spdlog::error("Source component not found: {}", source_comp);
        return false;
    }

    auto src_pins = src_comp->get_pins();
    CircuitPin* src_pin_ptr = nullptr;
    for (auto* pin : src_pins) {
        if (pin->id == source_pin) {
            src_pin_ptr = pin;
            break;
        }
    }

    if (!src_pin_ptr) {
        spdlog::error("Source pin not found: {}.{}", source_comp, source_pin);
        return false;
    }

    // Find target pin
    auto* tgt_comp = get_component(target_comp);
    if (!tgt_comp) {
        spdlog::error("Target component not found: {}", target_comp);
        return false;
    }

    auto tgt_pins = tgt_comp->get_pins();
    CircuitPin* tgt_pin_ptr = nullptr;
    for (auto* pin : tgt_pins) {
        if (pin->id == target_pin) {
            tgt_pin_ptr = pin;
            break;
        }
    }

    if (!tgt_pin_ptr) {
        spdlog::error("Target pin not found: {}.{}", target_comp, target_pin);
        return false;
    }

    // Create wire
    auto wire = std::make_unique<Wire>();
    wire->id = wire_id;
    wire->source = src_pin_ptr;
    wire->target = tgt_pin_ptr;
    m_wires[wire_id] = std::move(wire);

    spdlog::debug("Connected {}:{} -> {}:{}", source_comp, source_pin, target_comp, target_pin);
    return true;
}

bool CircuitSimulator::disconnect(const std::string& wire_id) {
    auto it = m_wires.find(wire_id);
    if (it == m_wires.end()) return false;

    m_wires.erase(it);
    spdlog::debug("Disconnected wire: {}", wire_id);
    return true;
}

void CircuitSimulator::step(double dt) {
    // Update all components
    for (auto& [id, comp] : m_components) {
        comp->update(dt);
    }

    // Propagate signals through wires
    for (auto& [id, wire] : m_wires) {
        if (!wire->source || !wire->target) continue;

        // Simple voltage propagation (ideal wire)
        wire->target->voltage = wire->source->voltage;

        // Current calculation (Ohm's law: I = V/R)
        float voltage_diff = wire->source->voltage - wire->target->voltage;
        wire->source->current = voltage_diff / wire->resistance;
        wire->target->current = wire->source->current;

        // Digital state propagation
        if (wire->source->type == PinType::Digital) {
            wire->target->digital_state = wire->source->digital_state;
        }
    }
}

void CircuitSimulator::reset() {
    for (auto& [id, comp] : m_components) {
        comp->reset();
    }
    spdlog::info("Circuit simulator reset");
}

std::vector<CircuitComponent*> CircuitSimulator::get_all_components() {
    std::vector<CircuitComponent*> result;
    result.reserve(m_components.size());
    for (auto& [id, comp] : m_components) {
        result.push_back(comp.get());
    }
    return result;
}

// ============================================================================
// H-Bridge Implementation
// ============================================================================

HBridge::HBridge(float supply_voltage)
    : m_supply_voltage(supply_voltage)
{
    // Initialize pins
    m_vcc.voltage = supply_voltage;
    m_gnd.voltage = 0.0f;

    spdlog::debug("H-Bridge initialized with {:.1f}V supply", supply_voltage);
}

void HBridge::set_pwm_duty(float duty) {
    m_pwm_duty = std::max(0.0f, std::min(1.0f, duty));
}

void HBridge::update(double dt) {
    // Update PWM phase
    m_pwm_phase += static_cast<float>(dt) * m_pwm_frequency;
    if (m_pwm_phase >= 1.0f) {
        m_pwm_phase -= 1.0f;
    }

    // Determine direction from IN1 and IN2
    bool in1 = m_in1.digital_state;
    bool in2 = m_in2.digital_state;
    bool enabled = m_en.digital_state;

    if (!enabled) {
        // Disabled = coast/brake mode
        m_direction = 0;
        m_output_voltage = 0.0f;
        m_out1.voltage = 0.0f;
        m_out2.voltage = 0.0f;
        return;
    }

    // H-Bridge truth table:
    // IN1=0, IN2=0: Brake (both outputs low)
    // IN1=0, IN2=1: Reverse (OUT1=0, OUT2=VCC)
    // IN1=1, IN2=0: Forward (OUT1=VCC, OUT2=0)
    // IN1=1, IN2=1: Brake (both outputs high)

    if (!in1 && !in2) {
        // Brake to ground
        m_direction = 0;
        m_output_voltage = 0.0f;
        m_out1.voltage = 0.0f;
        m_out2.voltage = 0.0f;
    }
    else if (!in1 && in2) {
        // Reverse
        m_direction = -1;
        float effective_voltage = m_supply_voltage * m_pwm_duty;
        m_output_voltage = -effective_voltage;
        m_out1.voltage = 0.0f;
        m_out2.voltage = effective_voltage;
    }
    else if (in1 && !in2) {
        // Forward
        m_direction = 1;
        float effective_voltage = m_supply_voltage * m_pwm_duty;
        m_output_voltage = effective_voltage;
        m_out1.voltage = effective_voltage;
        m_out2.voltage = 0.0f;
    }
    else { // in1 && in2
        // Brake to VCC
        m_direction = 0;
        m_output_voltage = 0.0f;
        m_out1.voltage = m_supply_voltage;
        m_out2.voltage = m_supply_voltage;
    }
}

void HBridge::reset() {
    m_pwm_phase = 0.0f;
    m_pwm_duty = 0.0f;
    m_direction = 0;
    m_output_voltage = 0.0f;

    m_in1.digital_state = false;
    m_in2.digital_state = false;
    m_en.digital_state = false;

    m_out1.voltage = 0.0f;
    m_out2.voltage = 0.0f;
}

} // namespace mechatron
