#pragma once

#include "core/Component.hpp"
#include "core/Port.hpp"
#include "CircuitSimulator.hpp"
#include <memory>
#include <vector>
#include <map>

namespace mechatron {

/**
 * @brief Maps CircuitPin types to Port domains
 */
inline PortDomain map_pin_type_to_domain(PinType pin_type) {
    switch (pin_type) {
        case PinType::Digital:   return PortDomain::Digital;
        case PinType::Analog:    return PortDomain::Analog;
        case PinType::Power:     return PortDomain::Electrical;
        case PinType::Ground:    return PortDomain::Electrical;
        default:                 return PortDomain::Analog;
    }
}

/**
 * @brief Maps PinDirection to PortDirection
 */
inline PortDirection map_pin_direction(PinDirection pin_dir) {
    switch (pin_dir) {
        case PinDirection::Input:         return PortDirection::Input;
        case PinDirection::Output:        return PortDirection::Output;
        case PinDirection::Bidirectional: return PortDirection::Bidirectional;
        default:                          return PortDirection::Bidirectional;
    }
}

/**
 * @brief Adapter to make CircuitComponent compatible with Component interface
 *
 * Bridges the gap between CircuitComponent (circuit simulation) and Component (plugin architecture).
 * Each electronic component can be wrapped to work with the plugin system.
 *
 * Ports are automatically created from CircuitPin definitions, enabling proper
 * electrical connections between components in the simulation.
 */
template<typename CircuitComp>
class CircuitComponentAdapter : public Component {
public:
    explicit CircuitComponentAdapter(std::unique_ptr<CircuitComp> circuit_comp)
        : m_circuit_component(std::move(circuit_comp)) {
        // Create ports from circuit pins during construction
        create_ports_from_pins();
    }

    ~CircuitComponentAdapter() override = default;

    std::string_view plugin_type() const override {
        return m_circuit_component->category();
    }

    std::string_view component_type() const override {
        return m_circuit_component->type();
    }

    std::string_view category() const override {
        return m_circuit_component->category();
    }

    void update(double dt) override {
        // Sync port values to circuit pins before update
        sync_ports_to_pins();
        m_circuit_component->update(dt);
        // Sync circuit pin values back to ports after update
        sync_pins_to_ports();
    }

    void serialize(nlohmann::json& out) const override {
        out["circuit_type"] = std::string(m_circuit_component->type());
        // Circuit components may have their own serialization
    }

    void deserialize(const nlohmann::json& in) override {
        // Load circuit component specific data
    }

    std::vector<Port*> get_ports() override {
        std::vector<Port*> result;
        for (auto& port : m_ports) {
            result.push_back(port.get());
        }
        return result;
    }

    CircuitComp* circuit_component() { return m_circuit_component.get(); }
    const CircuitComp* circuit_component() const { return m_circuit_component.get(); }

private:
    void create_ports_from_pins() {
        auto pins = m_circuit_component->get_pins();
        for (CircuitPin* pin : pins) {
            if (!pin) continue;

            PortDomain domain = map_pin_type_to_domain(pin->type);
            PortDirection direction = map_pin_direction(pin->direction);

            auto port = std::make_unique<Port>(
                pin->id,
                domain,
                direction
            );

            // Initialize port value from pin state
            if (pin->type == PinType::Digital) {
                port->set_value(pin->digital_state);
            } else {
                port->set_value(pin->voltage);
            }

            // Set owner after port is created (friend access)
            port->m_owner = this;

            m_ports.push_back(std::move(port));
            m_pin_to_port_index[pin->id] = static_cast<int>(m_ports.size() - 1);
        }
    }

    void sync_ports_to_pins() {
        auto pins = m_circuit_component->get_pins();
        for (CircuitPin* pin : pins) {
            if (!pin) continue;

            auto it = m_pin_to_port_index.find(pin->id);
            if (it == m_pin_to_port_index.end()) continue;

            Port* port = m_ports[it->second].get();

            // Read port value and write to pin
            if (pin->type == PinType::Digital) {
                if (const bool* val = port->get_value<bool>()) {
                    pin->digital_state = *val;
                }
            } else {
                if (const float* val = port->get_value<float>()) {
                    pin->voltage = *val;
                }
            }
        }
    }

    void sync_pins_to_ports() {
        auto pins = m_circuit_component->get_pins();
        for (CircuitPin* pin : pins) {
            if (!pin) continue;

            auto it = m_pin_to_port_index.find(pin->id);
            if (it == m_pin_to_port_index.end()) continue;

            Port* port = m_ports[it->second].get();

            // Read pin value and write to port
            if (pin->type == PinType::Digital) {
                port->set_value(pin->digital_state);
            } else {
                port->set_value(pin->voltage);
            }
        }
    }

    std::unique_ptr<CircuitComp> m_circuit_component;

    // Ports created from circuit pins
    std::vector<std::unique_ptr<Port>> m_ports;

    // Map pin ID to port index for quick lookup
    std::map<std::string, int, std::less<>> m_pin_to_port_index;
};

// Specific adapter instantiations for common circuit components
using ResistorComponent = CircuitComponentAdapter<Resistor>;
using CapacitorComponent = CircuitComponentAdapter<Capacitor>;
using InductorComponent = CircuitComponentAdapter<Inductor>;
using DiodeComponent = CircuitComponentAdapter<Diode>;
using ZenerDiodeComponent = CircuitComponentAdapter<ZenerDiode>;
using LEDComponent = CircuitComponentAdapter<LED>;
using BJTComponent = CircuitComponentAdapter<BJTTransistor>;
using MOSFETComponent = CircuitComponentAdapter<MOSFETTransistor>;
using BuckConverterComponent = CircuitComponentAdapter<BuckConverter>;
using BoostConverterComponent = CircuitComponentAdapter<BoostConverter>;
using MotorDriverComponent = CircuitComponentAdapter<MotorDriver>;
using HBridgeComponent = CircuitComponentAdapter<HBridge>;
using DCVoltageComponent = CircuitComponentAdapter<DCVoltageSource>;
using GroundComponent = CircuitComponentAdapter<Ground>;

} // namespace mechatron
