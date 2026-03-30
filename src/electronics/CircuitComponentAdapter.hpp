#pragma once

#include "core/Component.hpp"
#include "CircuitSimulator.hpp"
#include <memory>

namespace mechatron {

/**
 * @brief Adapter to make CircuitComponent compatible with Component interface
 *
 * Bridges the gap between CircuitComponent (circuit simulation) and Component (plugin architecture).
 * Each electronic component can be wrapped to work with the plugin system.
 */
template<typename CircuitComp>
class CircuitComponentAdapter : public Component {
public:
    explicit CircuitComponentAdapter(std::unique_ptr<CircuitComp> circuit_comp)
        : m_circuit_component(std::move(circuit_comp)) {}

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
        m_circuit_component->update(dt);
    }

    void serialize(nlohmann::json& out) const override {
        out["circuit_type"] = std::string(m_circuit_component->type());
        // Circuit components may have their own serialization
    }

    void deserialize(const nlohmann::json& in) override {
        // Load circuit component specific data
    }

    std::vector<Port*> get_ports() override {
        // Map CircuitPin to Port
        // This needs Port implementation for circuit pins
        return {};
    }

    CircuitComp* circuit_component() { return m_circuit_component.get(); }
    const CircuitComp* circuit_component() const { return m_circuit_component.get(); }

private:
    std::unique_ptr<CircuitComp> m_circuit_component;
};

// Specific adapter instantiations for common circuit components
using ResistorComponent = CircuitComponentAdapter<Resistor>;
using CapacitorComponent = CircuitComponentAdapter<Capacitor>;
using LEDComponent = CircuitComponentAdapter<LED>;
using HBridgeComponent = CircuitComponentAdapter<HBridge>;

} // namespace mechatron
