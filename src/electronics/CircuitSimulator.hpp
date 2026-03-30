#pragma once

#include "core/Types.hpp"
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <functional>

namespace mechatron {

// Pin direction
enum class PinDirection {
    Input,
    Output,
    Bidirectional
};

// Pin type
enum class PinType {
    Digital,      // Digital logic (0/1)
    Analog,       // Analog voltage (continuous)
    Power,        // Power supply (VCC/GND)
    Ground        // Ground reference
};

// Circuit pin
struct CircuitPin {
    std::string id;
    std::string component_id;
    PinDirection direction;
    PinType type;

    // Current state
    bool digital_state = false;
    float voltage = 0.0f;        // Volts
    float current = 0.0f;        // Amps
    float impedance = 1e6f;      // Ohms (default high impedance)

    // For output pins
    bool is_driven = false;
};

// Wire connection between pins
struct Wire {
    std::string id;
    CircuitPin* source = nullptr;
    CircuitPin* target = nullptr;
    float resistance = 0.001f;   // Wire resistance (Ohms)
};

// Forward declaration
class CircuitSimulator;

// Circuit component base
class CircuitComponent {
    friend class CircuitSimulator;

public:
    virtual ~CircuitComponent() = default;

    virtual std::string_view type() const = 0;
    virtual std::string_view category() const = 0;

    virtual void update(double dt) {}
    virtual void reset() {}

    // Get component pins
    virtual std::vector<CircuitPin*> get_pins() { return {}; }

    // Get/set parameters
    virtual void set_parameter(const std::string& name, double value) {}
    virtual double get_parameter(const std::string& name) const { return 0.0; }

    const std::string& id() const { return m_id; }

protected:
    std::string m_id;
};

// Basic circuit components
class Resistor : public CircuitComponent {
public:
    std::string_view type() const override { return "resistor"; }
    std::string_view category() const override { return "passive"; }

    Resistor(float resistance_ohm = 1000.0f) : m_resistance(resistance_ohm) {}

    std::vector<CircuitPin*> get_pins() override {
        return {&m_pin1, &m_pin2};
    }

    void set_parameter(const std::string& name, double value) override {
        if (name == "resistance") m_resistance = static_cast<float>(value);
    }

    double get_parameter(const std::string& name) const override {
        if (name == "resistance") return m_resistance;
        return 0.0;
    }

private:
    float m_resistance;
    CircuitPin m_pin1{"1", "", PinDirection::Bidirectional, PinType::Analog};
    CircuitPin m_pin2{"2", "", PinDirection::Bidirectional, PinType::Analog};
};

class Capacitor : public CircuitComponent {
public:
    std::string_view type() const override { return "capacitor"; }
    std::string_view category() const override { return "passive"; }

    Capacitor(float capacitance_farad = 1e-6f) : m_capacitance(capacitance_farad) {}

    std::vector<CircuitPin*> get_pins() override {
        return {&m_pin1, &m_pin2};
    }

    void update(double dt) override {
        // Simple capacitor charge model: V = Q/C, I = C * dV/dt
        // For now, just accumulate charge
    }

private:
    float m_capacitance;
    float m_charge = 0.0f;
    CircuitPin m_pin1{"1", "", PinDirection::Bidirectional, PinType::Analog};
    CircuitPin m_pin2{"2", "", PinDirection::Bidirectional, PinType::Analog};
};

// LED component
class LED : public CircuitComponent {
public:
    std::string_view type() const override { return "led"; }
    std::string_view category() const override { return "optoelectronic"; }

    LED(float forward_voltage = 2.0f) : m_forward_voltage(forward_voltage) {}

    std::vector<CircuitPin*> get_pins() override {
        return {&m_anode, &m_cathode};
    }

    bool is_lit() const { return m_is_lit; }

    void update(double dt) override {
        // LED is on if anode voltage > cathode + forward voltage
        float v_diff = m_anode.voltage - m_cathode.voltage;
        m_is_lit = v_diff > m_forward_voltage && m_anode.current > 1e-3f;
    }

private:
    float m_forward_voltage;
    bool m_is_lit = false;
    CircuitPin m_anode{"anode", "", PinDirection::Input, PinType::Analog};
    CircuitPin m_cathode{"cathode", "", PinDirection::Input, PinType::Analog};
};

// H-Bridge motor driver
class HBridge : public CircuitComponent {
public:
    std::string_view type() const override { return "h_bridge"; }
    std::string_view category() const override { return "power"; }

    HBridge(float supply_voltage = 12.0f);

    std::vector<CircuitPin*> get_pins() override {
        return {&m_in1, &m_in2, &m_en, &m_out1, &m_out2, &m_vcc, &m_gnd};
    }

    void set_parameter(const std::string& name, double value) override {
        if (name == "supply_voltage") m_supply_voltage = static_cast<float>(value);
        else if (name == "pwm_frequency") m_pwm_frequency = static_cast<float>(value);
    }

    double get_parameter(const std::string& name) const override {
        if (name == "supply_voltage") return m_supply_voltage;
        if (name == "pwm_frequency") return m_pwm_frequency;
        return 0.0;
    }

    // Control inputs
    void set_in1(bool state) { m_in1.digital_state = state; }
    void set_in2(bool state) { m_in2.digital_state = state; }
    void set_enable(bool state) { m_en.digital_state = state; }
    void set_pwm_duty(float duty); // 0.0 to 1.0

    // Output state
    float get_output_voltage() const { return m_output_voltage; }
    int get_direction() const { return m_direction; } // -1: reverse, 0: brake, 1: forward

    void update(double dt) override;
    void reset() override;

private:
    float m_supply_voltage = 12.0f;
    float m_pwm_frequency = 1000.0f; // Hz
    float m_pwm_duty = 0.0f;
    float m_pwm_phase = 0.0f;

    int m_direction = 0;       // -1, 0, 1
    float m_output_voltage = 0.0f;

    // Control pins
    CircuitPin m_in1{"IN1", "", PinDirection::Input, PinType::Digital};
    CircuitPin m_in2{"IN2", "", PinDirection::Input, PinType::Digital};
    CircuitPin m_en{"EN", "", PinDirection::Input, PinType::Digital};

    // Output pins
    CircuitPin m_out1{"OUT1", "", PinDirection::Output, PinType::Analog};
    CircuitPin m_out2{"OUT2", "", PinDirection::Output, PinType::Analog};

    // Power pins
    CircuitPin m_vcc{"VCC", "", PinDirection::Input, PinType::Power};
    CircuitPin m_gnd{"GND", "", PinDirection::Input, PinType::Ground};
};

// Simple circuit simulator
class CircuitSimulator {
public:
    CircuitSimulator();
    ~CircuitSimulator() = default;

    // Component management
    template<typename T, typename... Args>
    T* add_component(std::string id, Args&&... args) {
        auto comp = std::make_unique<T>(std::forward<Args>(args)...);
        comp->m_id = id;
        T* ptr = comp.get();
        m_components[id] = std::move(comp);
        return ptr;
    }

    bool remove_component(std::string_view id);
    CircuitComponent* get_component(std::string_view id);

    // Wire management
    bool connect(const std::string& wire_id,
                 const std::string& source_comp, const std::string& source_pin,
                 const std::string& target_comp, const std::string& target_pin);
    bool disconnect(const std::string& wire_id);

    // Simulation
    void step(double dt);
    void reset();

    // Query
    std::vector<CircuitComponent*> get_all_components();

private:
    std::unordered_map<std::string, std::unique_ptr<CircuitComponent>> m_components;
    std::unordered_map<std::string, std::unique_ptr<Wire>> m_wires;
};

} // namespace mechatron
