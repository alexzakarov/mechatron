#pragma once

#include "core/Types.hpp"
#include "MNASolver.hpp"
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <functional>

namespace mechatron {

class NgspiceWrapper;

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
    virtual std::vector<CircuitPin*> get_pins() const { return {}; }

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
    static float& default_resistance() {
        static float def = 1000.0f;  // 1kΩ default
        return def;
    }
    static void set_default_resistance(float ohms) { default_resistance() = ohms; }

    std::string_view type() const override { return "resistor"; }
    std::string_view category() const override { return "passive"; }

    Resistor(float resistance_ohm = default_resistance()) : m_resistance(resistance_ohm) {}

    std::vector<CircuitPin*> get_pins() const override {
        return {const_cast<CircuitPin*>(&m_pin1), const_cast<CircuitPin*>(&m_pin2)};
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

    std::vector<CircuitPin*> get_pins() const override {
        return {const_cast<CircuitPin*>(&m_pin1), const_cast<CircuitPin*>(&m_pin2)};
    }

    double get_parameter(const std::string& name) const override {
        if (name == "capacitance") return m_capacitance;
        return 0.0;
    }

    void set_parameter(const std::string& name, double value) override {
        if (name == "capacitance") m_capacitance = static_cast<float>(value);
    }

private:
    float m_capacitance;
    CircuitPin m_pin1{"1", "", PinDirection::Bidirectional, PinType::Analog};
    CircuitPin m_pin2{"2", "", PinDirection::Bidirectional, PinType::Analog};
};

// LED component
class LED : public CircuitComponent {
public:
    // LED color presets with typical forward voltages
    enum class Color {
        Red,        // 1.8V
        Green,      // 2.1V
        Blue,       // 3.3V
        Yellow,     // 2.0V
        White,      // 3.0V
        Infrared,   // 1.2V
        Ultraviolet,// 3.5V
        Custom      // User-specified voltage
    };

    static float default_forward_voltage() { return 2.0f; }
    static void set_default_forward_voltage(float vf) { /* Can be stored in settings */ }
    static float get_forward_voltage_for_color(Color color) {
        switch (color) {
            case Color::Red:        return 1.8f;
            case Color::Green:      return 2.1f;
            case Color::Blue:       return 3.3f;
            case Color::Yellow:     return 2.0f;
            case Color::White:      return 3.0f;
            case Color::Infrared:   return 1.2f;
            case Color::Ultraviolet:return 3.5f;
            case Color::Custom:     return default_forward_voltage();
        }
        return 2.0f;
    }

    std::string_view type() const override { return "led"; }
    std::string_view category() const override { return "optoelectronic"; }

    LED(float forward_voltage = default_forward_voltage(), Color color = Color::Custom)
        : m_forward_voltage(forward_voltage), m_color(color) {}

    LED(Color color) : LED(get_forward_voltage_for_color(color), color) {}

    Color get_color() const { return m_color; }
    void set_color(Color color) {
        m_color = color;
        if (color != Color::Custom) {
            m_forward_voltage = get_forward_voltage_for_color(color);
        }
    }

    std::vector<CircuitPin*> get_pins() const override {
        return {const_cast<CircuitPin*>(&m_anode), const_cast<CircuitPin*>(&m_cathode)};
    }

    bool is_lit() const { return m_is_lit; }

private:
    float m_forward_voltage;
    Color m_color = Color::Custom;
    bool m_is_lit = false;
    CircuitPin m_anode{"anode", "", PinDirection::Input, PinType::Analog};
    CircuitPin m_cathode{"cathode", "", PinDirection::Input, PinType::Analog};
};

// Inductor
class Inductor : public CircuitComponent {
public:
    std::string_view type() const override { return "inductor"; }
    std::string_view category() const override { return "passive"; }

    Inductor(float inductance_henry = 1e-3f) : m_inductance(inductance_henry) {}

    std::vector<CircuitPin*> get_pins() const override {
        return {const_cast<CircuitPin*>(&m_pin1), const_cast<CircuitPin*>(&m_pin2)};
    }

    void set_parameter(const std::string& name, double value) override {
        if (name == "inductance") m_inductance = static_cast<float>(value);
    }

    double get_parameter(const std::string& name) const override {
        if (name == "inductance") return m_inductance;
        return 0.0;
    }

private:
    float m_inductance;
    CircuitPin m_pin1{"1", "", PinDirection::Bidirectional, PinType::Analog};
    CircuitPin m_pin2{"2", "", PinDirection::Bidirectional, PinType::Analog};
};

// Diode
class Diode : public CircuitComponent {
public:
    std::string_view type() const override { return "diode"; }
    std::string_view category() const override { return "semiconductor"; }

    Diode(float forward_voltage = 0.7f, float on_resistance = 0.1f)
        : m_forward_voltage(forward_voltage), m_on_resistance(on_resistance) {}

    std::vector<CircuitPin*> get_pins() const override {
        return {const_cast<CircuitPin*>(&m_anode), const_cast<CircuitPin*>(&m_cathode)};
    }

    bool is_conducting() const { return m_conducting; }

    void set_parameter(const std::string& name, double value) override {
        if (name == "forward_voltage") m_forward_voltage = static_cast<float>(value);
        else if (name == "on_resistance") m_on_resistance = static_cast<float>(value);
    }

    double get_parameter(const std::string& name) const override {
        if (name == "forward_voltage") return m_forward_voltage;
        if (name == "on_resistance") return m_on_resistance;
        return 0.0;
    }

private:
    float m_forward_voltage;
    float m_on_resistance;
    bool m_conducting = false;
    CircuitPin m_anode{"anode", "", PinDirection::Bidirectional, PinType::Analog};
    CircuitPin m_cathode{"cathode", "", PinDirection::Bidirectional, PinType::Analog};
};

// Zener Diode
class ZenerDiode : public CircuitComponent {
public:
    std::string_view type() const override { return "zener_diode"; }
    std::string_view category() const override { return "semiconductor"; }

    ZenerDiode(float zener_voltage = 3.3f, float forward_voltage = 0.7f)
        : m_zener_voltage(zener_voltage), m_forward_voltage(forward_voltage) {}

    std::vector<CircuitPin*> get_pins() const override {
        return {const_cast<CircuitPin*>(&m_anode), const_cast<CircuitPin*>(&m_cathode)};
    }

    void set_parameter(const std::string& name, double value) override {
        if (name == "zener_voltage") m_zener_voltage = static_cast<float>(value);
        else if (name == "forward_voltage") m_forward_voltage = static_cast<float>(value);
    }

    double get_parameter(const std::string& name) const override {
        if (name == "zener_voltage") return m_zener_voltage;
        if (name == "forward_voltage") return m_forward_voltage;
        return 0.0;
    }

private:
    float m_zener_voltage;
    float m_forward_voltage;
    CircuitPin m_anode{"anode", "", PinDirection::Bidirectional, PinType::Analog};
    CircuitPin m_cathode{"cathode", "", PinDirection::Bidirectional, PinType::Analog};
};

// BJT Transistor (NPN or PNP)
class BJTTransistor : public CircuitComponent {
public:
    enum Type { NPN, PNP };

    std::string_view type() const override { return m_bjt_type == NPN ? "bjt_npn" : "bjt_pnp"; }
    std::string_view category() const override { return "semiconductor"; }

    BJTTransistor(Type bjt_type = NPN, float beta = 100.0f)
        : m_bjt_type(bjt_type), m_beta(beta) {}

    std::vector<CircuitPin*> get_pins() const override {
        return {const_cast<CircuitPin*>(&m_base), const_cast<CircuitPin*>(&m_collector), const_cast<CircuitPin*>(&m_emitter)};
    }

    void set_parameter(const std::string& name, double value) override {
        if (name == "beta") m_beta = static_cast<float>(value);
    }

    double get_parameter(const std::string& name) const override {
        if (name == "beta") return m_beta;
        return 0.0;
    }

private:
    Type m_bjt_type;
    float m_beta;
    CircuitPin m_base{"base", "", PinDirection::Input, PinType::Analog};
    CircuitPin m_collector{"collector", "", PinDirection::Bidirectional, PinType::Analog};
    CircuitPin m_emitter{"emitter", "", PinDirection::Bidirectional, PinType::Analog};
};

// MOSFET Transistor (N-Channel or P-Channel enhancement)
class MOSFETTransistor : public CircuitComponent {
public:
    enum Type { NChannel, PChannel };

    std::string_view type() const override { return m_mos_type == NChannel ? "mosfet_n" : "mosfet_p"; }
    std::string_view category() const override { return "semiconductor"; }

    MOSFETTransistor(Type mos_type = NChannel, float threshold_voltage = 2.0f, float kp = 1.0f)
        : m_mos_type(mos_type), m_vth(threshold_voltage), m_kp(kp) {}

    std::vector<CircuitPin*> get_pins() const override {
        return {const_cast<CircuitPin*>(&m_gate), const_cast<CircuitPin*>(&m_drain), const_cast<CircuitPin*>(&m_source)};
    }

    void set_parameter(const std::string& name, double value) override {
        if (name == "threshold_voltage") m_vth = static_cast<float>(value);
        else if (name == "kp") m_kp = static_cast<float>(value);
    }

    double get_parameter(const std::string& name) const override {
        if (name == "threshold_voltage") return m_vth;
        if (name == "kp") return m_kp;
        return 0.0;
    }

private:
    Type m_mos_type;
    float m_vth;
    float m_kp;
    CircuitPin m_gate{"gate", "", PinDirection::Input, PinType::Analog};
    CircuitPin m_drain{"drain", "", PinDirection::Bidirectional, PinType::Analog};
    CircuitPin m_source{"source", "", PinDirection::Bidirectional, PinType::Analog};
};

// Buck Converter (step-down DC-DC)
class BuckConverter : public CircuitComponent {
public:
    std::string_view type() const override { return "buck_converter"; }
    std::string_view category() const override { return "power"; }

    BuckConverter(float input_voltage = 12.0f, float duty_cycle = 0.5f)
        : m_input_voltage(input_voltage), m_duty_cycle(duty_cycle) {}

    std::vector<CircuitPin*> get_pins() const override {
        return {const_cast<CircuitPin*>(&m_vin), const_cast<CircuitPin*>(&m_gnd), const_cast<CircuitPin*>(&m_vout)};
    }

    void set_parameter(const std::string& name, double value) override {
        if (name == "input_voltage") m_input_voltage = static_cast<float>(value);
        else if (name == "duty_cycle") m_duty_cycle = static_cast<float>(value);
    }

    double get_parameter(const std::string& name) const override {
        if (name == "input_voltage") return m_input_voltage;
        if (name == "duty_cycle") return m_duty_cycle;
        if (name == "output_voltage") return m_output_voltage;
        return 0.0;
    }

    void update(double dt) override {
        // After ngspice simulation, read the VOUT pin voltage and update output parameter
        // The ngspice simulation calculates the actual output voltage based on the buck converter circuit
        m_output_voltage = m_vout.voltage;

        // Also update VIN pin voltage from input (for reference)
        // This ensures the pin voltages reflect the circuit state
    }

private:
    float m_input_voltage;
    float m_duty_cycle;
    float m_output_voltage = 0.0f;
    CircuitPin m_vin{"VIN", "", PinDirection::Input, PinType::Power};
    CircuitPin m_gnd{"GND", "", PinDirection::Input, PinType::Ground};
    CircuitPin m_vout{"VOUT", "", PinDirection::Output, PinType::Analog};
};

// Boost Converter (step-up DC-DC)
class BoostConverter : public CircuitComponent {
public:
    std::string_view type() const override { return "boost_converter"; }
    std::string_view category() const override { return "power"; }

    BoostConverter(float input_voltage = 5.0f, float duty_cycle = 0.5f)
        : m_input_voltage(input_voltage), m_duty_cycle(duty_cycle) {}

    std::vector<CircuitPin*> get_pins() const override {
        return {const_cast<CircuitPin*>(&m_vin), const_cast<CircuitPin*>(&m_gnd), const_cast<CircuitPin*>(&m_vout)};
    }

    void set_parameter(const std::string& name, double value) override {
        if (name == "input_voltage") m_input_voltage = static_cast<float>(value);
        else if (name == "duty_cycle") m_duty_cycle = static_cast<float>(value);
    }

    double get_parameter(const std::string& name) const override {
        if (name == "input_voltage") return m_input_voltage;
        if (name == "duty_cycle") return m_duty_cycle;
        if (name == "output_voltage") return m_output_voltage;
        return 0.0;
    }

    void update(double dt) override {
        // After ngspice simulation, read the VOUT pin voltage and update output parameter
        // The ngspice simulation calculates the actual output voltage based on the boost converter circuit
        m_output_voltage = m_vout.voltage;

        // Also update VIN pin voltage from input (for reference)
        // This ensures the pin voltages reflect the circuit state
    }

private:
    float m_input_voltage;
    float m_duty_cycle;
    float m_output_voltage = 0.0f;
    CircuitPin m_vin{"VIN", "", PinDirection::Input, PinType::Power};
    CircuitPin m_gnd{"GND", "", PinDirection::Input, PinType::Ground};
    CircuitPin m_vout{"VOUT", "", PinDirection::Output, PinType::Analog};
};

// Motor Driver (PWM/DIR/EN input -> differential voltage output)
class MotorDriver : public CircuitComponent {
public:
    static float& default_supply_voltage() {
        static float def = 12.0f;  // 12V default
        return def;
    }
    static void set_default_supply_voltage(float volts) { default_supply_voltage() = volts; }

    std::string_view type() const override { return "motor_driver"; }
    std::string_view category() const override { return "power"; }

    MotorDriver(float supply_voltage = default_supply_voltage()) : m_supply_voltage(supply_voltage) {}

    std::vector<CircuitPin*> get_pins() const override {
        return {const_cast<CircuitPin*>(&m_pwm), const_cast<CircuitPin*>(&m_dir), const_cast<CircuitPin*>(&m_en),
                const_cast<CircuitPin*>(&m_out_pos), const_cast<CircuitPin*>(&m_out_neg),
                const_cast<CircuitPin*>(&m_vcc), const_cast<CircuitPin*>(&m_gnd)};
    }

    void set_parameter(const std::string& name, double value) override {
        if (name == "supply_voltage") m_supply_voltage = static_cast<float>(value);
    }

    double get_parameter(const std::string& name) const override {
        if (name == "supply_voltage") return m_supply_voltage;
        if (name == "output_voltage") return m_output_voltage;
        return 0.0;
    }

    void update(double dt) override {
        // After ngspice simulation, read the output pin voltages and update output parameter
        // Motor driver output is differential: OUT+ relative to OUT-
        float out_pos_voltage = m_out_pos.voltage;
        float out_neg_voltage = m_out_neg.voltage;
        m_output_voltage = out_pos_voltage - out_neg_voltage;
    }

private:
    float m_supply_voltage;
    float m_output_voltage = 0.0f;
    CircuitPin m_pwm{"PWM", "", PinDirection::Input, PinType::Analog};
    CircuitPin m_dir{"DIR", "", PinDirection::Input, PinType::Digital};
    CircuitPin m_en{"EN", "", PinDirection::Input, PinType::Digital};
    CircuitPin m_out_pos{"OUT+", "", PinDirection::Output, PinType::Analog};
    CircuitPin m_out_neg{"OUT-", "", PinDirection::Output, PinType::Analog};
    CircuitPin m_vcc{"VCC", "", PinDirection::Input, PinType::Power};
    CircuitPin m_gnd{"GND", "", PinDirection::Input, PinType::Ground};
};

// DC Voltage Source (power supply)
class DCVoltageSource : public CircuitComponent {
public:
    static float& default_voltage() {
        static float def = 5.0f;  // 5V default
        return def;
    }
    static void set_default_voltage(float volts) { default_voltage() = volts; }

    std::string_view type() const override { return "dc_voltage"; }
    std::string_view category() const override { return "power"; }

    DCVoltageSource(float voltage = default_voltage()) : m_voltage(voltage) {}

    std::vector<CircuitPin*> get_pins() const override {
        return {const_cast<CircuitPin*>(&m_vout), const_cast<CircuitPin*>(&m_gnd)};
    }

    void set_parameter(const std::string& name, double value) override {
        if (name == "voltage") m_voltage = static_cast<float>(value);
    }

    double get_parameter(const std::string& name) const override {
        if (name == "voltage") return m_voltage;
        return 0.0;
    }

    void update(double dt) override {
        // Update pin voltages based on voltage parameter
        // V+ pin outputs the voltage, GND pin outputs 0V
        m_vout.voltage = m_voltage;
        m_gnd.voltage = 0.0f;
    }

private:
    float m_voltage;
    CircuitPin m_vout{"V+", "", PinDirection::Output, PinType::Power};
    CircuitPin m_gnd{"GND", "", PinDirection::Output, PinType::Analog};  // Changed from Ground to Analog
};

// Ground - 0V reference node (single terminal)
class Ground : public CircuitComponent {
public:
    std::string_view type() const override { return "ground"; }
    std::string_view category() const override { return "power"; }

    Ground() {}

    std::vector<CircuitPin*> get_pins() const override {
        return {const_cast<CircuitPin*>(&m_gnd)};
    }

private:
    CircuitPin m_gnd{"GND", "", PinDirection::Output, PinType::Ground};
};

// H-Bridge motor driver
class HBridge : public CircuitComponent {
public:
    std::string_view type() const override { return "h_bridge"; }
    std::string_view category() const override { return "power"; }

    HBridge(float supply_voltage = 12.0f);

    std::vector<CircuitPin*> get_pins() const override {
        return {const_cast<CircuitPin*>(&m_in1), const_cast<CircuitPin*>(&m_in2), const_cast<CircuitPin*>(&m_en),
                const_cast<CircuitPin*>(&m_out1), const_cast<CircuitPin*>(&m_out2),
                const_cast<CircuitPin*>(&m_vcc), const_cast<CircuitPin*>(&m_gnd)};
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

// Circuit simulation mode
enum class CircuitSimulationMode {
    NativeMNA,      // Use built-in Modified Nodal Analysis solver (default)
    Ngspice,        // Use ngspice for simulation (if available)
    Hybrid          // Automatically choose based on circuit complexity
};

// Simple circuit simulator
class CircuitSimulator {
public:
    CircuitSimulator(CircuitSimulationMode mode = CircuitSimulationMode::NativeMNA);
    ~CircuitSimulator();

    // Component management
    template<typename T, typename... Args>
    T* add_component(std::string id, Args&&... args) {
        auto comp = std::make_unique<T>(std::forward<Args>(args)...);
        comp->m_id = id;
        for (auto* pin : comp->get_pins()) {
            if (pin) {
                pin->component_id = id;
            }
        }
        T* ptr = comp.get();
        m_components_owned[id] = std::move(comp);
        return ptr;
    }

    // Add existing component pointer (does NOT take ownership)
    // The component must remain valid for the lifetime of the simulation
    // This allows using components from adapters directly without duplication
    CircuitComponent* add_component_external(std::string id, CircuitComponent* comp) {
        comp->m_id = id;
        // Update component_id for all pins to match the component ID
        for (auto* pin : comp->get_pins()) {
            pin->component_id = id;
        }
        m_components_external[id] = comp;
        return comp;
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
    // Owned components (created by simulator, will be deleted)
    std::unordered_map<std::string, std::unique_ptr<CircuitComponent>> m_components_owned;
    // External components (from adapters, NOT owned, will NOT be deleted)
    std::unordered_map<std::string, CircuitComponent*> m_components_external;
    std::unordered_map<std::string, std::unique_ptr<Wire>> m_wires;

    // Helper method
    void step_with_ngspice(double dt);

    // ngspice (kept alive across steps to avoid init/teardown churn)
    std::unique_ptr<NgspiceWrapper> m_ngspice;
    size_t m_last_netlist_hash = 0;
};

} // namespace mechatron
