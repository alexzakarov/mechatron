#pragma once

#include "core/Types.hpp"
#include "MNASolver.hpp"
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

    // MNA stamp: contribute to the matrix equation
    // node_map: maps CircuitPin* to node index (0 = ground)
    virtual void stamp(const std::unordered_map<CircuitPin*, int>& node_map,
                       MNASolver& solver, double dt) const { (void)node_map; (void)solver; (void)dt; }

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

    void update(double dt) override {
        // Ohm's Law: I = (V1 - V2) / R
        float v_diff = m_pin1.voltage - m_pin2.voltage;
        float current = v_diff / m_resistance;
        m_pin1.current = current;
        m_pin2.current = -current;
    }

    void stamp(const std::unordered_map<CircuitPin*, int>& node_map,
               MNASolver& solver, double dt) const override {
        (void)dt;
        int n1 = node_map.at(const_cast<CircuitPin*>(&m_pin1));
        int n2 = node_map.at(const_cast<CircuitPin*>(&m_pin2));
        double G = 1.0 / static_cast<double>(m_resistance);
        solver.add_conductance(n1, n2, G);
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
        // I = C * dV/dt -> for discrete time: I = C * (V1 - V2) / dt
        float v_diff = m_pin1.voltage - m_pin2.voltage;
        float dt_safe = (std::max)(static_cast<float>(dt), 0.0001f);
        float current = m_capacitance * v_diff / dt_safe;
        float max_current = 1.0f;
        current = (std::max)(-max_current, (std::min)(max_current, current));
        m_pin1.current = current;
        m_pin2.current = -current;
        m_charge = m_capacitance * v_diff;
    }

    void stamp(const std::unordered_map<CircuitPin*, int>& node_map,
               MNASolver& solver, double dt) const override {
        int n1 = node_map.at(const_cast<CircuitPin*>(&m_pin1));
        int n2 = node_map.at(const_cast<CircuitPin*>(&m_pin2));
        double C = static_cast<double>(m_capacitance);
        double dt_safe = (std::max)(dt, 1e-4);
        double G_eq = C / dt_safe;
        solver.add_conductance(n1, n2, G_eq);
        double v_prev = static_cast<double>(m_pin1.voltage - m_pin2.voltage);
        solver.add_current_source(n1, n2, G_eq * v_prev);
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
        (void)dt;
        float v_diff = m_anode.voltage - m_cathode.voltage;
        if (v_diff > m_forward_voltage) {
            float r_on = 0.1f;
            float current = (v_diff - m_forward_voltage) / r_on;
            float max_current = 0.5f;
            current = (std::max)(0.0f, (std::min)(max_current, current));
            m_anode.current = current;
            m_cathode.current = -current;
            m_is_lit = current > 1e-3f;
        } else {
            m_anode.current = 0.0f;
            m_cathode.current = 0.0f;
            m_is_lit = false;
        }
    }

    void stamp(const std::unordered_map<CircuitPin*, int>& node_map,
               MNASolver& solver, double dt) const override {
        (void)dt;
        int n_a = node_map.at(const_cast<CircuitPin*>(&m_anode));
        int n_c = node_map.at(const_cast<CircuitPin*>(&m_cathode));
        double v_diff = static_cast<double>(m_anode.voltage - m_cathode.voltage);
        if (v_diff > static_cast<double>(m_forward_voltage)) {
            double G = 1.0 / 0.1;  // R_on = 0.1 ohm
            solver.add_conductance(n_a, n_c, G);
            double I_offset = static_cast<double>(m_forward_voltage) / 0.1;
            solver.add_current_source(n_a, n_c, I_offset);
        } else {
            // Add minimum conductance for numerical stability (1MΩ leakage)
            solver.add_conductance(n_a, n_c, 1e-6);
        }
    }

private:
    float m_forward_voltage;
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

    std::vector<CircuitPin*> get_pins() override {
        return {&m_pin1, &m_pin2};
    }

    void update(double dt) override {
        // V = L * dI/dt -> dI = V * dt / L
        float v_diff = m_pin1.voltage - m_pin2.voltage;
        m_current += v_diff * static_cast<float>(dt) / m_inductance;
        m_pin1.current = m_current;
        m_pin2.current = -m_current;
    }

    void stamp(const std::unordered_map<CircuitPin*, int>& node_map,
               MNASolver& solver, double dt) const override {
        int n1 = node_map.at(const_cast<CircuitPin*>(&m_pin1));
        int n2 = node_map.at(const_cast<CircuitPin*>(&m_pin2));
        double L = static_cast<double>(m_inductance);
        double dt_safe = (std::max)(dt, 1e-6);
        double G_eq = dt_safe / L;
        solver.add_conductance(n1, n2, G_eq);
        solver.add_current_source(n1, n2, static_cast<double>(m_current));
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
    float m_current = 0.0f;
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

    std::vector<CircuitPin*> get_pins() override {
        return {&m_anode, &m_cathode};
    }

    void update(double dt) override {
        (void)dt;
        float v_diff = m_anode.voltage - m_cathode.voltage;
        if (v_diff > m_forward_voltage) {
            float current = (v_diff - m_forward_voltage) / m_on_resistance;
            float max_current = 1.0f;
            current = (std::max)(0.0f, (std::min)(max_current, current));
            m_anode.current = current;
            m_cathode.current = -current;
            m_conducting = true;
        } else {
            m_anode.current = 0.0f;
            m_cathode.current = 0.0f;
            m_conducting = false;
        }
    }

    void stamp(const std::unordered_map<CircuitPin*, int>& node_map,
               MNASolver& solver, double dt) const override {
        (void)dt;
        int n_a = node_map.at(const_cast<CircuitPin*>(&m_anode));
        int n_c = node_map.at(const_cast<CircuitPin*>(&m_cathode));
        double v_diff = static_cast<double>(m_anode.voltage - m_cathode.voltage);
        if (v_diff > static_cast<double>(m_forward_voltage)) {
            double G = 1.0 / static_cast<double>(m_on_resistance);
            solver.add_conductance(n_a, n_c, G);
            double I_offset = static_cast<double>(m_forward_voltage) / static_cast<double>(m_on_resistance);
            solver.add_current_source(n_a, n_c, I_offset);
        }
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

    std::vector<CircuitPin*> get_pins() override {
        return {&m_anode, &m_cathode};
    }

    void update(double dt) override {
        (void)dt;
        float v_diff = m_anode.voltage - m_cathode.voltage;
        float r_on = 0.1f;

        if (v_diff > m_forward_voltage) {
            float current = (v_diff - m_forward_voltage) / r_on;
            float max_current = 1.0f;
            current = (std::max)(0.0f, (std::min)(max_current, current));
            m_anode.current = current;
            m_cathode.current = -current;
            m_conducting = true;
        } else if (v_diff < -m_zener_voltage) {
            float excess_voltage = std::abs(v_diff) - m_zener_voltage;
            float current = excess_voltage / r_on;
            float max_current = 1.0f;
            current = (std::max)(0.0f, (std::min)(max_current, current));
            m_anode.current = -current;
            m_cathode.current = current;
            m_conducting = true;
        } else {
            m_anode.current = 0.0f;
            m_cathode.current = 0.0f;
            m_conducting = false;
        }
    }

    void stamp(const std::unordered_map<CircuitPin*, int>& node_map,
               MNASolver& solver, double dt) const override {
        (void)dt;
        int n_a = node_map.at(const_cast<CircuitPin*>(&m_anode));
        int n_c = node_map.at(const_cast<CircuitPin*>(&m_cathode));
        double v_diff = static_cast<double>(m_anode.voltage - m_cathode.voltage);
        double G = 1.0 / 0.1;
        if (v_diff > static_cast<double>(m_forward_voltage)) {
            solver.add_conductance(n_a, n_c, G);
            solver.add_current_source(n_a, n_c, static_cast<double>(m_forward_voltage) / 0.1);
        } else if (v_diff < -static_cast<double>(m_zener_voltage)) {
            solver.add_conductance(n_a, n_c, G);
            solver.add_current_source(n_c, n_a, static_cast<double>(m_zener_voltage) / 0.1);
        }
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
    bool m_conducting = false;
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

    std::vector<CircuitPin*> get_pins() override {
        return {&m_base, &m_collector, &m_emitter};
    }

    void update(double dt) override {
        float vbe = m_base.voltage - m_emitter.voltage;
        float vce = m_collector.voltage - m_emitter.voltage;

        if (m_bjt_type == NPN) {
            // NPN: active when Vbe > 0.7V
            if (vbe > 0.7f) {
                // Base current: Ib = (Vbe - 0.7) / Rb (assume 1k base resistor)
                float r_b = 1000.0f;
                float ib = (vbe - 0.7f) / r_b;
                // Collector current: Ic = beta * Ib
                float ic = m_beta * ib;
                // Saturation: Vce < 0.2V
                if (vce < 0.2f) ic = vce / 0.1f;
                // Set currents: Ic flows into collector, Ie = Ic + Ib flows out of emitter
                m_base.current = ib;
                m_collector.current = -ic;
                m_emitter.current = ic + ib;
            } else {
                // Cutoff: all currents zero
                m_base.current = 0.0f;
                m_collector.current = 0.0f;
                m_emitter.current = 0.0f;
            }
        } else {
            // PNP: active when Veb > 0.7V
            float veb = m_emitter.voltage - m_base.voltage;
            if (veb > 0.7f) {
                float r_b = 1000.0f;
                float ib = (veb - 0.7f) / r_b;
                float ic = m_beta * ib;
                if (-vce < 0.2f) ic = -vce / 0.1f;
                m_base.current = -ib;
                m_collector.current = ic;
                m_emitter.current = -(ic + ib);
            } else {
                m_base.current = 0.0f;
                m_collector.current = 0.0f;
                m_emitter.current = 0.0f;
            }
        }
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

    std::vector<CircuitPin*> get_pins() override {
        return {&m_gate, &m_drain, &m_source};
    }

    void update(double dt) override {
        (void)dt;
        double id = compute_drain_current();
        if (m_mos_type == NChannel) {
            m_drain.current = static_cast<float>(-id);
            m_source.current = static_cast<float>(id);
        } else {
            m_drain.current = static_cast<float>(id);
            m_source.current = static_cast<float>(-id);
        }
    }

    // Newton-Raphson linearized stamp for MNA.
    // At the operating point, Id(Vd,Vs) is linearized as:
    //   Id ≈ G_ds * (Vd - Vs) + I_offset
    // where G_ds = dId/d(Vds) and I_offset = Id - G_ds * Vds
    void stamp(const std::unordered_map<CircuitPin*, int>& node_map,
               MNASolver& solver, double dt) const override {
        (void)dt;
        int n_g = node_map.at(const_cast<CircuitPin*>(&m_gate));
        int n_d = node_map.at(const_cast<CircuitPin*>(&m_drain));
        int n_s = node_map.at(const_cast<CircuitPin*>(&m_source));

        // Use MNA-solved voltages if available, otherwise pin voltages
        double v_gate, v_drain, v_source;
        if (solver.is_solved()) {
            v_gate = solver.get_node_voltage(n_g);
            v_drain = solver.get_node_voltage(n_d);
            v_source = solver.get_node_voltage(n_s);
        } else {
            v_gate = static_cast<double>(m_gate.voltage);
            v_drain = static_cast<double>(m_drain.voltage);
            v_source = static_cast<double>(m_source.voltage);
        }

        double vgs, vds;
        if (m_mos_type == NChannel) {
            vgs = v_gate - v_source;
            vds = v_drain - v_source;
        } else {
            vgs = v_source - v_gate;
            vds = v_source - v_drain;
        }

        double vth = static_cast<double>(m_vth);
        double kp = static_cast<double>(m_kp);
        constexpr double G_MIN = 1e-9;
        constexpr double I_MAX = 200.0;
        constexpr double RDS_ON = 0.01;  // 10mΩ when fully ON

        double G_ds, I_offset;

        if (vgs <= vth) {
            // Cutoff: MOSFET OFF
            G_ds = G_MIN;
            I_offset = 0.0;
        } else {
            // MOSFET ON - use simple model for better convergence
            // For first iteration or when Vds is high, use Rds_on approximation
            // This gives a better starting point for Newton-Raphson
            if (!solver.is_solved() && vds > 1.0) {
                // First iteration with high Vds: assume MOSFET will be in linear region
                G_ds = 1.0 / RDS_ON;  // 100S
                I_offset = 0.0;
            } else {
                // Subsequent iterations or low Vds: use full Shichman-Hodges model
                double vov = vgs - vth;
                if (vds < vov) {
                    // Triode/linear region
                    G_ds = kp * vov;
                    I_offset = 0.0;
                } else {
                    // Saturation: cap Id FIRST, then linearize
                    double Id = (std::min)(0.5 * kp * vov * vov, I_MAX);
                    double lambda = 0.02;
                    G_ds = Id * lambda;
                    I_offset = Id - G_ds * vds;
                }
            }
        }

        G_ds = (std::max)(G_ds, G_MIN);

        // Gate leakage: tiny conductance to ground reference
        // This ensures gate node participates in MNA matrix
        // In DC analysis, gate needs a DC path - real MOSFETs have gate leakage (~1nA)
        // Ground is always node 0 in our MNA formulation
        constexpr double G_GATE_LEAK = 1e-9;  // 1GΩ leakage to ground
        solver.add_conductance(n_g, 0, G_GATE_LEAK);

        solver.add_conductance(n_d, n_s, G_ds);

        if (std::abs(I_offset) > 1e-15) {
            // For N-channel: Id flows INTO drain, OUT OF source
            // Current source stamps: +I at drain, -I at source
            if (m_mos_type == NChannel) {
                solver.add_current_source(n_d, n_s, I_offset);
            } else {
                // For P-channel: Id flows INTO source, OUT OF drain
                solver.add_current_source(n_s, n_d, I_offset);
            }
        }
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
    double compute_drain_current() const {
        double vgs, vds;
        if (m_mos_type == NChannel) {
            vgs = static_cast<double>(m_gate.voltage - m_source.voltage);
            vds = static_cast<double>(m_drain.voltage - m_source.voltage);
        } else {
            vgs = static_cast<double>(m_source.voltage - m_gate.voltage);
            vds = static_cast<double>(m_source.voltage - m_drain.voltage);
        }
        double vth = static_cast<double>(m_vth);
        double kp = static_cast<double>(m_kp);

        if (vgs <= vth) return 0.0;
        double vov = vgs - vth;
        if (vds < vov) {
            return kp * vov * vds;
        } else {
            double lambda = 0.02;
            return 0.5 * kp * vov * vov * (1.0 + lambda * vds);
        }
    }

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

    std::vector<CircuitPin*> get_pins() override {
        return {&m_vin, &m_gnd, &m_vout};
    }

    void update(double dt) override {
        // Use potential difference: VIN - GND
        float vin_actual = m_vin.voltage - m_gnd.voltage;
        // Vout = Vin * duty_cycle (ideal)
        m_output_voltage = vin_actual * m_duty_cycle;
        m_vout.voltage = m_output_voltage;
        m_vout.is_driven = true;
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

    std::vector<CircuitPin*> get_pins() override {
        return {&m_vin, &m_gnd, &m_vout};
    }

    void update(double dt) override {
        // Use potential difference: VIN - GND
        float vin_actual = m_vin.voltage - m_gnd.voltage;
        // Vout = Vin / (1 - duty_cycle) (ideal)
        float effective_d = (std::min)(m_duty_cycle, 0.95f);
        m_output_voltage = vin_actual / (1.0f - effective_d);
        m_vout.voltage = m_output_voltage;
        m_vout.is_driven = true;
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

private:
    float m_input_voltage;
    float m_duty_cycle;
    float m_output_voltage = 0.0f;
    CircuitPin m_vin{"VIN", "", PinDirection::Input, PinType::Power};
    CircuitPin m_gnd{"GND", "", PinDirection::Input, PinType::Ground};
    CircuitPin m_vout{"VOUT", "", PinDirection::Output, PinType::Analog};
};

// Motor Driver (simplified: PWM input → voltage output)
class MotorDriver : public CircuitComponent {
public:
    std::string_view type() const override { return "motor_driver"; }
    std::string_view category() const override { return "power"; }

    MotorDriver(float supply_voltage = 12.0f) : m_supply_voltage(supply_voltage) {}

    std::vector<CircuitPin*> get_pins() override {
        return {&m_pwm, &m_dir, &m_en, &m_out_pos, &m_out_neg, &m_vcc, &m_gnd};
    }

    void update(double dt) override {
        if (!m_en.digital_state) {
            m_out_pos.voltage = 0.0f;
            m_out_neg.voltage = 0.0f;
            m_output_voltage = 0.0f;
            return;
        }
        // Use VCC - GND as actual supply voltage
        float supply = m_vcc.voltage - m_gnd.voltage;
        if (supply <= 0.0f) supply = m_supply_voltage;  // Fallback to internal value

        float duty = m_pwm.voltage / supply;
        duty = (std::max)(0.0f, (std::min)(1.0f, duty));
        float vout = supply * duty;
        if (m_dir.digital_state) {
            m_out_pos.voltage = vout;
            m_out_neg.voltage = 0.0f;
        } else {
            m_out_pos.voltage = 0.0f;
            m_out_neg.voltage = vout;
        }
        m_output_voltage = m_dir.digital_state ? vout : -vout;
    }

    void set_parameter(const std::string& name, double value) override {
        if (name == "supply_voltage") m_supply_voltage = static_cast<float>(value);
    }

    double get_parameter(const std::string& name) const override {
        if (name == "supply_voltage") return m_supply_voltage;
        if (name == "output_voltage") return m_output_voltage;
        return 0.0;
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
    std::string_view type() const override { return "dc_voltage"; }
    std::string_view category() const override { return "power"; }

    DCVoltageSource(float voltage = 5.0f) : m_voltage(voltage) {}

    std::vector<CircuitPin*> get_pins() override {
        return {&m_vout, &m_gnd};
    }

    void update(double dt) override {
        (void)dt;
        m_vout.voltage = m_voltage;
        m_vout.is_driven = true;
        m_gnd.voltage = 0.0f;
        m_gnd.is_driven = true;
    }

    void stamp(const std::unordered_map<CircuitPin*, int>& node_map,
               MNASolver& solver, double dt) const override {
        (void)dt;
        int n_pos = node_map.at(const_cast<CircuitPin*>(&m_vout));
        int n_neg = node_map.at(const_cast<CircuitPin*>(&m_gnd));
        solver.add_voltage_source(n_pos, n_neg, static_cast<double>(m_voltage));
    }

    void set_parameter(const std::string& name, double value) override {
        if (name == "voltage") m_voltage = static_cast<float>(value);
    }

    double get_parameter(const std::string& name) const override {
        if (name == "voltage") return m_voltage;
        return 0.0;
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

    std::vector<CircuitPin*> get_pins() override {
        return {&m_gnd};
    }

    void update(double dt) override {
        m_gnd.voltage = 0.0f;
        m_gnd.is_driven = true;
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
        m_components_owned[id] = std::move(comp);
        return ptr;
    }

    // Add existing component pointer (does NOT take ownership)
    // The component must remain valid for the lifetime of the simulation
    // This allows using components from adapters directly without duplication
    CircuitComponent* add_component_external(std::string id, CircuitComponent* comp) {
        comp->m_id = id;
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

    // MNA node identification
    std::unordered_map<CircuitPin*, int> m_pin_to_node;
    int m_num_nodes = 0;
    int m_num_voltage_sources = 0;

    void build_nets();
};

} // namespace mechatron
