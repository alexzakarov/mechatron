#include "MCUMcuAvrPlugin.hpp"
#include "mcu/ATmegaInterpreter.hpp"
#include "mcu/QEMUInterface.hpp"
#include "mcu/PhysicalMCUBridge.hpp"
#include "core/Port.hpp"
#include <unordered_map>
#include <functional>
#include <algorithm>
#include <spdlog/spdlog.h>

namespace mechatron {

// ============================================================================
// MCU Component - wraps ATmegaInterpreter as a Component
// ============================================================================

class MCUComponent : public Component {
public:
    explicit MCUComponent(std::string mcu_type)
        : m_mcu_type(std::move(mcu_type))
    {
        // Create QEMU interface (simulation mode)
        m_qemu = std::make_unique<QEMUInterface>();
        if (m_mcu_type == "atmega328p") {
            m_qemu->set_mcu_variant_atmega328p();
            m_qemu->set_clock_frequency(16000000);
        } else if (m_mcu_type == "attiny85") {
            m_qemu->set_mcu_variant_attiny85();
            m_qemu->set_clock_frequency(m_qemu->mcu_variant().default_clock_hz);
        } else if (m_mcu_type == "atmega2560") {
            m_qemu->set_mcu_variant_atmega2560();
            m_qemu->set_clock_frequency(16000000);
        }
        // Create interpreter attached to QEMU interface
        m_interpreter = std::make_unique<ATmegaInterpreter>(*m_qemu);

        // Create digital pin ports (Arduino Uno: 14 digital pins)
        int digital_count = 14;
        int analog_count = 6;
        if (m_mcu_type == "attiny85") {
            digital_count = 6;
            analog_count = 4;
        } else if (m_mcu_type == "atmega2560") {
            // Arduino Mega: D0..D53, A0..A15
            digital_count = 54;
            analog_count = 16;
        }
        for (int i = 0; i < digital_count; ++i) {
            auto port = std::make_unique<Port>("D" + std::to_string(i),
                PortDomain::Digital, PortDirection::Bidirectional);
            assign_port_owner(port.get());
            m_digital_ports.push_back(std::move(port));
        }

        // Create analog pin ports (6 analog pins)
        for (int i = 0; i < analog_count; ++i) {
            auto port = std::make_unique<Port>("A" + std::to_string(i),
                PortDomain::Analog, PortDirection::Bidirectional);
            assign_port_owner(port.get());
            m_analog_ports.push_back(std::move(port));
        }

        m_vcc_port = std::make_unique<Port>("VCC", PortDomain::Electrical, PortDirection::Input);
        m_gnd_port = std::make_unique<Port>("GND", PortDomain::Electrical, PortDirection::Input);
        m_aref_port = std::make_unique<Port>("AREF", PortDomain::Electrical, PortDirection::Input);
        m_reset_port = std::make_unique<Port>("RESET", PortDomain::Digital, PortDirection::Input);
        assign_port_owner(m_vcc_port.get());
        assign_port_owner(m_gnd_port.get());
        assign_port_owner(m_aref_port.get());
        assign_port_owner(m_reset_port.get());
        // Default to "unpowered" unless the circuit provides rails.
        // These are inputs, so leaving them at 0 avoids phantom 5V when nothing is wired.
        m_vcc_port->set_value(0.0f);
        m_gnd_port->set_value(0.0f);
        m_aref_port->set_value(0.0f);
        m_reset_port->set_value(true); // pull-up semantics; real reset is handled via power/reset logic below
    }

    std::string_view plugin_type() const override { return "soft_mcu_avr"; }
    std::string_view component_type() const override { return m_mcu_type; }
    std::string_view category() const override { return "mcu"; }

    void update(double dt) override {
        if (!m_interpreter->is_loaded()) return;

        // Power gating: if VCC/GND are not actually wired (or VCC==GND), the MCU must not "run"
        // and must not source any voltage into the circuit.
        const bool powered = is_powered();
        if (!powered) {
            if (m_was_powered) {
                // Falling edge: simulate a power loss reset and clear outputs.
                m_interpreter->reset();
                m_qemu->memory().reset();
                m_pwm_time_s = 0.0;
                clear_output_ports_to_gnd();
                if (m_physical_enabled && m_physical.is_connected()) {
                    m_physical.set_digital_outputs(0, 0);
                    std::array<uint8_t, 20> pwm{};
                    pwm.fill(0);
                    m_physical.set_pwm_bulk(0, pwm);
                }
            } else {
                clear_output_ports_to_gnd();
            }
            m_was_powered = false;
            return;
        }
        if (!m_was_powered) {
            // Rising edge: start from a clean reset when power is first applied.
            m_interpreter->reset();
            m_qemu->memory().reset();
            m_pwm_time_s = 0.0;
        }
        m_was_powered = true;

        // Sync input pins to MCU
        sync_input_pins();

        // Pull physical inputs (if paired) after circuit sync so the real device can drive inputs.
        if (m_physical_enabled && m_physical.is_connected()) {
            m_physical.poll();
            auto inputs = m_physical.latest_inputs();
            if (inputs) {
                // Digital inputs: D0..D13 + A0..A5 as 14..19
                const uint8_t max_digital = static_cast<uint8_t>(
                    std::min<size_t>(32, m_digital_ports.size() + m_analog_ports.size()));
                for (uint8_t pin = 0; pin < max_digital; ++pin) {
                    bool bit = (inputs->digital_bits & (1u << pin)) != 0;
                    if (!m_qemu->is_digital_output(pin)) {
                        m_qemu->set_digital_input(pin, bit);
                        if (pin < m_digital_ports.size()) {
                            m_digital_ports[pin]->set_value(bit);
                        } else if (pin - m_digital_ports.size() < m_analog_ports.size()) {
                            m_analog_ports[pin - m_digital_ports.size()]->set_value(bit ? supply_voltage() : 0.0f);
                        }
                    }
                }
                // Analog inputs A0..A5
                for (uint8_t a = 0; a < m_analog_ports.size(); ++a) {
                    float v = (static_cast<float>(inputs->analog[a]) / 1023.0f) * supply_voltage();
                    m_qemu->set_analog_input(a, v);
                    m_analog_ports[a]->set_value(v);
                }
            }
        }

        // Execute instructions for the elapsed time
        // ATmega328P runs at 16 MHz → 16 instructions per microsecond (approx)
        uint32_t us = static_cast<uint32_t>(dt * 1e6);
        m_interpreter->execute_for_us(us);
        m_pwm_time_s += dt;

        // Sync output pins from MCU
        sync_output_pins();

        // Push outputs to physical MCU (if paired)
        if (m_physical_enabled && m_physical.is_connected()) {
            uint32_t digital_output_mask = 0;
            uint32_t digital_out_bits = 0;
            uint32_t pwm_output_mask = 0;
            std::array<uint8_t, 20> pwm{};
            pwm.fill(0);

            // D pins
            for (uint8_t pin = 0; pin < m_digital_ports.size() && pin < 20; ++pin) {
                if (m_qemu->is_digital_output(pin)) {
                    digital_output_mask |= (1u << pin);
                    bool hi = m_qemu->digital_read(pin);
                    if (hi) digital_out_bits |= (1u << pin);
                }
                if (m_qemu->is_digital_output(pin) && is_pwm_pin(pin)) {
                    pwm_output_mask |= (1u << pin);
                    float v = m_qemu->digital_output_voltage(pin);
                    uint8_t duty = static_cast<uint8_t>(std::clamp(v / 5.0f, 0.0f, 1.0f) * 255.0f);
                    pwm[pin] = duty;
                }
            }
            // A pins as digital 14..19
            for (uint8_t a = 0; a < m_analog_ports.size(); ++a) {
                uint8_t pin = static_cast<uint8_t>(m_digital_ports.size() + a);
                if (m_qemu->is_digital_output(pin)) {
                    digital_output_mask |= (1u << pin);
                    bool hi = m_qemu->digital_read(pin);
                    if (hi) digital_out_bits |= (1u << pin);
                }
            }

            m_physical.set_digital_outputs(digital_output_mask, digital_out_bits);
            m_physical.set_pwm_bulk(pwm_output_mask, pwm);
            m_physical.poll();
        }
    }

    void serialize(nlohmann::json& out) const override {
        out["mcu_type"] = m_mcu_type;
        out["firmware_path"] = m_firmware_path;
        out["physical_link_enabled"] = m_physical_enabled;
        out["physical_port"] = m_physical_port;
        out["physical_baud"] = m_physical_baud;
        if (m_interpreter->is_loaded()) {
            auto& state = m_interpreter->state();
            out["pc"] = state.PC;
            out["cycles"] = state.cycles;
            out["instruction_count"] = m_interpreter->instruction_count();
        }
    }

    void deserialize(const nlohmann::json& in) override {
        if (in.contains("firmware_path")) {
            std::string path = in["firmware_path"];
            if (!path.empty()) {
                load_firmware(path);
            }
        }
        if (in.contains("physical_link_enabled")) m_physical_enabled = in["physical_link_enabled"].get<bool>();
        if (in.contains("physical_port")) m_physical_port = in["physical_port"].get<std::string>();
        if (in.contains("physical_baud")) m_physical_baud = in["physical_baud"].get<int>();
    }

    std::vector<Port*> get_ports() override {
        std::vector<Port*> ports;
        ports.push_back(m_vcc_port.get());
        ports.push_back(m_gnd_port.get());
        ports.push_back(m_aref_port.get());
        ports.push_back(m_reset_port.get());
        for (auto& p : m_digital_ports) {
            assign_port_owner(p.get());
            ports.push_back(p.get());
        }
        for (auto& p : m_analog_ports) {
            assign_port_owner(p.get());
            ports.push_back(p.get());
        }
        return ports;
    }

    // Firmware loading
    bool load_firmware(const std::string& path) {
        if (!m_qemu->mcu_variant().supported_by_interpreter) {
            spdlog::error("MCU type {} is not supported by the current interpreter core", m_mcu_type);
            return false;
        }
        // Ensure we don't inherit latched pin states across uploads.
        m_qemu->stop();
        bool ok = m_qemu->launch(path) && m_interpreter->load_firmware(path);
        if (ok) {
            m_firmware_path = path;
            spdlog::info("MCU firmware loaded: {}", path);
            // Clear any UI/circuit-facing outputs immediately after reset so stale highs don't linger.
            clear_output_ports_to_gnd();
            // Also clear physical outputs cache (if paired).
            if (m_physical_enabled && m_physical.is_connected()) {
                m_physical.set_digital_outputs(0, 0);
                std::array<uint8_t, 20> pwm{};
                pwm.fill(0);
                m_physical.set_pwm_bulk(0, pwm);
            }
        } else {
            spdlog::error("Failed to load MCU firmware: {}", path);
        }
        return ok;
    }

    bool load_firmware_file(const std::string& path) override {
        return load_firmware(path);
    }

    bool physical_link_supported() const override { return true; }
    void physical_link_get_config(std::string& port, int& baud) const override {
        port = m_physical_port;
        baud = m_physical_baud;
    }
    void physical_link_set_config(const std::string& port, int baud) override {
        m_physical_port = port;
        m_physical_baud = baud;
    }
    bool physical_link_connect() override {
        if (m_physical_port.empty()) return false;
        const bool ok = m_physical.connect(m_physical_port, m_physical_baud);
        m_physical_enabled = ok;
        return ok;
    }
    void physical_link_disconnect() override {
        m_physical_enabled = false;
        m_physical.disconnect();
    }
    bool physical_link_is_connected() const override {
        return m_physical.is_connected();
    }

    void reset() {
        m_interpreter->reset();
    }

    bool is_loaded() const { return m_interpreter->is_loaded(); }

    // Direct pin access
    void digital_write(uint8_t pin, bool value) {
        if (pin < m_digital_ports.size()) {
            m_qemu->digital_write(pin, value);
        }
    }

    bool digital_read(uint8_t pin) {
        if (pin < m_digital_ports.size()) {
            return m_qemu->digital_read(pin);
        }
        return false;
    }

    float analog_read(uint8_t pin) {
        return m_qemu->analog_read(pin);
    }

    void analog_write(uint8_t pin, float voltage) {
        m_qemu->analog_write(pin, voltage);
    }

    bool get_mcu_pin_output_voltage(std::string_view pin_name, float& voltage) const override {
        uint8_t arduino_pin = 0;
        if (!pin_name_to_arduino_pin(pin_name, arduino_pin)) {
            return false;
        }
        if (!m_qemu->is_digital_output(arduino_pin)) {
            return false;
        }

        float high = supply_voltage();
        float sampled = 0.0f;
        if (m_qemu->pwm_output_voltage(arduino_pin, m_pwm_time_s, high, 0.0f, sampled)) {
            voltage = sampled;
        } else {
            voltage = (m_qemu->digital_output_voltage(arduino_pin) / 5.0f) * high;
        }
        return true;
    }

    bool set_mcu_pin_input_voltage(std::string_view pin_name, float voltage) override {
        uint8_t arduino_pin = 0;
        if (!pin_name_to_arduino_pin(pin_name, arduino_pin)) {
            return false;
        }

        const float* gnd = m_gnd_port ? m_gnd_port->get_value<float>() : nullptr;
        float relative_voltage = voltage - (gnd ? *gnd : 0.0f);
        relative_voltage = std::clamp(relative_voltage, 0.0f, supply_voltage());

        if (!m_qemu->is_digital_output(arduino_pin)) {
            m_qemu->set_digital_input(arduino_pin, relative_voltage >= digital_threshold());
        }
        if (pin_name.size() >= 2 && pin_name[0] == 'A') {
            int analog_pin = std::stoi(std::string(pin_name.substr(1)));
            m_qemu->set_analog_input(static_cast<uint8_t>(analog_pin), relative_voltage);
        }
        return true;
    }

private:
    float supply_voltage() const {
        const float* vcc = m_vcc_port ? m_vcc_port->get_value<float>() : nullptr;
        const float* gnd = m_gnd_port ? m_gnd_port->get_value<float>() : nullptr;

        const bool vcc_wired = m_vcc_port && !m_vcc_port->connections().empty();
        const bool gnd_wired = m_gnd_port && !m_gnd_port->connections().empty();

        // If rails are not wired, treat as unpowered.
        if (!vcc_wired || !gnd_wired || !vcc || !gnd) {
            return 0.0f;
        }

        return std::clamp(*vcc - *gnd, 0.0f, 5.5f);
    }

    float digital_threshold() const {
        return supply_voltage() * 0.5f;
    }

    bool is_powered() const {
        // Consider "powered" only when rails are wired and VCC-GND is a sane voltage.
        return supply_voltage() > 0.1f;
    }

    void clear_output_ports_to_gnd() {
        const float* gnd = m_gnd_port ? m_gnd_port->get_value<float>() : nullptr;
        float gnd_voltage = gnd ? *gnd : 0.0f;
        for (auto& p : m_digital_ports) {
            // Digital ports are Bidirectional; store as float to represent an electrical level in the UI.
            p->set_value(gnd_voltage);
        }
        for (auto& p : m_analog_ports) {
            p->set_value(gnd_voltage);
        }
    }

    bool pin_name_to_arduino_pin(std::string_view name, uint8_t& arduino_pin) const {
        if (name.size() < 2) return false;

        try {
            int index = std::stoi(std::string(name.substr(1)));
            if (name[0] == 'D' && index >= 0 && index < static_cast<int>(m_digital_ports.size())) {
                arduino_pin = static_cast<uint8_t>(index);
                return true;
            }
            if (name[0] == 'A' && index >= 0 && index < static_cast<int>(m_analog_ports.size())) {
                arduino_pin = static_cast<uint8_t>(m_digital_ports.size() + index);
                return true;
            }
        } catch (...) {
            return false;
        }
        return false;
    }

    bool is_pwm_pin(uint8_t arduino_pin) const {
        if (m_mcu_type == "atmega328p") {
            return ArduinoUno::is_pwm_pin(arduino_pin);
        }
        if (m_mcu_type == "attiny85") {
            return arduino_pin == 0 || arduino_pin == 1;
        }
        if (m_mcu_type == "atmega2560") {
            switch (arduino_pin) {
                case 2: case 3: case 4: case 5: case 6: case 7: case 8:
                case 9: case 10: case 11: case 12: case 13:
                case 44: case 45: case 46:
                    return true;
                default:
                    return false;
            }
        }
        return false;
    }

    void sync_input_pins() {
        const float* gnd = m_gnd_port ? m_gnd_port->get_value<float>() : nullptr;
        float gnd_voltage = gnd ? *gnd : 0.0f;

        // Read port values and write to MCU
        for (size_t i = 0; i < m_digital_ports.size(); ++i) {
            if (m_qemu->is_digital_output(static_cast<uint8_t>(i))) continue;
            // Digital pins may be represented as bool (pure logic) or float (voltage level).
            if (auto* b = m_digital_ports[i]->get_value<bool>()) {
                m_qemu->set_digital_input(static_cast<uint8_t>(i), *b);
            } else if (auto* v = m_digital_ports[i]->get_value<float>()) {
                float rel = std::clamp(*v - gnd_voltage, 0.0f, supply_voltage());
                m_qemu->set_digital_input(static_cast<uint8_t>(i), rel >= digital_threshold());
            } else if (auto* d = m_digital_ports[i]->get_value<double>()) {
                float rel = std::clamp(static_cast<float>(*d) - gnd_voltage, 0.0f, supply_voltage());
                m_qemu->set_digital_input(static_cast<uint8_t>(i), rel >= digital_threshold());
            }
        }
        for (size_t i = 0; i < m_analog_ports.size(); ++i) {
            auto* val = m_analog_ports[i]->get_value<float>();
            if (val) {
                float relative_voltage = std::clamp(*val - gnd_voltage, 0.0f, supply_voltage());
                m_qemu->set_analog_input(static_cast<uint8_t>(i), relative_voltage);
                m_qemu->set_digital_input(static_cast<uint8_t>(m_digital_ports.size() + i), relative_voltage >= digital_threshold());
            }
        }
    }

    void sync_output_pins() {
        const float* gnd = m_gnd_port ? m_gnd_port->get_value<float>() : nullptr;
        float gnd_voltage = gnd ? *gnd : 0.0f;
        float high = supply_voltage();

        // Read MCU pin states and update ports
        for (size_t i = 0; i < m_digital_ports.size(); ++i) {
            uint8_t arduino_pin = static_cast<uint8_t>(i);
            if (m_qemu->is_digital_output(arduino_pin)) {
                if (is_pwm_pin(arduino_pin)) {
                    float voltage = 0.0f;
                    if (!m_qemu->pwm_output_voltage(arduino_pin, m_pwm_time_s, high, 0.0f, voltage)) {
                        voltage = (m_qemu->digital_output_voltage(arduino_pin) / 5.0f) * high;
                    }
                    m_digital_ports[i]->set_value(gnd_voltage + voltage);
                } else {
                    bool state = m_qemu->digital_read(arduino_pin);
                    // Represent as electrical level so nets/scope/ngspice can consume it.
                    m_digital_ports[i]->set_value(gnd_voltage + (state ? high : 0.0f));
                }
            }
        }
        for (size_t i = 0; i < m_analog_ports.size(); ++i) {
            uint8_t arduino_pin = static_cast<uint8_t>(m_digital_ports.size() + i);
            if (m_qemu->is_digital_output(arduino_pin)) {
                float voltage = 0.0f;
                if (!m_qemu->pwm_output_voltage(arduino_pin, m_pwm_time_s, high, 0.0f, voltage)) {
                    voltage = (m_qemu->digital_output_voltage(arduino_pin) / 5.0f) * high;
                }
                m_analog_ports[i]->set_value(gnd_voltage + voltage);
            }
        }
    }

    std::string m_mcu_type;
    std::string m_firmware_path;

    std::unique_ptr<QEMUInterface> m_qemu;
    std::unique_ptr<ATmegaInterpreter> m_interpreter;

    // Physical device pairing (optional)
    bool m_physical_enabled = false;
    std::string m_physical_port;
    int m_physical_baud = 115200;
    PhysicalMCUBridge m_physical;

    // Ports
    std::unique_ptr<Port> m_vcc_port;
    std::unique_ptr<Port> m_gnd_port;
    std::unique_ptr<Port> m_aref_port;
    std::unique_ptr<Port> m_reset_port;
    std::vector<std::unique_ptr<Port>> m_digital_ports;
    std::vector<std::unique_ptr<Port>> m_analog_ports;

    bool m_was_powered = false;
    double m_pwm_time_s = 0.0;
};

// ============================================================================
// Plugin Implementation
// ============================================================================

std::vector<ComponentDescriptor> MCUMcuAvrPlugin::components() const {
    return {
        {"atmega328p", "ATmega328P", "mcu", "Arduino Uno microcontroller"},
        {"atmega2560", "ATmega2560", "mcu", "Arduino Mega microcontroller (core WIP)"},
        {"attiny85", "ATtiny85", "mcu", "Small AVR microcontroller"}
    };
}

std::unique_ptr<Component> MCUMcuAvrPlugin::create(std::string_view type) {
    using namespace std;
    static const unordered_map<string, function<unique_ptr<Component>()>> factory = {
        {"atmega328p", []() { return make_unique<MCUComponent>("atmega328p"); }},
        {"atmega2560", []() { return make_unique<MCUComponent>("atmega2560"); }},
        {"attiny85", []() { return make_unique<MCUComponent>("attiny85"); }}
    };

    auto it = factory.find(string(type));
    if (it != factory.end()) {
        return it->second();
    }
    return nullptr;
}

void MCUMcuAvrPlugin::on_register(PluginHost& host) {
}

void MCUMcuAvrPlugin::on_unregister() {
}

} // namespace mechatron
