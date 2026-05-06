#include "MCUMcuAvrPlugin.hpp"
#include "mcu/ATmegaInterpreter.hpp"
#include "mcu/QEMUInterface.hpp"
#include "core/Port.hpp"
#include <unordered_map>
#include <functional>
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
        // Create interpreter attached to QEMU interface
        m_interpreter = std::make_unique<ATmegaInterpreter>(*m_qemu);

        // Create digital pin ports (Arduino Uno: 14 digital pins)
        for (int i = 0; i < 14; ++i) {
            m_digital_ports.push_back(
                std::make_unique<Port>("D" + std::to_string(i),
                    PortDomain::Digital, PortDirection::Bidirectional));
        }

        // Create analog pin ports (6 analog pins)
        for (int i = 0; i < 6; ++i) {
            m_analog_ports.push_back(
                std::make_unique<Port>("A" + std::to_string(i),
                    PortDomain::Analog, PortDirection::Bidirectional));
        }
    }

    std::string_view plugin_type() const override { return "soft_mcu_avr"; }
    std::string_view component_type() const override { return m_mcu_type; }
    std::string_view category() const override { return "mcu"; }

    void update(double dt) override {
        if (!m_interpreter->is_loaded()) return;

        // Sync input pins to MCU
        sync_input_pins();

        // Execute instructions for the elapsed time
        // ATmega328P runs at 16 MHz → 16 instructions per microsecond (approx)
        uint32_t us = static_cast<uint32_t>(dt * 1e6);
        m_interpreter->execute_for_us(us);

        // Sync output pins from MCU
        sync_output_pins();
    }

    void serialize(nlohmann::json& out) const override {
        out["mcu_type"] = m_mcu_type;
        out["firmware_path"] = m_firmware_path;
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
    }

    std::vector<Port*> get_ports() override {
        std::vector<Port*> ports;
        for (auto& p : m_digital_ports) ports.push_back(p.get());
        for (auto& p : m_analog_ports) ports.push_back(p.get());
        return ports;
    }

    // Firmware loading
    bool load_firmware(const std::string& path) {
        bool ok = m_interpreter->load_firmware(path);
        if (ok) {
            m_firmware_path = path;
            spdlog::info("MCU firmware loaded: {}", path);
        } else {
            spdlog::error("Failed to load MCU firmware: {}", path);
        }
        return ok;
    }

    void reset() {
        m_interpreter->reset();
    }

    bool is_loaded() const { return m_interpreter->is_loaded(); }

    // Direct pin access
    void digital_write(uint8_t pin, bool value) {
        if (pin < 14) {
            m_qemu->digital_write(pin, value);
        }
    }

    bool digital_read(uint8_t pin) {
        if (pin < 14) {
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

private:
    void sync_input_pins() {
        // Read port values and write to MCU
        for (size_t i = 0; i < m_digital_ports.size(); ++i) {
            auto* val = m_digital_ports[i]->get_value<bool>();
            if (val) {
                m_qemu->digital_write(static_cast<uint8_t>(i), *val);
            }
        }
        for (size_t i = 0; i < m_analog_ports.size(); ++i) {
            auto* val = m_analog_ports[i]->get_value<float>();
            if (val) {
                m_qemu->analog_write(static_cast<uint8_t>(14 + i), *val);
            }
        }
    }

    void sync_output_pins() {
        // Read MCU pin states and update ports
        for (size_t i = 0; i < m_digital_ports.size(); ++i) {
            bool state = m_qemu->digital_read(static_cast<uint8_t>(i));
            m_digital_ports[i]->set_value(state);
        }
        for (size_t i = 0; i < m_analog_ports.size(); ++i) {
            float voltage = m_qemu->analog_read(static_cast<uint8_t>(i));
            m_analog_ports[i]->set_value(voltage);
        }
    }

    std::string m_mcu_type;
    std::string m_firmware_path;

    std::unique_ptr<QEMUInterface> m_qemu;
    std::unique_ptr<ATmegaInterpreter> m_interpreter;

    // Ports
    std::vector<std::unique_ptr<Port>> m_digital_ports;
    std::vector<std::unique_ptr<Port>> m_analog_ports;
};

// ============================================================================
// Plugin Implementation
// ============================================================================

std::vector<ComponentDescriptor> MCUMcuAvrPlugin::components() const {
    return {
        {"atmega328p", "ATmega328P", "mcu", "Arduino Uno microcontroller"},
        {"atmega2560", "ATmega2560", "mcu", "Arduino Mega microcontroller"},
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
