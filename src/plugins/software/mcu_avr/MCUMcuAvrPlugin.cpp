#include "MCUMcuAvrPlugin.hpp"
#include "mcu/ATmegaInterpreter.hpp"
#include "mcu/QEMUInterface.hpp"
#include <unordered_map>
#include <functional>

namespace mechatron {

std::vector<ComponentDescriptor> MCUMcuAvrPlugin::components() const {
    return {
        {"atmega328p", "ATmega328P", "mcu", "Arduino Uno microcontroller"},
        {"atmega2560", "ATmega2560", "mcu", "Arduino Mega microcontroller"},
        {"attiny85", "ATtiny85", "mcu", "Small AVR microcontroller"}
    };
}

std::unique_ptr<Component> MCUMcuAvrPlugin::create(std::string_view type) {
    // TODO: ATmegaInterpreter does not inherit from Component yet
    // Need to create a Component wrapper or make ATmegaInterpreter inherit from Component
    return nullptr;
}

void MCUMcuAvrPlugin::on_register(PluginHost& host) {
    // Initialize QEMU if available
}

void MCUMcuAvrPlugin::on_unregister() {
    // Cleanup QEMU resources
}

} // namespace mechatron
