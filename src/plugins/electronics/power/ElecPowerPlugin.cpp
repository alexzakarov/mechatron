#include "ElecPowerPlugin.hpp"
#include "electronics/CircuitSimulator.hpp"
#include "electronics/CircuitComponentAdapter.hpp"
#include <unordered_map>
#include <functional>

namespace mechatron {

std::vector<ComponentDescriptor> ElecPowerPlugin::components() const {
    return {
        {"dc_voltage", "DC Voltage", "power", "DC voltage source (power supply)"},
        {"h_bridge", "H-Bridge", "power", "H-Bridge motor driver"},
        {"buck_converter", "Buck Converter", "power", "Step-down DC-DC converter"},
        {"boost_converter", "Boost Converter", "power", "Step-up DC-DC converter"},
        {"motor_driver", "Motor Driver", "power", "General purpose motor driver"}
    };
}

std::unique_ptr<Component> ElecPowerPlugin::create(std::string_view type) {
    using namespace std;
    static const unordered_map<string, function<unique_ptr<Component>()>> factory = {
        {"dc_voltage", []() { return make_unique<DCVoltageComponent>(make_unique<DCVoltageSource>()); }},
        {"h_bridge", []() { return make_unique<HBridgeComponent>(make_unique<HBridge>()); }},
        {"buck_converter", []() { return make_unique<BuckConverterComponent>(make_unique<BuckConverter>()); }},
        {"boost_converter", []() { return make_unique<BoostConverterComponent>(make_unique<BoostConverter>()); }},
        {"motor_driver", []() { return make_unique<MotorDriverComponent>(make_unique<MotorDriver>()); }}
    };

    auto it = factory.find(string(type));
    if (it != factory.end()) {
        return it->second();
    }
    return nullptr;
}

void ElecPowerPlugin::on_register(PluginHost& host) {
}

void ElecPowerPlugin::on_unregister() {
}

} // namespace mechatron
