#include "MechMachineElementsPlugin.hpp"
#include "actuators/Actuator.hpp"
#include "sensors/Sensor.hpp"
#include <string>

namespace mechatron {

std::vector<ComponentDescriptor> MechMachineElementsPlugin::components() const {
    return {
        // Actuators
        {"solenoid_actuator", "Solenoid Actuator", "mechanical", "Electromechanical linear actuator"},
        {"dc_motor", "DC Motor", "mechanical", "Rotational DC motor with encoder support"},
        {"servo_motor", "Servo Motor", "mechanical", "Position-controlled servo motor"},

        // Sensors
        {"proximity_sensor", "Proximity Sensor", "mechanical", "Distance measuring sensor"},
        {"limit_switch", "Limit Switch", "mechanical", "Binary contact sensor"},
        {"rotary_encoder", "Rotary Encoder", "mechanical", "Angular position sensor"},
        {"potentiometer", "Potentiometer", "mechanical", "Angular position sensor (analog)"}
    };
}

std::unique_ptr<Component> MechMachineElementsPlugin::create(std::string_view type) {
    std::string t(type);

    // Actuators
    if (t == "solenoid_actuator") {
        return std::make_unique<SolenoidActuator>();
    }
    if (t == "dc_motor") {
        return std::make_unique<DCMotor>();
    }
    if (t == "servo_motor") {
        return std::make_unique<ServoMotor>();
    }

    // Sensors
    if (t == "proximity_sensor") {
        return std::make_unique<ProximitySensor>();
    }
    if (t == "limit_switch") {
        return std::make_unique<LimitSwitch>();
    }
    if (t == "rotary_encoder") {
        return std::make_unique<RotaryEncoder>();
    }
    if (t == "potentiometer") {
        return std::make_unique<Potentiometer>();
    }

    return nullptr;
}

void MechMachineElementsPlugin::on_register(PluginHost& host) {
    // Plugin initialization logic
}

void MechMachineElementsPlugin::on_unregister() {
    // Plugin cleanup
}

} // namespace mechatron
