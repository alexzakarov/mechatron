#include "MultiMagneticPlugin.hpp"
#include "actuators/Actuator.hpp"  // For SolenoidActuator
#include <unordered_map>
#include <functional>

namespace mechatron {

std::vector<ComponentDescriptor> MultiMagneticPlugin::components() const {
    return {
        {"solenoid_field", "Solenoid Field", "magnetic", "Electromagnetic field from solenoid"},
        {"permanent_magnet", "Permanent Magnet", "magnetic", "Static magnetic field source"},
        {"magnetic_circuit", "Magnetic Circuit", "magnetic", "Magnetic flux path"},
        {"eddy_current", "Eddy Current", "magnetic", "Induced current in conductors"}
    };
}

std::unique_ptr<Component> MultiMagneticPlugin::create(std::string_view type) {
    // SolenoidActuator already exists but is in mech_machine_elements
    // Magnetic field components need separate implementation
    return nullptr;
}

void MultiMagneticPlugin::on_register(PluginHost& host) {
    // Plugin initialization
}

void MultiMagneticPlugin::on_unregister() {
    // Cleanup
}

} // namespace mechatron
