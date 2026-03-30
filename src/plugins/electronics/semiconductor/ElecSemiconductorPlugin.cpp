#include "ElecSemiconductorPlugin.hpp"
#include "electronics/CircuitSimulator.hpp"
#include "electronics/CircuitComponentAdapter.hpp"
#include <unordered_map>
#include <functional>

namespace mechatron {

std::vector<ComponentDescriptor> ElecSemiconductorPlugin::components() const {
    return {
        {"diode", "Diode", "semiconductor", "General purpose diode"},
        {"zener_diode", "Zener Diode", "semiconductor", "Zener voltage reference diode"},
        {"led", "LED", "optoelectronic", "Light emitting diode"},
        {"bjt_npn", "NPN Transistor", "semiconductor", "NPN bipolar junction transistor"},
        {"bjt_pnp", "PNP Transistor", "semiconductor", "PNP bipolar junction transistor"},
        {"mosfet_n", "N-Channel MOSFET", "semiconductor", "N-channel enhancement MOSFET"},
        {"mosfet_p", "P-Channel MOSFET", "semiconductor", "P-channel enhancement MOSFET"}
    };
}

std::unique_ptr<Component> ElecSemiconductorPlugin::create(std::string_view type) {
    using namespace std;
    static const unordered_map<string, function<unique_ptr<Component>()>> factory = {
        {"led", []() { return make_unique<LEDComponent>(make_unique<LED>()); }}
        // Other semiconductor types need CircuitSimulator implementations
    };

    auto it = factory.find(string(type));
    if (it != factory.end()) {
        return it->second();
    }
    return nullptr;
}

void ElecSemiconductorPlugin::on_register(PluginHost& host) {
    // Plugin initialization
}

void ElecSemiconductorPlugin::on_unregister() {
    // Cleanup
}

} // namespace mechatron
