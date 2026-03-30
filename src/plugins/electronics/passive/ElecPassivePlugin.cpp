#include "ElecPassivePlugin.hpp"
#include "electronics/CircuitSimulator.hpp"
#include "electronics/CircuitComponentAdapter.hpp"
#include <unordered_map>
#include <functional>

namespace mechatron {

std::vector<ComponentDescriptor> ElecPassivePlugin::components() const {
    return {
        {"resistor", "Resistor", "electronic", "Passive resistor component"},
        {"capacitor", "Capacitor", "electronic", "Passive capacitor component"},
        {"inductor", "Inductor", "electronic", "Passive inductor component"}
    };
}

std::unique_ptr<Component> ElecPassivePlugin::create(std::string_view type) {
    using namespace std;
    static const unordered_map<string, function<unique_ptr<Component>()>> factory = {
        {"resistor", []() { return make_unique<ResistorComponent>(make_unique<Resistor>()); }},
        {"capacitor", []() { return make_unique<CapacitorComponent>(make_unique<Capacitor>()); }},
        {"inductor", []() {
            // Inductor not yet implemented in CircuitSimulator
            return nullptr;
        }}
    };

    auto it = factory.find(string(type));
    if (it != factory.end()) {
        return it->second();
    }
    return nullptr;
}

void ElecPassivePlugin::on_register(PluginHost& host) {
    // Plugin initialization
}

void ElecPassivePlugin::on_unregister() {
    // Cleanup
}

} // namespace mechatron
