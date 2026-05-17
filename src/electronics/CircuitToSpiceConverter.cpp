#include "CircuitToSpiceConverter.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cctype>
#include <regex>

namespace mechatron {

namespace {
std::string to_lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::string prefixed_spice_name(char prefix, const std::string& id) {
    if (!id.empty() && std::toupper(static_cast<unsigned char>(id.front())) == std::toupper(static_cast<unsigned char>(prefix))) {
        return id;
    }
    return std::string(1, prefix) + id;
}
}

// ============================================================================
// CircuitToSpiceConverter
// ============================================================================

CircuitToSpiceConverter::CircuitToSpiceConverter()
    : m_current_node_number(1)
{
    reset();
}

CircuitToSpiceConverter::CircuitToSpiceConverter(const Config& config)
    : m_config(config)
    , m_current_node_number(1)
{
    reset();
}

void CircuitToSpiceConverter::reset() {
    m_builder.clear();
    m_pin_to_node.clear();
    m_component_models.clear();
    m_current_node_number = m_config.start_node_number;

    m_builder.set_title("Mechatron Circuit");

    if (m_config.use_standard_models) {
        add_standard_models();
    }
}

void CircuitToSpiceConverter::add_standard_models() {
    auto models = SpiceModelLibrary::get_standard_models();
    spdlog::debug("[CircuitToSpiceConverter] Adding {} standard models", models.size());
    for (const auto& model : models) {
        // Extract model/subcircuit name and add to builder
        if (model.find(".model ") == 0 || model.find(".subckt ") == 0) {
            m_builder.add_model(model);
            // Check if model contains .ends for subckt
            if (model.find(".subckt ") == 0) {
                if (model.find(".ends") == std::string::npos) {
                    spdlog::error("[CircuitToSpiceConverter] SUBCKT MISSING .ends: {}", model);
                } else {
                    spdlog::debug("[CircuitToSpiceConverter] Added .subckt model (ends at pos {})", model.find(".ends"));
                }
            } else {
                spdlog::debug("[CircuitToSpiceConverter] Added model: {}", model.substr(0, std::min(size_t(50), model.length())));
            }
        } else {
            spdlog::warn("[CircuitToSpiceConverter] Skipping model (doesn't start with .model or .subckt): {}", model.substr(0, std::min(size_t(50), model.length())));
        }
    }
}

std::string CircuitToSpiceConverter::generate_node_name(const std::string& pin_id) {
    // Check if this pin already has a node assigned
    auto it = m_pin_to_node.find(pin_id);
    if (it != m_pin_to_node.end()) {
        return it->second;
    }

    // Generate new node name
    std::string node_name;

    if (m_config.auto_assign_node_numbers) {
        node_name = std::to_string(m_current_node_number++);
    } else {
        // Use pin ID as node name (sanitized)
        node_name = "n_" + pin_id;
        std::replace(node_name.begin(), node_name.end(), ' ', '_');
        std::replace(node_name.begin(), node_name.end(), '-', '_');
    }

    m_pin_to_node[pin_id] = node_name;

    if (m_config.verbose) {
        spdlog::debug("[CircuitToSpiceConverter] Assigned node '{}' to pin '{}'", node_name, pin_id);
    }

    return node_name;
}

std::string CircuitToSpiceConverter::get_node_for_pin(const std::string& component_id, const std::string& pin_name) {
    std::string pin_id = component_id + "." + pin_name;

    // First check if this pin already has a node assigned from wire merging
    auto it = m_pin_to_node.find(pin_id);
    if (it != m_pin_to_node.end()) {
        return it->second;  // Return the pre-assigned node from wire merging
    }

    // If no pre-assigned node, generate a new one
    return generate_node_name(pin_id);
}

void CircuitToSpiceConverter::add_current_save_directives(const std::vector<CircuitComponent*>& components) {
    std::string save = "all";

    auto add = [&save](const std::string& expression) {
        save += " " + expression;
    };

    for (const auto* component : components) {
        if (!component) continue;

        const std::string id = component->id();
        const std::string type(component->type());

        if (type == "resistor") {
            add("@" + prefixed_spice_name('R', id) + "[i]");
        } else if (type == "capacitor") {
            add("@" + prefixed_spice_name('C', id) + "[i]");
        } else if (type == "inductor") {
            add("@" + prefixed_spice_name('L', id) + "[i]");
        } else if (type == "diode" || type == "zener_diode" || type == "led") {
            add("@" + prefixed_spice_name('D', id) + "[id]");
        } else if (type == "bjt_npn" || type == "bjt_pnp") {
            const auto q = prefixed_spice_name('Q', id);
            add("@" + q + "[ib]");
            add("@" + q + "[ic]");
            add("@" + q + "[ie]");
        } else if (type == "mosfet_n" || type == "mosfet_p") {
            const auto m = prefixed_spice_name('M', id);
            add("@" + m + "[ig]");
            add("@" + m + "[id]");
            add("@" + m + "[is]");
        } else if (type == "h_bridge") {
            add(to_lower(prefixed_spice_name('V', id + "_vcc")) + "#branch");
        } else if (type == "buck_converter") {
            add("e.x" + id + ".e_buck#branch");
            add("@r.x" + id + ".r_out[i]");
            add("v.x" + id + ".v_in_sense#branch");
        } else if (type == "boost_converter") {
            add("e.x" + id + ".e_boost#branch");
            add("@r.x" + id + ".r_out[i]");
            add("v.x" + id + ".v_in_sense#branch");
        }
    }

    m_builder.add_simulation("save", save);
}

std::string CircuitToSpiceConverter::color_to_string(LED::Color color) {
    switch (color) {
        case LED::Color::Red:        return "red";
        case LED::Color::Green:      return "green";
        case LED::Color::Blue:       return "blue";
        case LED::Color::Yellow:     return "yellow";
        case LED::Color::White:      return "white";
        case LED::Color::Infrared:   return "ir";
        case LED::Color::Ultraviolet:return "uv";
        case LED::Color::Custom:     return "red";  // Default
    }
    return "red";
}

std::string CircuitToSpiceConverter::get_unique_model_name(const std::string& base_name) {
    // Find a unique model name by adding a suffix if needed
    std::string model_name = base_name;
    int suffix = 1;

    while (m_component_models.find(model_name) != m_component_models.end()) {
        model_name = base_name + "_" + std::to_string(suffix++);
    }

    m_component_models[model_name] = true;
    return model_name;
}

CircuitToSpiceConverter::Result CircuitToSpiceConverter::convert(std::vector<CircuitComponent*>& components) {
    return convert_components(components, true);
}

CircuitToSpiceConverter::Result CircuitToSpiceConverter::convert_components(
    std::vector<CircuitComponent*>& components,
    bool reset_state) {
    Result result;
    result.success = false;
    result.total_components = 0;
    result.total_nodes = 0;

    try {
        if (reset_state) {
            reset();
        } else {
            m_builder.clear();
            m_component_models.clear();
            m_builder.set_title("Mechatron Circuit");

            if (m_config.use_standard_models) {
                add_standard_models();
            }
        }

        // Convert each component
        for (const auto* component : components) {
            if (!component) continue;

            std::string_view type = component->type();
            bool converted = false;

            if (m_config.verbose) {
                spdlog::trace("[CircuitToSpiceConverter] Converting component '{}' of type '{}'",
                           component->id(), type);
            }

            // Dispatch to appropriate converter
            if (type == "resistor") {
                converted = convert_resistor(static_cast<const Resistor*>(component));
            } else if (type == "capacitor") {
                converted = convert_capacitor(static_cast<const Capacitor*>(component));
            } else if (type == "inductor") {
                converted = convert_inductor(static_cast<const Inductor*>(component));
            } else if (type == "diode") {
                converted = convert_diode(static_cast<const Diode*>(component));
            } else if (type == "led") {
                converted = convert_led(static_cast<const LED*>(component));
            } else if (type == "zener_diode") {
                converted = convert_zener_diode(static_cast<const ZenerDiode*>(component));
            } else if (type == "bjt_npn" || type == "bjt_pnp") {
                converted = convert_bjt_transistor(static_cast<const BJTTransistor*>(component));
            } else if (type == "mosfet_n" || type == "mosfet_p") {
                converted = convert_mosfet_transistor(static_cast<const MOSFETTransistor*>(component));
            } else if (type == "motor_driver") {
                converted = convert_motor_driver(static_cast<const MotorDriver*>(component));
            } else if (type == "buck_converter") {
                converted = convert_buck_converter(static_cast<const BuckConverter*>(component));
            } else if (type == "boost_converter") {
                converted = convert_boost_converter(static_cast<const BoostConverter*>(component));
            } else if (type == "dc_voltage") {
                converted = convert_dc_voltage_source(static_cast<const DCVoltageSource*>(component));
            } else if (type == "ground") {
                converted = convert_ground(static_cast<const Ground*>(component));
            } else if (type == "h_bridge") {
                converted = convert_h_bridge(static_cast<const HBridge*>(component));
            } else {
                // Try generic conversion
                converted = convert_generic_component(component);
            }

            if (converted) {
                result.total_components++;
            } else {
                spdlog::warn("[CircuitToSpiceConverter] Failed to convert component '{}' of type '{}'",
                           component->id(), type);
            }
        }

        // Add simulation command if requested
        if (m_config.include_simulation_command) {
            add_current_save_directives(components);

            std::string params;
            if (m_config.analysis_type == "tran") {
                params = std::to_string(m_config.simulation_time_step) + " " +
                         std::to_string(m_config.simulation_duration);
            }
            m_builder.add_simulation(m_config.analysis_type, params);
        }

        // Build the netlist
        result.netlist = m_builder.build();
        result.pin_to_node = m_pin_to_node;
        result.total_nodes = static_cast<int>(m_pin_to_node.size());
        result.success = true;

        spdlog::debug("[CircuitToSpiceConverter] Successfully converted {} components with {} nodes",
                   result.total_components, result.total_nodes);

        // Log the netlist for debugging
        spdlog::debug("[CircuitToSpiceConverter] Generated netlist:\n{}", result.netlist);

    } catch (const std::exception& e) {
        result.success = false;
        result.error = std::string("Exception during conversion: ") + e.what();
        spdlog::error("[CircuitToSpiceConverter] {}", result.error);
    }

    return result;
}

CircuitToSpiceConverter::Result CircuitToSpiceConverter::convert_with_wires(
    std::vector<CircuitComponent*>& components,
    const std::vector<Wire>& wires) {

    // First, assign nodes based on wire connections
    // Pins connected by wires should share the same node
    reset();

    // Build connectivity map
    std::unordered_map<std::string, std::vector<std::string>> pin_groups;
    std::unordered_map<std::string, std::string> pin_to_group;

    // Initialize each pin as its own group
    for (const auto* component : components) {
        if (!component) continue;
        auto pins = component->get_pins();
        for (const auto* pin : pins) {
            std::string pin_id = pin->component_id + "." + pin->id;
            pin_groups[pin_id] = {pin_id};
            pin_to_group[pin_id] = pin_id;
        }
    }

    // Merge groups based on wire connections
    spdlog::trace("[CircuitToSpice] Processing {} wire connections", wires.size());
    for (const auto& wire : wires) {
        if (!wire.source || !wire.target) continue;

        std::string source_pin = wire.source->component_id + "." + wire.source->id;
        std::string target_pin = wire.target->component_id + "." + wire.target->id;
        spdlog::trace("[CircuitToSpice] Wire: {} -> {}", source_pin, target_pin);

        // Find the groups for these pins
        std::string source_group = pin_to_group[source_pin];
        std::string target_group = pin_to_group[target_pin];

        if (source_group != target_group) {
            // Merge groups
            auto& source_vec = pin_groups[source_group];
            auto& target_vec = pin_groups[target_group];

            spdlog::trace("[CircuitToSpice] Merging groups: {} ({} pins) + {} ({} pins)",
                         source_group, source_vec.size(), target_group, target_vec.size());

            // Add all pins from target group to source group
            for (const auto& pin : target_vec) {
                source_vec.push_back(pin);
                pin_to_group[pin] = source_group;
            }

            // Remove target group
            pin_groups.erase(target_group);
        }
    }

    // Log final groups
    spdlog::trace("[CircuitToSpice] Final pin groups: {} groups", pin_groups.size());
    for (const auto& [group_id, pins] : pin_groups) {
        spdlog::trace("[CircuitToSpice] Group '{}' ({} pins): {}", group_id, pins.size(),
                     pins.empty() ? "empty" : pins[0]);
    }

    // Check if circuit has any ground component
    bool circuit_has_ground = false;
    for (const auto* component : components) {
        if (component && component->type() == "ground") {
            circuit_has_ground = true;
            spdlog::trace("[CircuitToSpiceConverter] Found ground component: {}", component->id());
            break;
        }
    }

    if (!circuit_has_ground) {
        spdlog::debug("[CircuitToSpiceConverter] No ground component found in circuit");
    }

    // Find first voltage source and detect actuators if no ground exists
    const DCVoltageSource* first_voltage_source = nullptr;
    std::vector<const CircuitComponent*> actuators_found;

    if (!circuit_has_ground) {
        for (const auto* component : components) {
            if (!component) continue;

            if (!first_voltage_source && component->type() == "dc_voltage") {
                first_voltage_source = static_cast<const DCVoltageSource*>(component);
                spdlog::trace("[CircuitToSpiceConverter] Found voltage source: {}", component->id());
            }

            // Also look for actuators that need ground reference
            std::string_view ctype = component->type();
            if (ctype == "dc_motor" || ctype == "servo_motor" || ctype == "solenoid_actuator") {
                actuators_found.push_back(component);
                spdlog::trace("[CircuitToSpiceConverter] Found actuator: {} (type: {})", component->id(), ctype);
            }
        }
        if (first_voltage_source) {
            spdlog::debug("[CircuitToSpiceConverter] No ground component found; using voltage source '{}' negative terminal as node 0", first_voltage_source->id());
        }
    }

    // Assign nodes to each group
    for (const auto& [group_id, pins] : pin_groups) {
        // Check if this group contains a ground pin
        bool has_ground = false;
        for (const auto& pin : pins) {
            // Extract component ID from pin_id (format: "component_id.pin_name")
            size_t dot_pos = pin.find('.');
            if (dot_pos != std::string::npos) {
                std::string component_id = pin.substr(0, dot_pos);
                // Find the component and check if it's a ground
                for (const auto* component : components) {
                    if (component && component->id() == component_id && component->type() == "ground") {
                        has_ground = true;
                        spdlog::trace("[CircuitToSpiceConverter] Group '{}' contains ground pin '{}', assigning to node 0", group_id, pin);
                        break;
                    }
                }
                if (has_ground) break;
            }
        }

        // CRITICAL: If no ground exists and this is NOT the voltage source GND group,
        // and we have a ground component, check if this group is connected to ground
        // This prevents floating nodes that cause singular matrix errors
        if (!has_ground && circuit_has_ground && first_voltage_source) {
            // Check if any pin in this group belongs to the voltage source
            bool is_vs_group = false;
            for (const auto& pin : pins) {
                size_t dot_pos = pin.find('.');
                if (dot_pos != std::string::npos) {
                    std::string component_id = pin.substr(0, dot_pos);
                    if (component_id == first_voltage_source->id()) {
                        is_vs_group = true;
                        break;
                    }
                }
            }

            // If this is not the voltage source group and not grounded, warn about floating node
            if (!is_vs_group) {
                std::string pins_str;
                for (size_t i = 0; i < pins.size(); ++i) {
                    if (i > 0) pins_str += ", ";
                    pins_str += pins[i];
                }
                spdlog::warn("[CircuitToSpiceConverter] Group '{}' appears to be floating (no ground connection). This may cause simulation errors.", group_id);
                spdlog::warn("[CircuitToSpiceConverter]   Pins in group: {}", pins_str);
            }
        }

        // If no ground component exists and this is the voltage source's negative pin, force it to ground
        if (!has_ground && first_voltage_source != nullptr) {
            for (const auto& pin : pins) {
                size_t dot_pos = pin.find('.');
                if (dot_pos != std::string::npos) {
                    std::string component_id = pin.substr(0, dot_pos);
                    std::string pin_name = pin.substr(dot_pos + 1);
                    // DC voltage source has pins "V+" and "GND", connect GND to ground
                    if (component_id == first_voltage_source->id() && pin_name == "GND") {
                        has_ground = true;
                        spdlog::trace("[CircuitToSpiceConverter] Auto-assigning voltage source '{}' GND terminal (pin: {}) to ground (node 0)", first_voltage_source->id(), pin);
                        break;
                    }
                }
            }
        }

        std::string node_name;
        if (has_ground) {
            node_name = "0";  // Ground node
        } else {
            node_name = generate_node_name(group_id);
        }

        // All pins in this group get the same node
        for (const auto& pin : pins) {
            m_pin_to_node[pin] = node_name;
            spdlog::trace("[CircuitToSpiceConverter] Pin '{}' -> node '{}'", pin, node_name);
        }
    }

    // Now convert components with pre-assigned nodes. Do not call convert()
    // here: convert() resets m_pin_to_node, which would discard the wire
    // groups we just built and split connected pins into separate SPICE nodes.
    return convert_components(components, false);
}

// ============================================================================
// Component-specific converters
// ============================================================================

bool CircuitToSpiceConverter::convert_resistor(const Resistor* resistor) {
    if (!resistor) return false;

    auto pins = resistor->get_pins();
    if (pins.size() < 2) return false;

    std::string node1 = get_node_for_pin(resistor->id(), pins[0]->id);
    std::string node2 = get_node_for_pin(resistor->id(), pins[1]->id);
    double resistance = resistor->get_parameter("resistance");

    m_builder.add_resistor(prefixed_spice_name('R', resistor->id()), node1, node2, resistance);
    return true;
}

bool CircuitToSpiceConverter::convert_capacitor(const Capacitor* capacitor) {
    if (!capacitor) return false;

    auto pins = capacitor->get_pins();
    if (pins.size() < 2) return false;

    std::string node1 = get_node_for_pin(capacitor->id(), pins[0]->id);
    std::string node2 = get_node_for_pin(capacitor->id(), pins[1]->id);
    double capacitance = capacitor->get_parameter("capacitance");

    m_builder.add_capacitor(prefixed_spice_name('C', capacitor->id()), node1, node2, capacitance);
    return true;
}

bool CircuitToSpiceConverter::convert_inductor(const Inductor* inductor) {
    if (!inductor) return false;

    auto pins = inductor->get_pins();
    if (pins.size() < 2) return false;

    std::string node1 = get_node_for_pin(inductor->id(), pins[0]->id);
    std::string node2 = get_node_for_pin(inductor->id(), pins[1]->id);
    double inductance = inductor->get_parameter("inductance");

    m_builder.add_inductor(prefixed_spice_name('L', inductor->id()), node1, node2, inductance);
    return true;
}

bool CircuitToSpiceConverter::convert_diode(const Diode* diode) {
    if (!diode) return false;

    auto pins = diode->get_pins();
    if (pins.size() < 2) return false;

    std::string node_anode = get_node_for_pin(diode->id(), pins[0]->id);
    std::string node_cathode = get_node_for_pin(diode->id(), pins[1]->id);

    // Use standard diode model
    m_builder.add_diode(prefixed_spice_name('D', diode->id()), node_anode, node_cathode, "D1N4148");
    return true;
}

bool CircuitToSpiceConverter::convert_led(const LED* led) {
    if (!led) return false;

    auto pins = led->get_pins();
    if (pins.size() < 2) return false;

    std::string node_anode = get_node_for_pin(led->id(), pins[0]->id);
    std::string node_cathode = get_node_for_pin(led->id(), pins[1]->id);

    double vf = led->get_parameter("forward_voltage");

    m_builder.add_led(prefixed_spice_name('D', led->id()), node_anode, node_cathode, vf);
    return true;
}

bool CircuitToSpiceConverter::convert_zener_diode(const ZenerDiode* zener) {
    if (!zener) return false;

    auto pins = zener->get_pins();
    if (pins.size() < 2) return false;

    std::string node_anode = get_node_for_pin(zener->id(), pins[0]->id);
    std::string node_cathode = get_node_for_pin(zener->id(), pins[1]->id);

    // Use zener diode model
    m_builder.add_diode(prefixed_spice_name('D', zener->id()), node_anode, node_cathode, "DZENER_5V1");
    return true;
}

bool CircuitToSpiceConverter::convert_bjt_transistor(const BJTTransistor* bjt) {
    if (!bjt) return false;

    auto pins = bjt->get_pins();
    if (pins.size() < 3) return false;

    std::string node_base = get_node_for_pin(bjt->id(), "base");
    std::string node_collector = get_node_for_pin(bjt->id(), "collector");
    std::string node_emitter = get_node_for_pin(bjt->id(), "emitter");

    bool is_npn = (bjt->type() == "bjt_npn");
    std::string model_name = is_npn ? "Q2N2222" : "Q2N3906";

    m_builder.add_bjt(prefixed_spice_name('Q', bjt->id()), node_collector, node_base, node_emitter, model_name);
    m_builder.add_resistor(prefixed_spice_name('R', bjt->id() + "_be_leak"), node_base, node_emitter, 1e9);
    return true;
}

bool CircuitToSpiceConverter::convert_mosfet_transistor(const MOSFETTransistor* mosfet) {
    if (!mosfet) return false;

    auto pins = mosfet->get_pins();
    if (pins.size() < 3) return false;

    std::string node_gate = get_node_for_pin(mosfet->id(), "gate");
    std::string node_drain = get_node_for_pin(mosfet->id(), "drain");
    std::string node_source = get_node_for_pin(mosfet->id(), "source");

    // For discrete MOSFETs, bulk is connected to source
    std::string node_bulk = node_source;

    bool is_nmos = (mosfet->type() == "mosfet_n");
    std::string model_name = is_nmos ? "LOGIC_NMOS" : "LOGIC_PMOS";

    m_builder.add_mosfet(prefixed_spice_name('M', mosfet->id()), node_drain, node_gate, node_source, node_bulk, model_name);
    m_builder.add_resistor(prefixed_spice_name('R', mosfet->id() + "_gs_leak"), node_gate, node_source, 1e9);
    return true;
}

bool CircuitToSpiceConverter::convert_motor_driver(const MotorDriver* driver) {
    if (!driver) return false;

    auto pins = driver->get_pins();
    if (pins.size() < 7) return false;

    // Motor driver pins: PWM, DIR, EN, OUT+, OUT-, VCC, GND
    std::string node_pwm = get_node_for_pin(driver->id(), "PWM");
    std::string node_dir = get_node_for_pin(driver->id(), "DIR");
    std::string node_en = get_node_for_pin(driver->id(), "EN");
    std::string node_out_pos = get_node_for_pin(driver->id(), "OUT+");
    std::string node_out_neg = get_node_for_pin(driver->id(), "OUT-");
    std::string node_vcc = get_node_for_pin(driver->id(), "VCC");
    std::string node_gnd = get_node_for_pin(driver->id(), "GND");

    m_builder.add_motor_driver(driver->id(), node_vcc, node_gnd,
                              node_pwm, node_dir, node_en, node_gnd,
                              node_out_pos, node_out_neg);

    return true;
}

bool CircuitToSpiceConverter::convert_buck_converter(const BuckConverter* buck) {
    if (!buck) return false;

    auto pins = buck->get_pins();
    if (pins.size() < 3) return false;

    std::string node_vin = get_node_for_pin(buck->id(), "VIN");
    std::string node_gnd = get_node_for_pin(buck->id(), "GND");
    std::string node_vout = get_node_for_pin(buck->id(), "VOUT");

    double duty = buck->get_parameter("duty_cycle");
    double L = 10e-6;   // 10 µH default
    double C = 100e-6;  // 100 µF default
    double freq = 1e3;  // 1 kHz switching frequency (slower for better simulation visibility)

    // Buck converter now has internal PWM generation, no external PWM needed
    m_builder.add_buck_converter(buck->id(), node_vin, node_gnd, node_vout, L, C, freq, "NMOS_PWR", duty);

    return true;
}

bool CircuitToSpiceConverter::convert_boost_converter(const BoostConverter* boost) {
    if (!boost) return false;

    auto pins = boost->get_pins();
    if (pins.size() < 3) return false;

    std::string node_vin = get_node_for_pin(boost->id(), "VIN");
    std::string node_gnd = get_node_for_pin(boost->id(), "GND");
    std::string node_vout = get_node_for_pin(boost->id(), "VOUT");

    double duty = boost->get_parameter("duty_cycle");
    double L = 10e-6;   // 10 µH default
    double C = 100e-6;  // 100 µF default
    double freq = 100e3; // 100 kHz switching frequency

    // Boost converter now has internal PWM generation, no external PWM needed
    m_builder.add_boost_converter(boost->id(), node_vin, node_gnd, node_vout, L, C, freq, "NMOS_PWR", duty);

    return true;
}

bool CircuitToSpiceConverter::convert_dc_voltage_source(const DCVoltageSource* source) {
    if (!source) return false;

    auto pins = source->get_pins();
    if (pins.size() < 2) return false;

    std::string node_pos = get_node_for_pin(source->id(), pins[0]->id);
    std::string node_neg = get_node_for_pin(source->id(), pins[1]->id);
    double voltage = source->get_parameter("voltage");

    // CRITICAL FIX: If the negative terminal (GND) is not connected to ground (node 0),
    // automatically remap it to node 0. This prevents floating nodes and singular matrix errors.
    if (node_neg != "0") {
        spdlog::debug("[CircuitToSpiceConverter] Auto-grounding DC voltage source '{}' negative terminal from node '{}' to '0'", source->id(), node_neg);

        // Find all pins mapped to node_neg and remap them to node 0
        for (auto& [pin_id, node] : m_pin_to_node) {
            if (node == node_neg) {
                spdlog::debug("[CircuitToSpiceConverter] Remapped pin '{}' from node '{}' to '0'", pin_id, node_neg);
                node = "0";
            }
        }

        // Update node_neg to 0
        node_neg = "0";
    }

    // SPICE requires voltage sources to start with 'V'
    std::string spice_name = "V" + source->id();
    m_builder.add_dc_voltage(spice_name, node_pos, node_neg, voltage);
    return true;
}

bool CircuitToSpiceConverter::convert_ground(const Ground* ground) {
    if (!ground) return false;

    auto pins = ground->get_pins();
    if (pins.size() < 1) return false;

    // In SPICE, ground is implicitly node 0
    // We need to map the ground pin to node "0" directly
    std::string ground_pin_id = ground->id() + "." + pins[0]->id;

    spdlog::trace("[CircuitToSpiceConverter] Processing ground component: {}, pin: {}", ground->id(), pins[0]->id);
    spdlog::trace("[CircuitToSpiceConverter] Total pins in m_pin_to_node: {}", m_pin_to_node.size());

    // Check if this pin is already in a group (from wire connections)
    // If so, we need to map the ENTIRE group to node 0 (all pins connected to ground)
    auto it = m_pin_to_node.find(ground_pin_id);
    if (it != m_pin_to_node.end()) {
        // This pin is already mapped to a node from wire grouping
        // We need to remap it to ground (node 0)
        std::string old_node = it->second;
        spdlog::trace("[CircuitToSpiceConverter] Ground pin '{}' found in node '{}', remapping to '0'", ground_pin_id, old_node);

        // CRITICAL: Remap ALL pins that were mapped to the old node to node 0
        // But ONLY for circuit components, NOT actuators (motors, servos, etc.)
        int remapped_count = 0;
        for (auto& [pin_id, node] : m_pin_to_node) {
            // Skip actuators - they should NOT be auto-grounded
            // Actuators have component types like: dc_motor, servo_motor, solenoid_actuator
            bool is_actuator = false;
            for (const auto& actuator_type : {"dc_motor", "servo_motor", "solenoid_actuator", "stepper_motor"}) {
                if (pin_id.find(actuator_type) != std::string::npos) {
                    is_actuator = true;
                    spdlog::debug("[CircuitToSpiceConverter] Skipping auto-ground for actuator pin '{}'", pin_id);
                    break;
                }
            }

            if (!is_actuator && node == old_node) {
                node = "0";
                remapped_count++;
                spdlog::trace("[CircuitToSpiceConverter] Remapped pin '{}' from node '{}' to '0'", pin_id, old_node);
            }
        }
        spdlog::trace("[CircuitToSpiceConverter] Remapped {} pins from node '{}' to '0'", remapped_count, old_node);
    } else {
        // CRITICAL FIX: Ground pin is not in the map, which means it's not connected via wires.
        // This is actually OK - we need to find OTHER pins that should be grounded.
        // Look for pins with "GND" in their name and map them to node 0.
        // But ONLY for voltage sources, NOT actuators!
        spdlog::trace("[CircuitToSpiceConverter] Ground pin '{}' not found in map (not connected via wires)", ground_pin_id);
        spdlog::trace("[CircuitToSpiceConverter] Looking for voltage source GND pins to auto-ground...");

        int auto_grounded_count = 0;
        for (auto& [pin_id, node] : m_pin_to_node) {
            // Check if this pin has "GND" in its name (e.g., "dc_voltage_1.GND")
            // AND it's from a voltage source (NOT from actuators)
            if ((pin_id.find(".GND") != std::string::npos || pin_id.find(".gnd") != std::string::npos)) {
                // Skip actuators
                bool is_actuator = false;
                for (const auto& actuator_type : {"dc_motor", "servo_motor", "solenoid_actuator", "stepper_motor"}) {
                    if (pin_id.find(actuator_type) != std::string::npos) {
                        is_actuator = true;
                        spdlog::debug("[CircuitToSpiceConverter] Skipping auto-ground for actuator pin '{}'", pin_id);
                        break;
                    }
                }

                if (!is_actuator) {
                    std::string old_node = node;
                    node = "0";  // Remap to ground
                    auto_grounded_count++;
                    spdlog::trace("[CircuitToSpiceConverter] Auto-grounded pin '{}' from node '{}' to '0'", pin_id, old_node);
                }
            }
        }

        if (auto_grounded_count > 0) {
            spdlog::trace("[CircuitToSpiceConverter] Auto-grounded {} voltage source pins", auto_grounded_count);
        } else {
            spdlog::trace("[CircuitToSpiceConverter] No GND pins found to auto-ground");
        }

        // Also map the ground pin itself to node 0
        m_pin_to_node[ground_pin_id] = "0";
        spdlog::trace("[CircuitToSpiceConverter] Mapped ground pin '{}' to node '0'", ground_pin_id);
    }

    // Don't add ground to the netlist - it's implicit in SPICE
    // But we do return true to indicate successful conversion
    return true;
}

bool CircuitToSpiceConverter::convert_h_bridge(const HBridge* hbridge) {
    if (!hbridge) return false;

    auto pins = hbridge->get_pins();
    if (pins.size() < 7) return false;

    // H-Bridge pins: IN1, IN2, EN, OUT1, OUT2, VCC, GND
    std::string node_in1 = get_node_for_pin(hbridge->id(), "IN1");
    std::string node_in2 = get_node_for_pin(hbridge->id(), "IN2");
    std::string node_en = get_node_for_pin(hbridge->id(), "EN");
    std::string node_out1 = get_node_for_pin(hbridge->id(), "OUT1");
    std::string node_out2 = get_node_for_pin(hbridge->id(), "OUT2");
    std::string node_vcc = get_node_for_pin(hbridge->id(), "VCC");
    std::string node_gnd = get_node_for_pin(hbridge->id(), "GND");

    m_builder.add_h_bridge(hbridge->id(), node_vcc, node_gnd,
                           node_in1, node_in2, node_en,
                           node_out1, node_out2);

    return true;
}

bool CircuitToSpiceConverter::convert_generic_component(const CircuitComponent* component) {
    if (!component) return false;

    spdlog::warn("[CircuitToSpiceConverter] No specific converter for component type '{}', using generic",
               component->type());

    // Generic conversion: try to extract basic 2-pin behavior
    auto pins = component->get_pins();
    if (pins.size() >= 2) {
        std::string node1 = get_node_for_pin(component->id(), pins[0]->id);
        std::string node2 = get_node_for_pin(component->id(), pins[1]->id);

        // Add as a resistor with very high resistance (open circuit approximation)
        m_builder.add_resistor(component->id(), node1, node2, 1e9);
        return true;
    }

    return false;
}

// ============================================================================
// CircuitSimulatorWithSpice
// ============================================================================

CircuitSimulatorWithSpice::CircuitSimulatorWithSpice(SimulationMode mode)
    : m_mode(mode)
    , m_converter()
{
#if MECHATRON_HAVE_NGSPICE
    m_ngspice = std::make_unique<NgspiceWrapper>();
#endif
}

void CircuitSimulatorWithSpice::add_component(CircuitComponent* component) {
    if (component) {
        m_components.push_back(component);
    }
}

void CircuitSimulatorWithSpice::add_wire(const Wire& wire) {
    m_wires.push_back(wire);
}

CircuitComponent* CircuitSimulatorWithSpice::get_component(const std::string& id) {
    for (auto* component : m_components) {
        if (component && component->id() == id) {
            return component;
        }
    }
    return nullptr;
}

void CircuitSimulatorWithSpice::reset() {
    m_components.clear();
    m_wires.clear();
}

bool CircuitSimulatorWithSpice::is_ngspice_available() const {
#if MECHATRON_HAVE_NGSPICE
    return m_ngspice && m_ngspice->is_available();
#else
    return false;
#endif
}

bool CircuitSimulatorWithSpice::should_use_ngspice() const {
    switch (m_mode) {
        case SimulationMode::NativeMNA:
            return false;
        case SimulationMode::Ngspice:
            return is_ngspice_available();
        case SimulationMode::Hybrid:
            // Use ngspice for complex circuits (>10 components or contains non-linear elements)
            if (!is_ngspice_available()) return false;

            // Count complex components
            int complex_count = 0;
            for (const auto* comp : m_components) {
                if (!comp) continue;
                std::string_view type = comp->type();
                if (type == "bjt_npn" || type == "bjt_pnp" ||
                    type == "mosfet_n" || type == "mosfet_p" ||
                    type == "diode" || type == "led" ||
                    type == "motor_driver" ||
                    type == "buck_converter" || type == "boost_converter") {
                    complex_count++;
                }
            }

            // Use ngspice if we have complex components or many total components
            return complex_count > 0 || m_components.size() > 10;
    }
    return false;
}

CircuitSimulationResult CircuitSimulatorWithSpice::simulate(double duration, double time_step) {
    CircuitSimulationResult result;
    result.success = false;
    result.simulation_time = duration;
    result.time_steps = static_cast<int>(duration / time_step);

    if (should_use_ngspice()) {
        spdlog::info("[CircuitSimulatorWithSpice] Using ngspice for simulation");
        result = simulate_with_ngspice(duration, time_step);
    } else {
        spdlog::info("[CircuitSimulatorWithSpice] Using native MNA for simulation");
        result = simulate_with_mna(duration, time_step);
    }

    return result;
}

CircuitSimulationResult CircuitSimulatorWithSpice::simulate_with_mna(double duration, double time_step) {
    CircuitSimulationResult result;
    result.success = false;
    result.simulation_time = duration;

    result.error = "Native MNA simulation not yet implemented through this interface";

    spdlog::warn("[CircuitSimulatorWithSpice] {}", result.error);
    return result;
}

CircuitSimulationResult CircuitSimulatorWithSpice::simulate_with_ngspice(double duration, double time_step) {
    CircuitSimulationResult result;
    result.success = false;
    result.simulation_time = duration;
    result.time_steps = static_cast<int>(duration / time_step);

#if MECHATRON_HAVE_NGSPICE
    if (!m_ngspice || !m_ngspice->is_available()) {
        result.error = "ngspice is not available";
        spdlog::error("[CircuitSimulatorWithSpice] {}", result.error);
        return result;
    }

    // Convert circuit to SPICE netlist
    CircuitToSpiceConverter::Config config;
    config.include_simulation_command = true;
    config.simulation_duration = duration;
    config.simulation_time_step = time_step;
    m_converter.set_config(config);

    auto convert_result = m_converter.convert_with_wires(m_components, m_wires);

    if (!convert_result.success) {
        result.error = "Failed to convert circuit to SPICE: " + convert_result.error;
        spdlog::error("[CircuitSimulatorWithSpice] {}", result.error);
        return result;
    }

    // Run simulation with ngspice
    auto ngspice_result = m_ngspice->simulate(convert_result.netlist, duration, time_step);

    if (!ngspice_result.success) {
        result.error = "ngspice simulation failed: " + ngspice_result.error;
        spdlog::error("[CircuitSimulatorWithSpice] {}", result.error);
        return result;
    }

    // Convert ngspice results to our format
    for (const auto& time_point : ngspice_result.time_points) {
        double t = time_point.first;

        for (const auto& nv : time_point.second) {
            result.node_voltages[nv.node].push_back({t, nv.voltage});
        }
    }

    result.success = true;
    spdlog::info("[CircuitSimulatorWithSpice] Simulation completed with {} time points",
               ngspice_result.time_points.size());

#else
    result.error = "ngspice support was not enabled during build";
    spdlog::error("[CircuitSimulatorWithSpice] {}", result.error);
#endif

    return result;
}

} // namespace mechatron
