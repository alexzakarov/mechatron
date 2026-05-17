#include "CircuitSimulator.hpp"
#include "CircuitToSpiceConverter.hpp"
#include "NgspiceWrapper.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cctype>
#include <mutex>

namespace mechatron {

// Circuit simulation constants
namespace CircuitConstants {
    constexpr float GROUND_VOLTAGE = 0.0f;           // Ground reference voltage (V)
    constexpr float NORMALIZED_MIN = 0.0f;            // Minimum normalized value
    constexpr float NORMALIZED_MAX = 1.0f;            // Maximum normalized value
    constexpr float PWM_PERIOD = 1.0f;                // Full PWM period (normalized)
    constexpr float DEFAULT_SUPPLY_TOLERANCE = 0.0f;  // Minimum valid supply voltage (V)
}

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

std::string current_key(const std::string& expression) {
    return to_lower(expression);
}

void set_two_pin_current(CircuitComponent* comp, double current) {
    auto pins = comp->get_pins();
    if (pins.size() < 2) return;
    if (pins[0]) pins[0]->current = static_cast<float>(current);
    if (pins[1]) pins[1]->current = static_cast<float>(-current);
}

void set_pin_current(CircuitComponent* comp, const std::string& pin_name, double current) {
    for (auto* pin : comp->get_pins()) {
        if (pin && pin->id == pin_name) {
            pin->current = static_cast<float>(current);
            return;
        }
    }
}
}

CircuitSimulator::CircuitSimulator(CircuitSimulationMode mode) {
    (void)mode;
#if MECHATRON_HAVE_NGSPICE
    m_ngspice = std::make_unique<NgspiceWrapper>();
#endif
}

CircuitSimulator::~CircuitSimulator() = default;

bool CircuitSimulator::remove_component(std::string_view id) {
    std::string id_str(id);

    // Check if it's an external component
    auto ext_it = m_components_external.find(id_str);
    if (ext_it != m_components_external.end()) {
        // Remove connected wires
        std::vector<std::string> wires_to_remove;
        for (auto& [wire_id, wire] : m_wires) {
            if ((wire->source && wire->source->component_id == id_str) ||
                (wire->target && wire->target->component_id == id_str)) {
                wires_to_remove.push_back(wire_id);
            }
        }
        for (const auto& wire_id : wires_to_remove) {
            m_wires.erase(wire_id);
        }

        // Just remove the reference - don't delete the external component
        m_components_external.erase(ext_it);
        return true;
    }

    // Check if it's an owned component
    auto owned_it = m_components_owned.find(id_str);
    if (owned_it != m_components_owned.end()) {
        // Remove connected wires
        std::vector<std::string> wires_to_remove;
        for (auto& [wire_id, wire] : m_wires) {
            if ((wire->source && wire->source->component_id == id_str) ||
                (wire->target && wire->target->component_id == id_str)) {
                wires_to_remove.push_back(wire_id);
            }
        }
        for (const auto& wire_id : wires_to_remove) {
            m_wires.erase(wire_id);
        }

        // unique_ptr will automatically delete the component
        m_components_owned.erase(owned_it);
        return true;
    }

    return false;
}

CircuitComponent* CircuitSimulator::get_component(std::string_view id) {
    std::string id_str(id);

    // Check external components first
    auto ext_it = m_components_external.find(id_str);
    if (ext_it != m_components_external.end()) {
        return ext_it->second;
    }

    // Check owned components
    auto owned_it = m_components_owned.find(id_str);
    if (owned_it != m_components_owned.end()) {
        return owned_it->second.get();
    }

    return nullptr;
}

bool CircuitSimulator::connect(const std::string& wire_id,
                                const std::string& source_comp,
                                const std::string& source_pin,
                                const std::string& target_comp,
                                const std::string& target_pin) {
    // Find source pin
    auto* src_comp = get_component(source_comp);
    if (!src_comp) {
        spdlog::error("Source component not found: {}", source_comp);
        return false;
    }

    auto src_pins = src_comp->get_pins();
    CircuitPin* src_pin_ptr = nullptr;
    for (auto* pin : src_pins) {
        if (pin->id == source_pin) {
            src_pin_ptr = pin;
            break;
        }
    }

    if (!src_pin_ptr) {
        spdlog::error("Source pin not found: {}.{}", source_comp, source_pin);
        return false;
    }

    // Find target pin
    auto* tgt_comp = get_component(target_comp);
    if (!tgt_comp) {
        spdlog::error("Target component not found: {}", target_comp);
        return false;
    }

    auto tgt_pins = tgt_comp->get_pins();
    CircuitPin* tgt_pin_ptr = nullptr;
    for (auto* pin : tgt_pins) {
        if (pin->id == target_pin) {
            tgt_pin_ptr = pin;
            break;
        }
    }

    if (!tgt_pin_ptr) {
        spdlog::error("Target pin not found: {}.{}", target_comp, target_pin);
        return false;
    }

    // Create wire
    auto wire = std::make_unique<Wire>();
    wire->id = wire_id;
    wire->source = src_pin_ptr;
    wire->target = tgt_pin_ptr;
    m_wires[wire_id] = std::move(wire);

    return true;
}

bool CircuitSimulator::disconnect(const std::string& wire_id) {
    auto it = m_wires.find(wire_id);
    if (it == m_wires.end()) return false;

    m_wires.erase(it);
    return true;
}

void CircuitSimulator::step(double dt) {
    // Only run simulation if we have components
    if (m_components_owned.empty() && m_components_external.empty()) return;

    // Run ngspice circuit simulation
    step_with_ngspice(dt);
}

void CircuitSimulator::step_with_ngspice(double dt) {

#if MECHATRON_HAVE_NGSPICE
    static std::mutex ngspice_mutex;
    std::lock_guard<std::mutex> ngspice_lock(ngspice_mutex);

    // Collect all components
    std::vector<CircuitComponent*> all_components;
    for (auto& [id, comp] : m_components_owned) {
        all_components.push_back(comp.get());
    }
    for (auto& [id, comp] : m_components_external) {
        all_components.push_back(comp);
    }

    // Collect all wires
    std::vector<Wire> wires;
    for (auto& [id, wire] : m_wires) {
        wires.push_back(*wire);
    }

    try {
        if (!m_ngspice || !m_ngspice->is_available()) {
            return;
        }

        // Convert circuit to SPICE netlist
        CircuitToSpiceConverter converter;
        CircuitToSpiceConverter::Config config;
        config.include_simulation_command = true;
        config.analysis_type = "op";
        config.simulation_duration = dt;
        config.simulation_time_step = dt;
        converter.set_config(config);

        auto result = converter.convert_with_wires(all_components, wires);

        if (!result.success) {
            return;
        }

        // Run simulation
        // Use a longer simulation duration to ensure ngspice produces meaningful results
        // The transient analysis needs enough time to settle
        double sim_duration = std::max(dt, 0.01);  // At least 10ms
        double sim_timestep = sim_duration / 10.0;  // 10 time steps

        const size_t net_hash = std::hash<std::string>{}(result.netlist);
        const bool is_op = (result.netlist.find(".op") != std::string::npos) || (result.netlist.find(".OP") != std::string::npos);
        // For operating point, results are purely a function of the netlist.
        if (is_op && net_hash == m_last_netlist_hash) {
            return;
        }
        m_last_netlist_hash = net_hash;

        spdlog::trace("[NGSPICE] Running simulation: duration={}s, timestep={}s", sim_duration, sim_timestep);
        auto sim_result = m_ngspice->simulate(result.netlist, sim_duration, sim_timestep);

        if (!sim_result.success) {
            spdlog::error("[NGSPICE] Simulation failed: {}", sim_result.error);
            return;
        }

        spdlog::trace("[NGSPICE] Simulation completed: {} time points, {} vectors",
                      sim_result.time_points.size(),
                      sim_result.time_points.empty() ? 0 : sim_result.time_points.back().second.size());

        // Update pin voltages from simulation results
        // ngspice returns voltages for each time point
        if (!sim_result.time_points.empty()) {
            const auto& last_point = sim_result.time_points.back();

            // Debug: log what we received
            spdlog::trace("[NGSPICE] Received {} node voltages", last_point.second.size());
            for (const auto& nv : last_point.second) {
                spdlog::trace("[NGSPICE] Node '{}' voltage: {}", nv.node, nv.voltage);
            }

            // Debug: log pin-to-node mapping
            spdlog::trace("[NGSPICE] Pin-to-node mapping size: {}", result.pin_to_node.size());
            for (const auto& [pin_id, node] : result.pin_to_node) {
                spdlog::trace("[NGSPICE] Pin '{}' -> Node '{}'", pin_id, node);
            }

            auto node_matches = [](const std::string& result_node, const std::string& pin_node) {
                if (result_node == pin_node) return true;
                return result_node == "V(" + pin_node + ")";
            };

            for (auto* comp : all_components) {
                for (CircuitPin* pin : comp->get_pins()) {
                    if (!pin) continue;
                    pin->current = 0.0f;

                    // Find the node for this pin
                    std::string pin_id = comp->id() + "." + pin->id;
                    auto it = result.pin_to_node.find(pin_id);

                    if (it != result.pin_to_node.end()) {
                        std::string node_name = it->second;

                        if (node_name == "0") {
                            pin->voltage = 0.0f;
                            spdlog::trace("[NGSPICE] Updated pin {} to 0V", pin_id);
                            continue;
                        }

                        // Find voltage for this node
                        bool found_voltage = false;
                        for (const auto& nv : last_point.second) {
                            if (node_matches(nv.node, node_name)) {
                                pin->voltage = static_cast<float>(nv.voltage);
                                spdlog::trace("[NGSPICE] Updated pin {} to {}V", pin_id, pin->voltage);
                                found_voltage = true;
                                break;
                            }
                        }
                        if (!found_voltage) {
                            pin->voltage = 0.0f;
                            spdlog::trace("[NGSPICE] Node '{}' for pin '{}' was not returned; reset pin to 0V", node_name, pin_id);
                        }
                    } else {
                        pin->voltage = 0.0f;
                        spdlog::trace("[NGSPICE] No node mapping for pin '{}'", pin_id);
                    }
                }
            }

            std::unordered_map<std::string, double> current_vectors;
            for (const auto& nv : last_point.second) {
                current_vectors[current_key(nv.node)] = nv.voltage;
            }

            auto get_current = [&current_vectors](const std::string& expression, double& value) {
                auto it = current_vectors.find(current_key(expression));
                if (it == current_vectors.end()) return false;
                value = it->second;
                return true;
            };

            for (auto* comp : all_components) {
                if (!comp) continue;

                const std::string id = comp->id();
                const std::string type(comp->type());
                double current = 0.0;

                if (type == "resistor" && get_current("@" + prefixed_spice_name('R', id) + "[i]", current)) {
                    set_two_pin_current(comp, current);
                } else if (type == "capacitor" && get_current("@" + prefixed_spice_name('C', id) + "[i]", current)) {
                    set_two_pin_current(comp, current);
                } else if (type == "inductor" && get_current("@" + prefixed_spice_name('L', id) + "[i]", current)) {
                    set_two_pin_current(comp, current);
                } else if ((type == "diode" || type == "zener_diode" || type == "led") &&
                           get_current("@" + prefixed_spice_name('D', id) + "[id]", current)) {
                    set_two_pin_current(comp, current);
                } else if (type == "dc_voltage" && get_current(to_lower(prefixed_spice_name('V', id)) + "#branch", current)) {
                    set_two_pin_current(comp, current);
                } else if (type == "h_bridge" && get_current(to_lower(prefixed_spice_name('V', id + "_vcc")) + "#branch", current)) {
                    set_pin_current(comp, "VCC", current);
                    set_pin_current(comp, "GND", -current);
                } else if (type == "buck_converter" && get_current("e.x" + id + ".e_buck#branch", current)) {
                    double input_current = 0.0;
                    double r_out_current = 0.0;
                    get_current("v.x" + id + ".v_in_sense#branch", input_current);
                    if (!get_current("@r.x" + id + ".r_out[i]", r_out_current)) {
                        r_out_current = current;
                    }
                    double output_current = -r_out_current;
                    set_pin_current(comp, "VIN", input_current);
                    set_pin_current(comp, "VOUT", output_current);
                    set_pin_current(comp, "GND", -(input_current + output_current));
                } else if (type == "boost_converter" && get_current("e.x" + id + ".e_boost#branch", current)) {
                    double input_current = 0.0;
                    double r_out_current = 0.0;
                    get_current("v.x" + id + ".v_in_sense#branch", input_current);
                    if (!get_current("@r.x" + id + ".r_out[i]", r_out_current)) {
                        r_out_current = current;
                    }
                    double output_current = -r_out_current;
                    set_pin_current(comp, "VIN", input_current);
                    set_pin_current(comp, "VOUT", output_current);
                    set_pin_current(comp, "GND", -(input_current + output_current));
                } else if (type == "bjt_npn" || type == "bjt_pnp") {
                    const auto q = prefixed_spice_name('Q', id);
                    if (get_current("@" + q + "[ib]", current)) set_pin_current(comp, "base", current);
                    if (get_current("@" + q + "[ic]", current)) set_pin_current(comp, "collector", current);
                    if (get_current("@" + q + "[ie]", current)) set_pin_current(comp, "emitter", current);
                } else if (type == "mosfet_n" || type == "mosfet_p") {
                    const auto m = prefixed_spice_name('M', id);
                    if (get_current("@" + m + "[ig]", current)) set_pin_current(comp, "gate", current);
                    if (get_current("@" + m + "[id]", current)) set_pin_current(comp, "drain", current);
                    if (get_current("@" + m + "[is]", current)) set_pin_current(comp, "source", current);
                }
            }
        } else {
            spdlog::warn("[NGSPICE] No time points in simulation result");
        }

        // Update components
        for (auto* comp : all_components) {
            comp->update(dt);
        }


    } catch (const std::exception& e) {
        (void)e; // Suppress unused warning
    }
#else
    // ngspice not available - cannot simulate
#endif
}

void CircuitSimulator::reset() {
    for (auto& [id, comp] : m_components_owned) {
        comp->reset();
    }
    for (auto& [id, comp] : m_components_external) {
        comp->reset();
    }
}

std::vector<CircuitComponent*> CircuitSimulator::get_all_components() {
    std::vector<CircuitComponent*> result;
    result.reserve(m_components_owned.size() + m_components_external.size());
    for (auto& [id, comp] : m_components_owned) {
        result.push_back(comp.get());
    }
    for (auto& [id, comp] : m_components_external) {
        result.push_back(comp);
    }
    return result;
}

// ============================================================================
// H-Bridge Implementation
// ============================================================================

HBridge::HBridge(float supply_voltage)
    : m_supply_voltage(supply_voltage)
{
    // Initialize pins
    m_vcc.voltage = supply_voltage;
    m_gnd.voltage = CircuitConstants::GROUND_VOLTAGE;
}

void HBridge::set_pwm_duty(float duty) {
    m_pwm_duty = std::max(CircuitConstants::NORMALIZED_MIN, std::min(CircuitConstants::NORMALIZED_MAX, duty));
}

void HBridge::reset() {
    m_pwm_phase = 0.0f;
    m_pwm_duty = 0.0f;
    m_direction = 0;
    m_output_voltage = 0.0f;

    m_in1.digital_state = false;
    m_in2.digital_state = false;
    m_en.digital_state = false;

    m_out1.voltage = 0.0f;
    m_out2.voltage = 0.0f;
}

} // namespace mechatron
