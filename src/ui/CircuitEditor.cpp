#include "CircuitEditor.hpp"
#include "core/SimulationOrchestrator.hpp"
#include "core/Registry.hpp"
#include "core/Component.hpp"
#include "core/Port.hpp"
#include "actuators/Actuator.hpp"
#include "sensors/Sensor.hpp"
#include "electronics/CircuitComponentAdapter.hpp"
#include "electronics/CircuitSimulator.hpp"
#include <imgui.h>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>

namespace mechatron {

// ============================================================================
// Type Mapping
// ============================================================================

std::pair<std::string, std::string> CircuitEditor::map_type_to_plugin(const std::string& type) {
    static const std::map<std::string, std::pair<std::string, std::string>> mapping = {
        {"resistor",          {"elec_passive",       "resistor"}},
        {"capacitor",         {"elec_passive",       "capacitor"}},
        {"inductor",          {"elec_passive",       "inductor"}},
        {"diode",             {"elec_semiconductor", "diode"}},
        {"zener_diode",       {"elec_semiconductor", "zener_diode"}},
        {"led",               {"elec_semiconductor", "led"}},
        {"bjt_npn",           {"elec_semiconductor", "bjt_npn"}},
        {"bjt_pnp",           {"elec_semiconductor", "bjt_pnp"}},
        {"mosfet_n",          {"elec_semiconductor", "mosfet_n"}},
        {"mosfet_p",          {"elec_semiconductor", "mosfet_p"}},
        {"h_bridge",          {"elec_power",         "h_bridge"}},
        {"buck_converter",    {"elec_power",         "buck_converter"}},
        {"boost_converter",   {"elec_power",         "boost_converter"}},
        {"motor_driver",      {"elec_power",         "motor_driver"}},
        {"dc_voltage",        {"elec_power",         "dc_voltage"}},
        {"ground",            {"elec_passive",       "ground"}},
        {"atmega328p",        {"soft_mcu_avr",       "atmega328p"}},
        {"atmega2560",        {"soft_mcu_avr",       "atmega2560"}},
        {"attiny85",          {"soft_mcu_avr",       "attiny85"}},
        {"limit_switch",      {"mech_machine_elements", "limit_switch"}},
        {"proximity_sensor",  {"mech_machine_elements", "proximity_sensor"}},
        {"rotary_encoder",    {"mech_machine_elements", "rotary_encoder"}},
        {"solenoid",          {"mech_machine_elements", "solenoid_actuator"}},
        {"dc_motor",          {"mech_machine_elements", "dc_motor"}},
        {"servo_motor",       {"mech_machine_elements", "servo_motor"}},
        {"pid_controller",    {"soft_control",       "pid_controller"}},
        {"pi_controller",     {"soft_control",       "pi_controller"}},
    };

    auto it = mapping.find(type);
    if (it != mapping.end()) return it->second;
    return {"", ""};
}

// ============================================================================
// Registry Sync
// ============================================================================

int CircuitEditor::find_node_by_id(const std::string& id) const {
    for (size_t i = 0; i < m_nodes.size(); ++i) {
        if (m_nodes[i].id == id) return static_cast<int>(i);
    }
    return -1;
}

// ============================================================================
// Pin Management
// ============================================================================

void CircuitEditor::refresh_node_pins(CircuitNode& node) {
    if (!m_orchestrator) return;

    Component* comp = m_orchestrator->registry().get(node.id);
    if (!comp) {
        node.pins.clear();
        node.pins_dirty = false;
        return;
    }

    node.pins.clear();
    auto ports = comp->get_ports();
    for (size_t i = 0; i < ports.size(); ++i) {
        PinInfo pin;
        pin.name = std::string(ports[i]->name());
        pin.port_index = static_cast<int>(i);
        pin.is_input = (ports[i]->direction() == PortDirection::Input ||
                        ports[i]->direction() == PortDirection::Bidirectional);
        node.pins.push_back(pin);
    }
    node.pins_dirty = false;
}

void CircuitEditor::sync_from_registry() {
    if (!m_orchestrator) return;

    // Add components from registry that aren't tracked yet
    m_orchestrator->registry().for_each([this](Component& comp) {
        int idx = find_node_by_id(comp.id());
        if (idx >= 0) return; // Already tracked

        std::string_view cat = comp.category();
        std::string_view comp_type = comp.component_type();

        bool is_circuit = (cat == "electronic" || cat == "passive" || cat == "semiconductor" ||
                          cat == "optoelectronic" || cat == "power" || cat == "mcu" ||
                          cat == "actuator" || cat == "sensor" || cat == "control" ||
                          cat == "estimator" || cat == "thermal" || cat == "magnetic");

        if (!is_circuit) return;

        CircuitNode node;
        node.id = comp.id();
        node.type = std::string(comp_type);
        node.plugin_name = std::string(comp.plugin_type());
        node.position[0] = 50.0f + static_cast<float>(m_nodes.size() % 5) * 80.0f;
        node.position[1] = 50.0f + static_cast<float>(m_nodes.size() / 5) * 60.0f;
        node.pins_dirty = true;
        m_nodes.push_back(node);
    });

    // Remove nodes no longer in registry
    for (int i = static_cast<int>(m_nodes.size()) - 1; i >= 0; --i) {
        if (!m_orchestrator->registry().get(m_nodes[i].id)) {
            std::string removed_id = m_nodes[i].id;
            auto it = std::remove_if(m_wires.begin(), m_wires.end(),
                [&removed_id](const WireConnection& w) {
                    return w.from_node == removed_id || w.to_node == removed_id;
                });
            m_wires.erase(it, m_wires.end());

            m_nodes.erase(m_nodes.begin() + i);
            if (m_selected_node_index == i) {
                m_selected_node_index = -1;
                m_selected_node_id.clear();
            } else if (m_selected_node_index > i) {
                m_selected_node_index--;
            }
        }
    }

    // Mark all nodes for pin refresh
    for (auto& node : m_nodes) {
        node.pins_dirty = true;
    }
}

void CircuitEditor::sync_selection_to_orchestrator() {
    if (!m_orchestrator) return;

    if (m_selected_node_index >= 0 && m_selected_node_index < static_cast<int>(m_nodes.size())) {
        const std::string& id = m_nodes[m_selected_node_index].id;
        if (m_orchestrator->get_selected_component() != id) {
            m_orchestrator->set_selected_component(id);
        }
    }
}

void CircuitEditor::sync_selection_from_orchestrator() {
    if (!m_orchestrator) return;

    const std::string& ext = m_orchestrator->get_selected_component();
    if (ext != m_selected_node_id) {
        if (ext.empty()) {
            m_selected_node_index = -1;
            m_selected_node_id.clear();
        } else {
            int idx = find_node_by_id(ext);
            if (idx >= 0) {
                m_selected_node_index = idx;
                m_selected_node_id = ext;
            }
        }
    }
}

// ============================================================================
// Node Management
// ============================================================================

void CircuitEditor::add_node(const std::string& type) {
    if (!m_orchestrator) return;

    auto [plugin_name, comp_type] = map_type_to_plugin(type);
    if (plugin_name.empty()) {
        spdlog::warn("Unknown circuit component type: {}", type);
        return;
    }

    m_next_node_num++;
    std::string id = comp_type + "_" + std::to_string(m_next_node_num);

    Component* comp = m_orchestrator->create_component(plugin_name, comp_type, id);
    if (!comp) {
        spdlog::warn("Failed to create circuit component: {}/{}", plugin_name, comp_type);
        return;
    }

    CircuitNode node;
    node.id = id;
    node.type = comp_type;
    node.plugin_name = plugin_name;
    node.position[0] = 50.0f + static_cast<float>(m_nodes.size() % 5) * 80.0f;
    node.position[1] = 50.0f + static_cast<float>(m_nodes.size() / 5) * 60.0f;
    node.pins_dirty = true;
    m_nodes.push_back(node);

    m_selected_node_index = static_cast<int>(m_nodes.size()) - 1;
    m_selected_node_id = id;
    m_selected_wire_index = -1;  // Deselect wire
    sync_selection_to_orchestrator();

    spdlog::info("Added circuit component: {} ({})", type, id);
}

void CircuitEditor::add_esc_template() {
    if (!m_orchestrator) return;

    spdlog::info("[ESC_TEMPLATE] Adding 3-phase ESC template...");

    // Save current node count for positioning
    size_t start_node_count = m_nodes.size();

    // Helper to add component with auto-positioning
    auto add_comp = [this, &start_node_count](const char* plugin, const char* type,
                                                const char* id, float x, float y) -> Component* {
        Component* comp = m_orchestrator->create_component(plugin, type, id);
        if (!comp) {
            spdlog::warn("[ESC_TEMPLATE] Failed to create: {}/{}", plugin, type);
            return nullptr;
        }

        CircuitNode node;
        node.id = id;
        node.type = type;
        node.plugin_name = plugin;
        node.position[0] = x;
        node.position[1] = y;
        node.pins_dirty = true;
        m_nodes.push_back(node);

        spdlog::info("[ESC_TEMPLATE] Added component: {} ({})", id, type);
        return comp;
    };

    float start_x = 100.0f;
    float start_y = 100.0f;

    // 1. Power Supply
    add_comp("elec_power", "dc_voltage", "BAT", start_x, start_y);
    add_comp("elec_passive", "ground", "GND", start_x, start_y + 80);

    // 2. 6 MOSFETs (3 half-bridges)
    float mosfet_y = start_y + 200;
    add_comp("elec_semiconductor", "mosfet_n", "AH", start_x, mosfet_y);      // High-side A
    add_comp("elec_semiconductor", "mosfet_n", "AL", start_x, mosfet_y + 80);  // Low-side A
    add_comp("elec_semiconductor", "mosfet_n", "BH", start_x + 150, mosfet_y);  // High-side B
    add_comp("elec_semiconductor", "mosfet_n", "BL", start_x + 150, mosfet_y + 80); // Low-side B
    add_comp("elec_semiconductor", "mosfet_n", "CH", start_x + 300, mosfet_y);  // High-side C
    add_comp("elec_semiconductor", "mosfet_n", "CL", start_x + 300, mosfet_y + 80); // Low-side C

    // 3. Gate Drive Sources
    float gate_y = start_y + 400;
    add_comp("elec_power", "dc_voltage", "G_AH", start_x, gate_y);
    add_comp("elec_power", "dc_voltage", "G_AL", start_x, gate_y + 60);
    add_comp("elec_power", "dc_voltage", "G_BH", start_x + 150, gate_y);
    add_comp("elec_power", "dc_voltage", "G_BL", start_x + 150, gate_y + 60);
    add_comp("elec_power", "dc_voltage", "G_CH", start_x + 300, gate_y);
    add_comp("elec_power", "dc_voltage", "G_CL", start_x + 300, gate_y + 60);

    // 4. Gate Resistors
    float rg_y = start_y + 550;
    add_comp("elec_passive", "resistor", "RgAH", start_x + 60, rg_y);
    add_comp("elec_passive", "resistor", "RgAL", start_x + 60, rg_y + 40);
    add_comp("elec_passive", "resistor", "RgBH", start_x + 210, rg_y);
    add_comp("elec_passive", "resistor", "RgBL", start_x + 210, rg_y + 40);
    add_comp("elec_passive", "resistor", "RgCH", start_x + 360, rg_y);
    add_comp("elec_passive", "resistor", "RgCL", start_x + 360, rg_y + 40);

    // 5. Motor Windings (Y-connected)
    float load_y = start_y + 680;
    add_comp("elec_passive", "resistor", "R_A", start_x + 30, load_y);
    add_comp("elec_passive", "resistor", "R_B", start_x + 150, load_y);
    add_comp("elec_passive", "resistor", "R_C", start_x + 270, load_y);

    spdlog::info("[ESC_TEMPLATE] All components added, creating wires...");

    // Helper to add wire with direction detection
    auto add_wire_auto = [this](const char* from_node, const char* from_pin,
                                 const char* to_node, const char* to_pin) {
        // Determine direction from port types
        Component* src_comp = m_orchestrator->registry().get(from_node);
        Component* tgt_comp = m_orchestrator->registry().get(to_node);
        if (!src_comp || !tgt_comp) return;

        Port* src_port = nullptr;
        Port* tgt_port = nullptr;

        for (auto* p : src_comp->get_ports()) {
            if (p->name() == from_pin) { src_port = p; break; }
        }
        for (auto* p : tgt_comp->get_ports()) {
            if (p->name() == to_pin) { tgt_port = p; break; }
        }

        if (!src_port || !tgt_port) return;

        PinDirection from_dir = (src_port->direction() == PortDirection::Output) ?
                                PinDirection::Output : PinDirection::Input;
        PinDirection to_dir = (tgt_port->direction() == PortDirection::Output) ?
                              PinDirection::Output : PinDirection::Input;

        add_wire(from_node, from_pin, to_node, to_pin, from_dir, to_dir);
    };

    // Power connections
    add_wire_auto("BAT", "GND", "GND", "GND");
    add_wire_auto("BAT", "V+", "AH", "drain");
    add_wire_auto("BAT", "V+", "BH", "drain");
    add_wire_auto("BAT", "V+", "CH", "drain");

    // Phase nodes (half-bridge outputs)
    add_wire_auto("AH", "source", "AL", "drain");  // Phase A
    add_wire_auto("BH", "source", "BL", "drain");  // Phase B
    add_wire_auto("CH", "source", "CL", "drain");  // Phase C

    // Low-side to GND
    add_wire_auto("AL", "source", "GND", "GND");
    add_wire_auto("BL", "source", "GND", "GND");
    add_wire_auto("CL", "source", "GND", "GND");

    // High-side gate connections (bootstrap)
    add_wire_auto("G_AH", "V+", "RgAH", "1");
    add_wire_auto("RgAH", "2", "AH", "gate");
    add_wire_auto("G_AH", "GND", "AH", "source");  // Bootstrap reference

    add_wire_auto("G_BH", "V+", "RgBH", "1");
    add_wire_auto("RgBH", "2", "BH", "gate");
    add_wire_auto("G_BH", "GND", "BH", "source");

    add_wire_auto("G_CH", "V+", "RgCH", "1");
    add_wire_auto("RgCH", "2", "CH", "gate");
    add_wire_auto("G_CH", "GND", "CH", "source");

    // Low-side gate connections (GND referenced)
    add_wire_auto("G_AL", "V+", "RgAL", "1");
    add_wire_auto("RgAL", "2", "AL", "gate");
    add_wire_auto("G_AL", "GND", "GND", "GND");

    add_wire_auto("G_BL", "V+", "RgBL", "1");
    add_wire_auto("RgBL", "2", "BL", "gate");
    add_wire_auto("G_BL", "GND", "GND", "GND");

    add_wire_auto("G_CL", "V+", "RgCL", "1");
    add_wire_auto("RgCL", "2", "CL", "gate");
    add_wire_auto("G_CL", "GND", "GND", "GND");

    // Motor winding connections (Y topology)
    add_wire_auto("AH", "source", "R_A", "1");  // Phase A to winding A
    add_wire_auto("BH", "source", "R_B", "1");  // Phase B to winding B
    add_wire_auto("CH", "source", "R_C", "1");  // Phase C to winding C

    // Star point (Y connection)
    add_wire_auto("R_A", "2", "R_B", "2");  // Winding A to Winding B
    add_wire_auto("R_B", "2", "R_C", "2");  // Winding B to Winding C

    spdlog::info("[ESC_TEMPLATE] ESC template added successfully! {} components, {} wires",
                 m_nodes.size() - start_node_count, m_wires.size());

    // Select the first MOSFET to show properties
    m_selected_node_index = static_cast<int>(start_node_count);  // Select AH
    m_selected_node_id = "AH";
    m_selected_wire_index = -1;
}

void CircuitEditor::remove_node(int index) {
    if (!m_orchestrator || index < 0 || index >= static_cast<int>(m_nodes.size())) return;

    std::string removed_id = m_nodes[index].id;
    spdlog::info("Removing circuit component: {}", removed_id);

    m_orchestrator->remove_component(removed_id);

    // Remove connected wires (also cleans up Port Connections)
    std::vector<int> wires_to_remove;
    for (int i = 0; i < static_cast<int>(m_wires.size()); ++i) {
        if (m_wires[i].from_node == removed_id || m_wires[i].to_node == removed_id) {
            remove_wire(i);
            wires_to_remove.push_back(i);
        }
    }

    m_nodes.erase(m_nodes.begin() + index);

    if (m_selected_node_index == index) {
        m_selected_node_index = -1;
        m_selected_node_id.clear();
        m_orchestrator->set_selected_component("");
    } else if (m_selected_node_index > index) {
        m_selected_node_index--;
    }
}

bool CircuitEditor::add_wire(const std::string& from_node, const std::string& from_pin,
                             const std::string& to_node, const std::string& to_pin,
                             PinDirection from_dir, PinDirection to_dir) {
    // Direction validation kaldırıldı - aynı renkteki pin'ler birbirine bağlanabilir

    // Use from_node/to_node as-is (no direction swapping)
    std::string src_node = from_node;
    std::string src_pin = from_pin;
    std::string tgt_node = to_node;
    std::string tgt_pin = to_pin;

    // Check for duplicate wire
    for (const auto& w : m_wires) {
        if (w.from_node == src_node && w.from_pin_name == src_pin &&
            w.to_node == tgt_node && w.to_pin_name == tgt_pin) {
            spdlog::warn("Duplicate wire: {}[{}] -> {}[{}]", src_node, src_pin, tgt_node, tgt_pin);
            return false;
        }
    }

    // Self-connection check
    if (src_node == tgt_node && src_pin == tgt_pin) {
        spdlog::warn("Cannot connect pin to itself");
        return false;
    }

    WireConnection wire;
    wire.uid = "wire_" + std::to_string(m_next_wire_uid++);
    wire.from_node = src_node;
    wire.from_pin_name = src_pin;
    wire.to_node = tgt_node;
    wire.to_pin_name = tgt_pin;
    m_wires.push_back(wire);

    // Create actual Port Connection in the orchestrator
    if (m_orchestrator) {
        Component* src_comp = m_orchestrator->registry().get(src_node);
        Component* tgt_comp = m_orchestrator->registry().get(tgt_node);
        if (src_comp && tgt_comp) {
            auto src_ports = src_comp->get_ports();
            auto tgt_ports = tgt_comp->get_ports();

            Port* src_port = nullptr;
            Port* tgt_port = nullptr;

            for (auto* p : src_ports) {
                if (p->name() == src_pin) { src_port = p; break; }
            }
            for (auto* p : tgt_ports) {
                if (p->name() == tgt_pin) { tgt_port = p; break; }
            }

            if (src_port && tgt_port) {
                m_orchestrator->connect(src_port, tgt_port, wire.uid);
                spdlog::info("Connected ports: {}[{}] -> {}[{}] (UID: {})", src_node, src_pin, tgt_node, tgt_pin, wire.uid);
            } else {
                spdlog::warn("Port not found for wire connection: {}[{}] -> {}[{}]", src_node, src_pin, tgt_node, tgt_pin);
            }
        }
    }

    spdlog::info("Added wire: {}[{}] -> {}[{}]", src_node, src_pin, tgt_node, tgt_pin);
    return true;
}

void CircuitEditor::remove_wire(int index) {
    spdlog::info("[REMOVE_WIRE] ENTER: index={}, m_wires.size()={}", index, m_wires.size());

    if (index < 0 || index >= static_cast<int>(m_wires.size())) {
        spdlog::warn("[REMOVE_WIRE] Index out of range: {} (size={})", index, m_wires.size());
        return;
    }

    const auto& wire = m_wires[index];
    spdlog::info("[REMOVE_WIRE] Wire: {}[{}] -> {}[{}]",
                 wire.from_node, wire.from_pin_name,
                 wire.to_node, wire.to_pin_name);

    // Remove the actual Port Connection from orchestrator
    if (m_orchestrator) {
        spdlog::info("[REMOVE_WIRE] Orchestrator OK");
        Component* src_comp = m_orchestrator->registry().get(wire.from_node);
        Component* tgt_comp = m_orchestrator->registry().get(wire.to_node);

        if (src_comp && tgt_comp) {
            spdlog::info("[REMOVE_WIRE] Components OK: src='{}' tgt='{}'", src_comp->id(), tgt_comp->id());
            auto src_ports = src_comp->get_ports();
            auto tgt_ports = tgt_comp->get_ports();
            spdlog::info("[REMOVE_WIRE] src_ports={}, tgt_ports={}", src_ports.size(), tgt_ports.size());

            Port* src_port = nullptr;
            Port* tgt_port = nullptr;

            for (auto* p : src_ports) {
                spdlog::info("[REMOVE_WIRE]   src port='{}'", p->name());
                if (p->name() == wire.from_pin_name) { src_port = p; break; }
            }
            for (auto* p : tgt_ports) {
                spdlog::info("[REMOVE_WIRE]   tgt port='{}'", p->name());
                if (p->name() == wire.to_pin_name) { tgt_port = p; break; }
            }

            if (src_port && tgt_port) {
                spdlog::info("[REMOVE_WIRE] Both ports found: src='{}' tgt='{}'", src_port->name(), tgt_port->name());
                spdlog::info("[REMOVE_WIRE] src_conns={}, tgt_conns={}",
                             src_port->connections().size(), tgt_port->connections().size());

                bool disconnected = false;
                for (auto* conn : src_port->connections()) {
                    spdlog::info("[REMOVE_WIRE]   conn: src='{}' tgt='{}'",
                                 conn->source ? conn->source->name() : "null",
                                 conn->target ? conn->target->name() : "null");
                    if ((conn->source == src_port && conn->target == tgt_port) ||
                        (conn->source == tgt_port && conn->target == src_port)) {
                        spdlog::info("[REMOVE_WIRE]   MATCH! Disconnecting...");
                        m_orchestrator->disconnect(conn);
                        disconnected = true;
                        spdlog::info("[REMOVE_WIRE] Disconnected ports: {}[{}] -> {}[{}]",
                                     wire.from_node, wire.from_pin_name,
                                     wire.to_node, wire.to_pin_name);
                        break;
                    }
                }
                if (!disconnected) {
                    spdlog::warn("[REMOVE_WIRE] No matching connection found!");
                }
            } else {
                spdlog::warn("[REMOVE_WIRE] Port not found: src={}, tgt={}",
                             src_port ? "found" : "MISSING",
                             tgt_port ? "found" : "MISSING");
            }
        } else {
            spdlog::warn("[REMOVE_WIRE] Component not found: src={}, tgt={}",
                         src_comp ? "found" : "MISSING",
                         tgt_comp ? "found" : "MISSING");
        }
    } else {
        spdlog::warn("[REMOVE_WIRE] No orchestrator!");
    }

    spdlog::info("[REMOVE_WIRE] Erasing wire from m_wires...");
    m_wires.erase(m_wires.begin() + index);
    spdlog::info("[REMOVE_WIRE] After erase, m_wires.size()={}", m_wires.size());
}

// ============================================================================
// Hit Detection
// ============================================================================

int CircuitEditor::get_node_at_position(float x, float y) {
    const float node_width = 100.0f;
    const float pin_spacing = 18.0f;
    const float label_height = 28.0f;

    for (size_t i = 0; i < m_nodes.size(); ++i) {
        const auto& node = m_nodes[i];
        int input_count = 0, output_count = 0;
        for (const auto& pin : node.pins) {
            if (pin.is_input) input_count++;
            else output_count++;
        }
        if (input_count == 0) input_count = 1;
        if (output_count == 0) output_count = 1;
        int max_side = std::max(input_count, output_count);
        float node_height = label_height + static_cast<float>(max_side) * pin_spacing + 10.0f;

        if (x >= node.position[0] && x <= node.position[0] + node_width &&
            y >= node.position[1] && y <= node.position[1] + node_height) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

const CircuitEditor::PinInfo* CircuitEditor::get_pin_at_position(const CircuitNode& node, float x, float y) {
    const float pin_spacing = 18.0f;
    const float label_height = 28.0f;
    const float node_width = 100.0f;
    const float hit_radius = 10.0f;

    // Only use REAL ports from the component - no fallback
    if (node.pins.empty()) return nullptr;

    int input_idx = 0;
    for (const auto& pin : node.pins) {
        if (!pin.is_input) continue;
        float pin_x = node.position[0];
        float pin_y = node.position[1] + label_height + static_cast<float>(input_idx) * pin_spacing;
        float dist = std::sqrt((x - pin_x) * (x - pin_x) + (y - pin_y) * (y - pin_y));
        if (dist < hit_radius) return &pin;
        input_idx++;
    }

    int output_idx = 0;
    for (const auto& pin : node.pins) {
        if (pin.is_input) continue;
        float pin_x = node.position[0] + node_width;
        float pin_y = node.position[1] + label_height + static_cast<float>(output_idx) * pin_spacing;
        float dist = std::sqrt((x - pin_x) * (x - pin_x) + (y - pin_y) * (y - pin_y));
        if (dist < hit_radius) return &pin;
        output_idx++;
    }

    return nullptr;
}

ImVec2 CircuitEditor::get_pin_canvas_pos(const CircuitNode& node, const std::string& pin_name, ImVec2 canvas_pos) {
    const float pin_spacing = 18.0f;
    const float label_height = 28.0f;
    const float node_width = 100.0f;

    // Find in actual pins only
    for (const auto& pin : node.pins) {
        if (pin.name != pin_name) continue;
        int same_dir_idx = 0;
        for (const auto& p2 : node.pins) {
            if (p2.is_input == pin.is_input && &p2 == &pin) break;
            if (p2.is_input == pin.is_input) same_dir_idx++;
        }
        float px = pin.is_input ? node.position[0] : node.position[0] + node_width;
        float py = node.position[1] + label_height + static_cast<float>(same_dir_idx) * pin_spacing;
        return ImVec2(canvas_pos.x + px, canvas_pos.y + py);
    }

    return ImVec2(0, 0);
}

// Wire hit detection (point-to-line-segment distance with margin)
int CircuitEditor::get_wire_at_position(float x, float y, ImVec2 canvas_pos) {
    const float wire_hit_margin = 12.0f;

    for (size_t i = 0; i < m_wires.size(); ++i) {
        const auto& wire = m_wires[i];

        // Find node positions
        ImVec2 from(0, 0), to(0, 0);
        bool found_from = false, found_to = false;

        for (const auto& node : m_nodes) {
            if (node.id == wire.from_node) {
                from = get_pin_canvas_pos(node, wire.from_pin_name, canvas_pos);
                found_from = (from.x != 0 || from.y != 0);
            }
            if (node.id == wire.to_node) {
                to = get_pin_canvas_pos(node, wire.to_pin_name, canvas_pos);
                found_to = (to.x != 0 || to.y != 0);
            }
        }

        if (!found_from || !found_to) {
            spdlog::info("[WIRE_HIT] Wire #{}: from={} (found={}), to={} (found={})",
                         i,
                         found_from ? "OK" : "MISSING",
                         found_from ? "OK" : "MISSING",
                         found_to ? "OK" : "MISSING",
                         found_to ? "OK" : "MISSING");
            continue;
        }

        spdlog::info("[WIRE_HIT] Wire #{}: from=({:.0f},{:.0f}) to=({:.0f},{:.0f}), click=({:.0f},{:.0f})",
                     i, from.x, from.y, to.x, to.y, x, y);

        // For Bezier curves, check multiple points along the curve
        float dx = to.x - from.x;
        float ctrl_offset = std::max(40.0f, std::abs(dx) * 0.4f);
        ImVec2 ctrl1 = { from.x + ctrl_offset, from.y };
        ImVec2 ctrl2 = { to.x - ctrl_offset, to.y };

        // Sample Bezier at 16 points
        for (int s = 0; s <= 16; s++) {
            float t = s / 16.0f;
            float u = 1.0f - t;
            float px = u*u*u*from.x + 3*u*u*t*ctrl1.x + 3*u*t*t*ctrl2.x + t*t*t*to.x;
            float py = u*u*u*from.y + 3*u*u*t*ctrl1.y + 3*u*t*t*ctrl2.y + t*t*t*to.y;

            float dist = std::sqrt((x - px) * (x - px) + (y - py) * (y - py));
            if (dist < wire_hit_margin) {
                spdlog::info("[WIRE_HIT] Wire #{} HIT at t={:.2f}, dist={:.1f}", i, t, dist);
                return static_cast<int>(i);
            }
        }
    }

    return -1;
}

// ============================================================================
// Main Render
// ============================================================================

void CircuitEditor::render(SimulationOrchestrator& orchestrator) {
    m_orchestrator = &orchestrator;

    // Sync with registry
    sync_from_registry();

    // Refresh pins for all nodes
    for (auto& node : m_nodes) {
        if (node.pins_dirty) {
            refresh_node_pins(node);
        }
    }

    // Sync selection from orchestrator
    sync_selection_from_orchestrator();

    ImGui::Begin("Circuit Editor");

    // Menu bar
    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("Edit")) {
            if (ImGui::MenuItem("Delete Selected", "Del")) {
                if (m_selected_wire_index >= 0) {
                    remove_wire(m_selected_wire_index);
                    m_selected_wire_index = -1;
                } else if (m_selected_node_index >= 0) {
                    remove_node(m_selected_node_index);
                }
            }
            if (ImGui::MenuItem("Clear All")) {
                // Remove all Port Connections
                for (const auto& wire : m_wires) {
                    Component* src_comp = m_orchestrator->registry().get(wire.from_node);
                    Component* tgt_comp = m_orchestrator->registry().get(wire.to_node);
                    if (src_comp && tgt_comp) {
                        auto src_ports = src_comp->get_ports();
                        auto tgt_ports = tgt_comp->get_ports();
                        for (auto* sp : src_ports) {
                            for (auto* conn : sp->connections()) {
                                for (auto* tp : tgt_ports) {
                                    if (conn->source == tp || conn->target == tp) {
                                        m_orchestrator->disconnect(conn);
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
                for (int i = static_cast<int>(m_nodes.size()) - 1; i >= 0; --i) {
                    m_orchestrator->remove_component(m_nodes[i].id);
                }
                m_nodes.clear();
                m_wires.clear();
                m_selected_node_index = -1;
                m_selected_wire_index = -1;
                m_selected_node_id.clear();
                m_orchestrator->set_selected_component("");
                spdlog::info("Circuit cleared");
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Component List", nullptr, &m_show_component_list);
            ImGui::MenuItem("Properties", nullptr, &m_show_properties);
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }

    // Layout
    if (m_show_component_list) {
        ImGui::BeginChild("ComponentPalette", ImVec2(150, 0), true);
        render_component_palette();
        ImGui::EndChild();
        ImGui::SameLine();
    }

    ImGui::BeginChild("CircuitCanvas", ImVec2(m_show_properties ? -200 : 0, 0), true);
    render_circuit_canvas();
    ImGui::EndChild();

    if (m_show_properties) {
        ImGui::SameLine();
        ImGui::BeginChild("PropertiesPanel", ImVec2(200, 0), true);
        render_properties_panel();
        ImGui::EndChild();
    }

    ImGui::End();
}

// ============================================================================
// Component Palette
// ============================================================================

void CircuitEditor::render_component_palette() {
    ImGui::Text("Components");
    ImGui::Separator();
    ImGui::InputText("Filter", m_component_filter, 64);
    ImGui::Spacing();

    auto add_btn = [this](const char* label, const char* type) {
        std::string filter(m_component_filter);
        if (!filter.empty() && std::string(label).find(filter) == std::string::npos) return;
        if (ImGui::Button(label, ImVec2(120, 0))) {
            add_node(type);
        }
    };

    ImGui::TextDisabled("Passive");
    add_btn("Resistor", "resistor");
    add_btn("Capacitor", "capacitor");
    add_btn("Inductor", "inductor");

    ImGui::Spacing();
    ImGui::TextDisabled("Semiconductors");
    add_btn("Diode", "diode");
    add_btn("Zener Diode", "zener_diode");
    add_btn("LED", "led");
    add_btn("BJT NPN", "bjt_npn");
    add_btn("BJT PNP", "bjt_pnp");
    add_btn("MOSFET N", "mosfet_n");
    add_btn("MOSFET P", "mosfet_p");

    ImGui::Spacing();
    ImGui::TextDisabled("Power");
    add_btn("H-Bridge", "h_bridge");
    add_btn("Buck Conv.", "buck_converter");
    add_btn("Boost Conv.", "boost_converter");
    add_btn("Motor Driver", "motor_driver");

    ImGui::Spacing();
    ImGui::TextDisabled("Sources");
    add_btn("DC Voltage", "dc_voltage");
    add_btn("Ground", "ground");

    ImGui::Spacing();
    ImGui::TextDisabled("MCU");
    add_btn("ATmega328P", "atmega328p");
    add_btn("ATmega2560", "atmega2560");
    add_btn("ATtiny85", "attiny85");

    ImGui::Spacing();
    ImGui::TextDisabled("Sensors");
    add_btn("Limit Switch", "limit_switch");
    add_btn("Proximity", "proximity_sensor");
    add_btn("Encoder", "rotary_encoder");

    ImGui::Spacing();
    ImGui::TextDisabled("Actuators");
    add_btn("DC Motor", "dc_motor");
    add_btn("Servo Motor", "servo_motor");
    add_btn("Solenoid", "solenoid");

    ImGui::Spacing();
    ImGui::TextDisabled("Control");
    add_btn("PID Ctrl", "pid_controller");
    add_btn("PI Ctrl", "pi_controller");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::TextDisabled("Templates");
    if (ImGui::Button("ESC 3-Phase", ImVec2(120, 0))) {
        add_esc_template();
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("3-Phase BLDC ESC\n6 MOSFETs with gate drives");
    }
}

// ============================================================================
// Circuit Canvas
// ============================================================================

void CircuitEditor::render_circuit_canvas() {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();

    // Invisible button for canvas interaction
    ImGui::InvisibleButton("canvas", canvas_size,
        ImGuiButtonFlags_MouseButtonLeft);

    bool is_hovered = ImGui::IsItemHovered();

    // Wire right-click detection (separate from invisible button)
    if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        ImVec2 mouse_pos = ImGui::GetMousePos();
        // Pass SCREEN coordinates to hit detection
        int wire_idx = get_wire_at_position(mouse_pos.x, mouse_pos.y, canvas_pos);
        if (wire_idx >= 0) {
            m_selected_wire_index = wire_idx;
            m_wire_context_menu_pending = true;
            m_wire_context_menu_pos = ImVec2(mouse_pos.x, mouse_pos.y);
            spdlog::info("[WIRE] Right-clicked wire #{}: {}[{}] -> {}[{}]",
                         wire_idx,
                         m_wires[wire_idx].from_node.c_str(),
                         m_wires[wire_idx].from_pin_name.c_str(),
                         m_wires[wire_idx].to_node.c_str(),
                         m_wires[wire_idx].to_pin_name.c_str());
        } else {
            m_wire_context_menu_pending = false;
        }
    } else if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        m_wire_context_menu_pending = false;
    }

    // Handle mouse input
    if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        ImVec2 mouse_pos = ImGui::GetMousePos();
        float rel_x = mouse_pos.x - canvas_pos.x;
        float rel_y = mouse_pos.y - canvas_pos.y;

        int clicked = get_node_at_position(rel_x, rel_y);
        if (clicked >= 0) {
            // Node click - select + drag
            m_selected_node_index = clicked;
            m_selected_node_id = m_nodes[clicked].id;
            m_selected_wire_index = -1;  // Deselect wire
            m_dragging_node = true;
            m_dragged_node_index = clicked;
            m_drag_offset[0] = rel_x - m_nodes[clicked].position[0];
            m_drag_offset[1] = rel_y - m_nodes[clicked].position[1];
            sync_selection_to_orchestrator();
        } else {
            // Check pin click
            bool pin_clicked = false;
            for (size_t i = 0; i < m_nodes.size(); ++i) {
                const PinInfo* pin = get_pin_at_position(m_nodes[i], rel_x, rel_y);
                if (pin) {
                    m_creating_wire = true;
                    m_wire_start_node = m_nodes[i].id;
                    m_wire_start_pin = pin->name;
                    m_wire_start_pin_dir = pin->is_input ? PinDirection::Input : PinDirection::Output;
                    pin_clicked = true;
                    break;
                }
            }
            if (!pin_clicked) {
                // Check wire click (selection)
                int wire_idx = get_wire_at_position(rel_x, rel_y, canvas_pos);
                if (wire_idx >= 0) {
                    m_selected_wire_index = wire_idx;
                    m_selected_node_index = -1;
                    m_selected_node_id.clear();
                } else {
                    // Deselect everything
                    m_selected_node_index = -1;
                    m_selected_node_id.clear();
                    m_selected_wire_index = -1;
                }
            }
            sync_selection_to_orchestrator();
        }
    }

    // Wire creation - release
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        if (m_creating_wire) {
            ImVec2 mouse_pos = ImGui::GetMousePos();
            float rel_x = mouse_pos.x - canvas_pos.x;
            float rel_y = mouse_pos.y - canvas_pos.y;

            for (const auto& node : m_nodes) {
                const PinInfo* pin = get_pin_at_position(node, rel_x, rel_y);
                if (pin && node.id != m_wire_start_node) {
                    PinDirection end_dir = pin->is_input ? PinDirection::Input : PinDirection::Output;
                    add_wire(m_wire_start_node, m_wire_start_pin, node.id, pin->name,
                             m_wire_start_pin_dir, end_dir);
                    break;
                }
            }
        }
        m_dragging_node = false;
        m_creating_wire = false;
    }

    // Drag node
    if (m_dragging_node && m_dragged_node_index >= 0 && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        ImVec2 mouse_pos = ImGui::GetMousePos();
        m_nodes[m_dragged_node_index].position[0] = mouse_pos.x - canvas_pos.x - m_drag_offset[0];
        m_nodes[m_dragged_node_index].position[1] = mouse_pos.y - canvas_pos.y - m_drag_offset[1];
    }

    // Delete key
    if (is_hovered && ImGui::IsKeyPressed(ImGuiKey_Delete)) {
        if (m_selected_wire_index >= 0) {
            remove_wire(m_selected_wire_index);
            m_selected_wire_index = -1;
        } else if (m_selected_node_index >= 0) {
            remove_node(m_selected_node_index);
        }
    }

    // ---- Drawing ----

    // Grid
    const float grid_size = 20.0f;
    ImU32 grid_color = ImGui::GetColorU32(ImVec4(0.3f, 0.3f, 0.3f, 0.3f));
    for (float x = 0; x < canvas_size.x; x += grid_size) {
        draw_list->AddLine(ImVec2(canvas_pos.x + x, canvas_pos.y),
                          ImVec2(canvas_pos.x + x, canvas_pos.y + canvas_size.y), grid_color);
    }
    for (float y = 0; y < canvas_size.y; y += grid_size) {
        draw_list->AddLine(ImVec2(canvas_pos.x, canvas_pos.y + y),
                          ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + y), grid_color);
    }

    // Junction dots (where wires share the same pin)
    auto draw_junction_dot = [draw_list, canvas_pos](float x, float y, int wire_count) {
        if (wire_count < 2) return;
        ImU32 dot_color = ImGui::GetColorU32(ImVec4(0.9f, 0.9f, 0.1f, 1.0f));
        draw_list->AddCircleFilled(ImVec2(x, y), 3.0f, dot_color);
        draw_list->AddCircle(ImVec2(x, y), 3.0f, IM_COL32(255, 255, 255, 200), 12, 1.5f);
    };

    // Track pin connection counts for junction dots
    std::map<std::pair<std::string, std::string>, int> pin_wire_count;
    for (const auto& wire : m_wires) {
        pin_wire_count[{wire.from_node, wire.from_pin_name}]++;
        pin_wire_count[{wire.to_node, wire.to_pin_name}]++;
    }

    // Wires - Bezier curves
    ImU32 wire_color = ImGui::GetColorU32(ImVec4(0.8f, 0.8f, 0.2f, 1.0f));
    ImU32 wire_selected_color = ImGui::GetColorU32(ImVec4(1.0f, 0.5f, 0.3f, 1.0f));

    for (size_t i = 0; i < m_wires.size(); ++i) {
        const auto& wire = m_wires[i];
        bool is_selected = (static_cast<int>(i) == m_selected_wire_index);
        ImU32 color = is_selected ? wire_selected_color : wire_color;
        float thickness = is_selected ? 3.0f : 2.0f;

        ImVec2 from_pos(0, 0), to_pos(0, 0);
        bool from_found = false, to_found = false;

        for (const auto& node : m_nodes) {
            if (node.id == wire.from_node) {
                from_pos = get_pin_canvas_pos(node, wire.from_pin_name, canvas_pos);
                if (from_pos.x != 0 || from_pos.y != 0) from_found = true;
            }
            if (node.id == wire.to_node) {
                to_pos = get_pin_canvas_pos(node, wire.to_pin_name, canvas_pos);
                if (to_pos.x != 0 || to_pos.y != 0) to_found = true;
            }
            if (from_found && to_found) break;
        }

        if (from_found && to_found) {
            // Bezier curve: control points create a smooth S-curve
            float dx = std::abs(to_pos.x - from_pos.x);
            float ctrl_offset = std::max(40.0f, dx * 0.4f);
            ImVec2 ctrl1 = { from_pos.x + ctrl_offset, from_pos.y };
            ImVec2 ctrl2 = { to_pos.x - ctrl_offset, to_pos.y };

            draw_list->AddBezierCubic(from_pos, ctrl1, ctrl2, to_pos, color, thickness, 32);

            // Junction dot at from pin
            auto it_from = pin_wire_count.find({wire.from_node, wire.from_pin_name});
            if (it_from != pin_wire_count.end() && it_from->second >= 2) {
                draw_junction_dot(from_pos.x, from_pos.y, it_from->second);
            }

            // Junction dot at to pin
            auto it_to = pin_wire_count.find({wire.to_node, wire.to_pin_name});
            if (it_to != pin_wire_count.end() && it_to->second >= 2) {
                draw_junction_dot(to_pos.x, to_pos.y, it_to->second);
            }
        }
    }

    // Wire being created - Bezier temp line
    if (m_creating_wire) {
        ImVec2 mouse_pos = ImGui::GetMousePos();
        ImVec2 start(0, 0);
        bool found = false;

        for (const auto& node : m_nodes) {
            if (node.id == m_wire_start_node) {
                start = get_pin_canvas_pos(node, m_wire_start_pin, canvas_pos);
                if (start.x != 0 || start.y != 0) found = true;
                break;
            }
        }

        if (found) {
            float dx = std::abs(mouse_pos.x - start.x);
            float ctrl_offset = std::max(40.0f, dx * 0.4f);
            ImVec2 ctrl1 = { start.x + ctrl_offset, start.y };
            ImVec2 ctrl2 = { mouse_pos.x - ctrl_offset, mouse_pos.y };

            draw_list->AddBezierCubic(start, ctrl1, ctrl2, mouse_pos, wire_color, 2.0f, 32);
        }
    }

    // Nodes
    const float node_width = 100.0f;
    const float pin_spacing = 18.0f;
    const float pin_radius = 5.0f;
    const float label_height = 28.0f;

    for (size_t i = 0; i < m_nodes.size(); ++i) {
        const auto& node = m_nodes[i];
        int input_count = 0, output_count = 0;
        for (const auto& pin : node.pins) {
            if (pin.is_input) input_count++;
            else output_count++;
        }
        if (input_count == 0) input_count = 1;
        if (output_count == 0) output_count = 1;

        int max_side = std::max(input_count, output_count);
        float node_height = label_height + static_cast<float>(max_side) * pin_spacing + 10.0f;

        ImVec2 node_pos(canvas_pos.x + node.position[0], canvas_pos.y + node.position[1]);

        bool is_selected = (static_cast<int>(i) == m_selected_node_index);
        ImU32 bg_color = is_selected ?
            ImGui::GetColorU32(ImVec4(0.3f, 0.5f, 0.7f, 0.8f)) :
            ImGui::GetColorU32(ImVec4(0.2f, 0.2f, 0.2f, 0.9f));

        draw_list->AddRectFilled(node_pos, ImVec2(node_pos.x + node_width, node_pos.y + node_height), bg_color, 4.0f);
        draw_list->AddRect(node_pos, ImVec2(node_pos.x + node_width, node_pos.y + node_height),
                          is_selected ? IM_COL32(100, 200, 255, 255) : IM_COL32(255, 255, 255, 255), 4.0f, 0, is_selected ? 2.5f : 1.5f);

        // Label
        draw_list->AddText(ImVec2(node_pos.x + 5, node_pos.y + 5),
                          IM_COL32(255, 255, 255, 255), node.type.c_str());

        // Draw input pins (left side)
        int input_idx = 0;
        for (const auto& pin : node.pins) {
            if (!pin.is_input) continue;

            float pin_x = node_pos.x;
            float pin_y = node_pos.y + label_height + static_cast<float>(input_idx) * pin_spacing;

            bool is_hovered_pin = false;
            ImVec2 mp = ImGui::GetMousePos();
            float rmx = mp.x - canvas_pos.x;
            float rmy = mp.y - canvas_pos.y;
            float dist = std::sqrt((rmx - pin_x) * (rmx - pin_x) + (rmy - pin_y) * (rmy - pin_y));
            is_hovered_pin = (dist < 8.0f);

            ImU32 pin_color = is_hovered_pin ?
                IM_COL32(255, 255, 150, 255) :
                IM_COL32(255, 200, 100, 255);
            draw_list->AddCircleFilled(ImVec2(pin_x, pin_y), pin_radius, pin_color);
            draw_list->AddCircle(ImVec2(pin_x, pin_y), pin_radius, IM_COL32(255, 255, 255, 200), 12, 1.0f);

            // Pin label
            draw_list->AddText(ImVec2(pin_x + 8, pin_y - 5),
                              IM_COL32(220, 220, 220, 255), pin.name.c_str());

            input_idx++;
        }

        // Draw output pins (right side)
        int output_idx = 0;
        for (const auto& pin : node.pins) {
            if (pin.is_input) continue;

            float pin_x = node_pos.x + node_width;
            float pin_y = node_pos.y + label_height + static_cast<float>(output_idx) * pin_spacing;

            bool is_hovered_pin = false;
            ImVec2 mp = ImGui::GetMousePos();
            float rmx = mp.x - canvas_pos.x;
            float rmy = mp.y - canvas_pos.y;
            float dist = std::sqrt((rmx - pin_x) * (rmx - pin_x) + (rmy - pin_y) * (rmy - pin_y));
            is_hovered_pin = (dist < 8.0f);

            ImU32 pin_color = is_hovered_pin ?
                IM_COL32(150, 255, 255, 255) :
                IM_COL32(100, 200, 255, 255);
            draw_list->AddCircleFilled(ImVec2(pin_x, pin_y), pin_radius, pin_color);
            draw_list->AddCircle(ImVec2(pin_x, pin_y), pin_radius, IM_COL32(255, 255, 255, 200), 12, 1.0f);

            // Pin label (right-aligned)
            float text_width = ImGui::CalcTextSize(pin.name.c_str()).x;
            draw_list->AddText(ImVec2(pin_x - 8 - text_width, pin_y - 5),
                              IM_COL32(220, 220, 220, 255), pin.name.c_str());

            output_idx++;
        }
    }

    // Status bar
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + canvas_size.y - 20);
    ImGui::TextDisabled("Nodes: %d  Wires: %d%s",
                        static_cast<int>(m_nodes.size()),
                        static_cast<int>(m_wires.size()),
                        m_selected_wire_index >= 0 ? "  [Wire selected - right-click to delete]" : "");

    // Wire context popup (render inside canvas so it appears at correct position)
    if (m_wire_context_menu_pending && m_selected_wire_index >= 0 && m_selected_wire_index < static_cast<int>(m_wires.size())) {
        ImGui::SetNextWindowPos(m_wire_context_menu_pos);
        ImGui::OpenPopup("WireDeletePopup");
        m_wire_context_menu_pending = false;
    }

    if (ImGui::BeginPopup("WireDeletePopup")) {
        if (m_selected_wire_index >= 0 && m_selected_wire_index < static_cast<int>(m_wires.size())) {
            const auto& wire = m_wires[m_selected_wire_index];

            spdlog::info("[WIRE_POPUP] Showing popup for wire #{}: {}[{}] -> {}[{}], UID='{}'",
                         m_selected_wire_index,
                         wire.from_node.c_str(), wire.from_pin_name.c_str(),
                         wire.to_node.c_str(), wire.to_pin_name.c_str(),
                         wire.uid.c_str());

            ImGui::Text("Wire Information");
            ImGui::Separator();

            // Show UID
            ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), "UID: %s",
                wire.uid.empty() ? "(EMPTY)" : wire.uid.c_str());

            ImGui::Spacing();
            ImGui::Text("Connection:");
            ImGui::Text("%s[%s] -> %s[%s]",
                        wire.from_node.c_str(), wire.from_pin_name.c_str(),
                        wire.to_node.c_str(), wire.to_pin_name.c_str());

            // Show current flow information
            if (m_orchestrator) {
                ImGui::Separator();
                ImGui::TextColored(ImVec4(0.6f, 0.6f, 1.0f, 1.0f), "Current Flow");

                // Get source and target components
                Component* src_comp = m_orchestrator->registry().get(wire.from_node);
                Component* tgt_comp = m_orchestrator->registry().get(wire.to_node);

                if (src_comp && tgt_comp) {
                    // Get ports
                    auto src_ports = src_comp->get_ports();
                    auto tgt_ports = tgt_comp->get_ports();

                    Port* src_port = nullptr;
                    Port* tgt_port = nullptr;

                    for (auto* p : src_ports) {
                        if (p->name() == wire.from_pin_name) { src_port = p; break; }
                    }
                    for (auto* p : tgt_ports) {
                        if (p->name() == wire.to_pin_name) { tgt_port = p; break; }
                    }

                    // Show voltages
                    float src_voltage = 0.0f, tgt_voltage = 0.0f;
                    if (src_port) {
                        if (const float* f = src_port->get_value<float>()) {
                            src_voltage = *f;
                        }
                    }
                    if (tgt_port) {
                        if (const float* f = tgt_port->get_value<float>()) {
                            tgt_voltage = *f;
                        }
                    }

                    ImGui::Text("Voltage: %.2f V -> %.2f V", src_voltage, tgt_voltage);

                    // Debug log
                    spdlog::info("[WIRE_CONTEXT] Wire: {}[{}] -> {}[{}], V: {:.2f}V -> {:.2f}V",
                                 wire.from_node, wire.from_pin_name, wire.to_node, wire.to_pin_name,
                                 src_voltage, tgt_voltage);

                    // Get current from circuit pins if available
                    // Try source component first, then target component
                    float current_a = 0.0f;
                    bool current_found = false;

                    #define GET_CURRENT(AdapterType, comp_ptr, pin_name) \
                        if (auto* adapted = dynamic_cast<AdapterType*>(comp_ptr)) { \
                            auto pins = adapted->circuit_component()->get_pins(); \
                            for (auto* pin : pins) { \
                                if (pin && pin->id == pin_name) { \
                                    current_a = pin->current; \
                                    current_found = true; \
                                    break; \
                                } \
                            } \
                        }

                    // Try to get current from source component
                    std::string_view src_cat = src_comp->category();
                    std::string_view src_ctype = src_comp->component_type();

                    if (src_cat == "passive" || src_cat == "semiconductor" || src_cat == "power") {
                        if (src_ctype == "resistor") { GET_CURRENT(ResistorComponent, src_comp, wire.from_pin_name); }
                        else if (src_ctype == "capacitor") { GET_CURRENT(CapacitorComponent, src_comp, wire.from_pin_name); }
                        else if (src_ctype == "inductor") { GET_CURRENT(InductorComponent, src_comp, wire.from_pin_name); }
                        else if (src_ctype == "diode") { GET_CURRENT(DiodeComponent, src_comp, wire.from_pin_name); }
                        else if (src_ctype == "led") { GET_CURRENT(LEDComponent, src_comp, wire.from_pin_name); }
                        else if (src_ctype == "zener_diode") { GET_CURRENT(ZenerDiodeComponent, src_comp, wire.from_pin_name); }
                        else if (src_ctype == "bjt_npn" || src_ctype == "bjt_pnp") { GET_CURRENT(BJTComponent, src_comp, wire.from_pin_name); }
                        else if (src_ctype == "mosfet_n" || src_ctype == "mosfet_p") { GET_CURRENT(MOSFETComponent, src_comp, wire.from_pin_name); }
                        else if (src_ctype == "dc_voltage") { GET_CURRENT(DCVoltageComponent, src_comp, wire.from_pin_name); }
                        else if (src_ctype == "ground") { GET_CURRENT(GroundComponent, src_comp, wire.from_pin_name); }
                    }

                    // If not found in source, try target component
                    if (!current_found && tgt_comp) {
                        std::string_view tgt_cat = tgt_comp->category();
                        std::string_view tgt_ctype = tgt_comp->component_type();

                        if (tgt_cat == "passive" || tgt_cat == "semiconductor" || tgt_cat == "power") {
                            if (tgt_ctype == "resistor") { GET_CURRENT(ResistorComponent, tgt_comp, wire.to_pin_name); }
                            else if (tgt_ctype == "capacitor") { GET_CURRENT(CapacitorComponent, tgt_comp, wire.to_pin_name); }
                            else if (tgt_ctype == "inductor") { GET_CURRENT(InductorComponent, tgt_comp, wire.to_pin_name); }
                            else if (tgt_ctype == "diode") { GET_CURRENT(DiodeComponent, tgt_comp, wire.to_pin_name); }
                            else if (tgt_ctype == "led") { GET_CURRENT(LEDComponent, tgt_comp, wire.to_pin_name); }
                            else if (tgt_ctype == "zener_diode") { GET_CURRENT(ZenerDiodeComponent, tgt_comp, wire.to_pin_name); }
                            else if (tgt_ctype == "bjt_npn" || tgt_ctype == "bjt_pnp") { GET_CURRENT(BJTComponent, tgt_comp, wire.to_pin_name); }
                            else if (tgt_ctype == "mosfet_n" || tgt_ctype == "mosfet_p") { GET_CURRENT(MOSFETComponent, tgt_comp, wire.to_pin_name); }
                            else if (tgt_ctype == "dc_voltage") { GET_CURRENT(DCVoltageComponent, tgt_comp, wire.to_pin_name); }
                            else if (tgt_ctype == "ground") { GET_CURRENT(GroundComponent, tgt_comp, wire.to_pin_name); }
                        }
                    }

                    #undef GET_CURRENT

                    // Debug log
                    spdlog::info("[WIRE_CONTEXT] current_found={} current_a={}",
                                 current_found, current_a);

                    // Format current
                    if (current_found) {
                        std::string current_str;
                        if (std::abs(current_a) < 1e-6f) {
                            current_str = "0 A";
                        } else if (std::abs(current_a) < 1e-3f) {
                            current_str = std::to_string(current_a * 1e6f) + " µA";
                        } else if (std::abs(current_a) < 1.0f) {
                            current_str = std::to_string(current_a * 1e3f) + " mA";
                        } else {
                            current_str = std::to_string(current_a) + " A";
                        }
                        ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.5f, 1.0f), "Current: %s", current_str.c_str());
                    } else {
                        ImGui::TextDisabled("Current: N/A");
                    }
                }
            }

            ImGui::Separator();
            ImGui::Spacing();

            if (ImGui::Button("Delete", ImVec2(100, 0))) {
                int idx = m_selected_wire_index;
                spdlog::info("[WIRE] Delete button pressed! Index={}", idx);
                remove_wire(idx);
                m_selected_wire_index = -1;
                ImGui::CloseCurrentPopup();
                spdlog::info("[WIRE] After erase, m_wires.size()={}", m_wires.size());
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(100, 0))) {
                ImGui::CloseCurrentPopup();
            }
        }
        ImGui::EndPopup();
    }
}

// ============================================================================
// Properties Panel
// ============================================================================

// Helper macros for safe type casting
#define TRY_CAST(type) dynamic_cast<type*>(comp)

void CircuitEditor::render_properties_panel() {
    ImGui::Text("Properties");
    ImGui::Separator();

    // Wire properties
    if (m_selected_wire_index >= 0 && m_selected_wire_index < static_cast<int>(m_wires.size())) {
        const auto& wire = m_wires[m_selected_wire_index];
        ImGui::Text("Wire #%d", m_selected_wire_index + 1);
        ImGui::Separator();

        // Show UID (with debug info if empty)
        if (wire.uid.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "UID: (EMPTY - BUG!)");
            spdlog::warn("[WIRE] Selected wire #{} has empty UID!", m_selected_wire_index);
        } else {
            ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), "UID: %s", wire.uid.c_str());
        }

        ImGui::Spacing();
        ImGui::Text("From: %s[%s]", wire.from_node.c_str(), wire.from_pin_name.c_str());
        ImGui::Text("To:   %s[%s]", wire.to_node.c_str(), wire.to_pin_name.c_str());
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Press Delete to remove wire");
        return;
    }

    if (m_selected_node_index < 0 || m_selected_node_index >= static_cast<int>(m_nodes.size())) {
        ImGui::TextDisabled("No component selected");
        return;
    }

    const auto& node = m_nodes[m_selected_node_index];

    ImGui::Text("ID: %s", node.id.c_str());
    ImGui::Text("Type: %s", node.type.c_str());
    ImGui::Text("Plugin: %s", node.plugin_name.c_str());
    ImGui::Spacing();

    // Show Component properties from Registry
    if (m_orchestrator) {
        Component* comp = m_orchestrator->registry().get(node.id);
        if (comp) {
            // 3D Transform
            ImGui::TextColored(ImVec4(0.6f, 0.6f, 1.0f, 1.0f), "3D Transform");
            auto& t = comp->transform();
            ImGui::Indent();
            ImGui::DragFloat3("Position##3d", &t.position.x, 0.1f);
            ImGui::DragFloat3("Scale##3d", &t.scale.x, 0.01f, 0.01f, 100.0f);
            ImGui::Unindent();
            ImGui::Separator();

            // Runtime info based on component type
            auto cat = comp->category();
            auto ctype = comp->component_type();

            // ---- ACTUATORS ----
            if (cat == "actuator") {
                if (auto* motor = TRY_CAST(DCMotor)) {
                    float input_voltage = motor->get_input_voltage();

                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "Runtime Data");
                    ImGui::Indent();
                    ImGui::Text("RPM: %.1f", motor->get_rpm());
                    ImGui::Text("Angular Velocity: %.2f rad/s", motor->get_angular_velocity());
                    ImGui::Text("Torque: %.4f Nm", motor->get_torque());
                    ImGui::Text("Input Voltage: %.2f V", input_voltage);

                    // Voltage indicator bar
                    float voltage_pct = std::min(input_voltage / 48.0f, 1.0f);
                    ImGui::ProgressBar(voltage_pct, ImVec2(-1, 6));

                    ImGui::Text("Enabled: %s", motor->is_enabled() ? "Yes" : "No");
                    ImGui::Separator();
                    ImGui::TextColored(ImVec4(0.6f, 0.6f, 1.0f, 1.0f), "Parameters");
                    ImGui::Text("Voltage Rating: %.1f V", 12.0f);
                    ImGui::Text("No-Load RPM: %.0f", 3000.0f);
                    ImGui::Text("Stall Torque: %.2f Nm", 0.5f);
                    ImGui::Unindent();
                } else if (auto* sol = TRY_CAST(SolenoidActuator)) {
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "Runtime Data");
                    ImGui::Indent();
                    ImGui::Text("Position: %.2f", sol->get_position());
                    ImGui::Text("Enabled: %s", sol->is_enabled() ? "Yes" : "No");
                    ImGui::Text("Input: %.2f", sol->get_input());
                    ImGui::Separator();
                    ImGui::TextColored(ImVec4(0.6f, 0.6f, 1.0f, 1.0f), "Parameters");
                    ImGui::Text("Resistance: %.1f Ohm", 12.0f);
                    ImGui::Text("Inductance: %.2f H", 0.5f);
                    ImGui::Text("Stroke: %.1f mm", 10.0f);
                    ImGui::Unindent();
                } else if (auto* servo = TRY_CAST(ServoMotor)) {
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "Runtime Data");
                    ImGui::Indent();
                    ImGui::Text("Current Angle: %.1f deg", servo->get_current_angle());
                    ImGui::Text("Enabled: %s", servo->is_enabled() ? "Yes" : "No");
                    ImGui::Separator();
                    ImGui::TextColored(ImVec4(0.6f, 0.6f, 1.0f, 1.0f), "Parameters");
                    ImGui::Text("Min Angle: %.0f deg", 0.0f);
                    ImGui::Text("Max Angle: %.0f deg", 180.0f);
                    ImGui::Text("Max Speed: %.0f deg/s", 300.0f);
                    ImGui::Unindent();
                }
            }
            // ---- SENSORS ----
            else if (cat == "sensor") {
                if (auto* prox = TRY_CAST(ProximitySensor)) {
                    ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.5f, 1.0f), "Runtime Data");
                    ImGui::Indent();
                    ImGui::Text("Distance: %.3f m", prox->get_distance());
                    ImGui::Text("Voltage Out: %.2f V", prox->read());
                    ImGui::Text("Max Range: %.1f m", 5.0f);
                    ImGui::Text("FOV: %.0f deg", 30.0f);
                    ImGui::Unindent();
                } else if (auto* ls = TRY_CAST(LimitSwitch)) {
                    ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.5f, 1.0f), "Runtime Data");
                    ImGui::Indent();
                    ImGui::Text("Triggered: %s", ls->is_triggered() ? "Yes" : "No");
                    ImGui::Text("Output: %.1f V", ls->read());
                    ImGui::Unindent();
                } else if (auto* enc = TRY_CAST(RotaryEncoder)) {
                    ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.5f, 1.0f), "Runtime Data");
                    ImGui::Indent();
                    ImGui::Text("Angle: %.1f deg", enc->get_angle_degrees());
                    ImGui::Text("Angle: %.3f rad", enc->get_angle_radians());
                    ImGui::Text("Angular Velocity: %.2f rad/s", enc->get_angular_velocity());
                    ImGui::Text("Pulses: %d", enc->get_pulse_count());
                    ImGui::Text("Resolution: %d PPR", 360);
                    ImGui::Unindent();
                } else if (auto* pot = TRY_CAST(Potentiometer)) {
                    ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.5f, 1.0f), "Runtime Data");
                    ImGui::Indent();
                    ImGui::Text("Angle: %.1f deg", pot->get_angle());
                    ImGui::Text("Voltage Out: %.2f V", pot->read());
                    ImGui::Unindent();
                }
            }
            // ---- PASSIVE (Resistor, Capacitor, Inductor) ----
            else if (cat == "passive") {
                if (ctype == "resistor") {
                    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "Parameters");
                    ImGui::Indent();
                    ImGui::Text("Resistance: 1000 Ohm");
                    ImGui::Text("Type: Passive");
                    ImGui::Unindent();
                } else if (ctype == "capacitor") {
                    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "Parameters");
                    ImGui::Indent();
                    ImGui::Text("Capacitance: 1 uF");
                    ImGui::Text("Type: Passive");
                    ImGui::Unindent();
                } else if (ctype == "inductor") {
                    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "Parameters");
                    ImGui::Indent();
                    ImGui::Text("Inductance: 1 mH");
                    ImGui::Text("Type: Passive");
                    ImGui::Unindent();
                } else if (ctype == "ground") {
                    ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.2f, 1.0f), "Ground Reference");
                    ImGui::Indent();
                    ImGui::Text("Voltage: 0.00 V");
                    ImGui::Text("Type: Ground");
                    ImGui::Unindent();
                }
            }
            // ---- SEMICONDUCTOR ----
            else if (cat == "semiconductor") {
                if (ctype == "diode" || ctype == "led") {
                    ImGui::TextColored(ImVec4(0.6f, 1.0f, 0.6f, 1.0f), "Runtime Data");
                    ImGui::Indent();
                    ImGui::Text("Type: %s", ctype == "led" ? "LED" : "Diode");
                    ImGui::Text("Forward Voltage: 0.7 V");
                    ImGui::Text("Type: Semiconductor");
                    ImGui::Unindent();
                } else if (ctype == "bjt_npn" || ctype == "bjt_pnp") {
                    ImGui::TextColored(ImVec4(0.6f, 1.0f, 0.6f, 1.0f), "Runtime Data");
                    ImGui::Indent();
                    ImGui::Text("Type: %s", ctype == "bjt_npn" ? "NPN" : "PNP");
                    ImGui::Text("Beta: 100");
                    ImGui::Text("Type: BJT Transistor");
                    ImGui::Unindent();
                } else if (ctype == "mosfet_n" || ctype == "mosfet_p") {
                    ImGui::TextColored(ImVec4(0.6f, 1.0f, 0.6f, 1.0f), "Runtime Data");
                    ImGui::Indent();
                    ImGui::Text("Type: %s", ctype == "mosfet_n" ? "N-Channel" : "P-Channel");
                    ImGui::Text("Threshold: 2.0 V");
                    ImGui::Text("Type: MOSFET");
                    ImGui::Unindent();
                } else if (ctype == "zener_diode") {
                    ImGui::TextColored(ImVec4(0.6f, 1.0f, 0.6f, 1.0f), "Runtime Data");
                    ImGui::Indent();
                    ImGui::Text("Zener Voltage: 3.3 V");
                    ImGui::Text("Type: Zener Diode");
                    ImGui::Unindent();
                }
            }
            // ---- POWER ----
            else if (cat == "power") {
                if (ctype == "dc_voltage") {
                    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f), "DC Voltage Source");
                    ImGui::Indent();

                    // Read current voltage from component
                    float current_voltage = 5.0f;
                    if (auto* dc_comp = dynamic_cast<CircuitComponentAdapter<DCVoltageSource>*>(comp)) {
                        current_voltage = static_cast<float>(dc_comp->circuit_component()->get_parameter("voltage"));
                    }

                    // Editable voltage slider
                    bool voltage_changed = ImGui::SliderFloat("Output Voltage", &current_voltage, 0.0f, 48.0f, "%.1f V");

                    if (voltage_changed) {
                        if (auto* dc_comp = dynamic_cast<CircuitComponentAdapter<DCVoltageSource>*>(comp)) {
                            dc_comp->circuit_component()->set_parameter("voltage", current_voltage);
                            spdlog::info("DC Voltage set to {:.1f} V", current_voltage);
                        }
                    }

                    ImGui::Text("Type: DC Voltage Source");
                    ImGui::Text("Range: 0V - 48V");
                    ImGui::Unindent();
                } else if (ctype == "h_bridge") {
                    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f), "Runtime Data");
                    ImGui::Indent();
                    ImGui::Text("Supply Voltage: 12.0 V");
                    ImGui::Text("PWM Frequency: 1000 Hz");
                    ImGui::Text("Type: H-Bridge Motor Driver");
                    ImGui::Unindent();
                } else if (ctype == "buck_converter") {
                    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f), "Runtime Data");
                    ImGui::Indent();
                    ImGui::Text("Input Voltage: 12.0 V");
                    ImGui::Text("Duty Cycle: 50%%");
                    ImGui::Text("Type: Buck Converter");
                    ImGui::Unindent();
                } else if (ctype == "boost_converter") {
                    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f), "Runtime Data");
                    ImGui::Indent();
                    ImGui::Text("Input Voltage: 5.0 V");
                    ImGui::Text("Duty Cycle: 50%%");
                    ImGui::Text("Type: Boost Converter");
                    ImGui::Unindent();
                } else if (ctype == "motor_driver") {
                    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f), "Runtime Data");
                    ImGui::Indent();
                    ImGui::Text("Supply Voltage: 12.0 V");
                    ImGui::Text("Type: Motor Driver");
                    ImGui::Unindent();
                }
            }
            // ---- MCU ----
            else if (cat == "mcu") {
                ImGui::TextColored(ImVec4(0.8f, 0.6f, 1.0f, 1.0f), "MCU");
                ImGui::Indent();
                ImGui::Text("Type: %s", node.type.c_str());
                ImGui::Text("Status: Ready");
                ImGui::Unindent();
            }
            // ---- CONTROL ----
            else if (cat == "control") {
                ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), "Controller");
                ImGui::Indent();
                ImGui::Text("Type: %s", node.type.c_str());
                ImGui::Unindent();
            }

            // Ports with Current Flow
            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.6f, 0.6f, 1.0f, 1.0f), "Pin Voltages & Currents");
            ImGui::Indent();
            auto ports = comp->get_ports();
            if (ports.empty()) {
                ImGui::TextDisabled("No ports");
            } else {
                // Try to get circuit pins for current info
                std::vector<CircuitPin*> circuit_pins;

                // Get circuit pins if this is a CircuitComponentAdapter
                if (cat == "passive" || cat == "semiconductor" || cat == "power") {
                    // Try different adapter types
                    #define GET_PINS(AdapterType) \
                        if (auto* adapted = dynamic_cast<AdapterType*>(comp)) { \
                            auto pins = adapted->circuit_component()->get_pins(); \
                            circuit_pins.assign(pins.begin(), pins.end()); \
                        }

                    if (ctype == "resistor") {
                        GET_PINS(ResistorComponent);
                    } else if (ctype == "capacitor") {
                        GET_PINS(CapacitorComponent);
                    } else if (ctype == "inductor") {
                        GET_PINS(InductorComponent);
                    } else if (ctype == "diode") {
                        GET_PINS(DiodeComponent);
                    } else if (ctype == "led") {
                        GET_PINS(LEDComponent);
                    } else if (ctype == "zener_diode") {
                        GET_PINS(ZenerDiodeComponent);
                    } else if (ctype == "bjt_npn" || ctype == "bjt_pnp") {
                        GET_PINS(BJTComponent);
                    } else if (ctype == "mosfet_n" || ctype == "mosfet_p") {
                        GET_PINS(MOSFETComponent);
                    } else if (ctype == "dc_voltage") {
                        GET_PINS(DCVoltageComponent);
                    } else if (ctype == "ground") {
                        GET_PINS(GroundComponent);
                    } else if (ctype == "h_bridge") {
                        GET_PINS(HBridgeComponent);
                    } else if (ctype == "buck_converter") {
                        GET_PINS(BuckConverterComponent);
                    } else if (ctype == "boost_converter") {
                        GET_PINS(BoostConverterComponent);
                    } else if (ctype == "motor_driver") {
                        GET_PINS(MotorDriverComponent);
                    }

                    #undef GET_PINS
                }

                for (size_t i = 0; i < ports.size(); ++i) {
                    auto* port = ports[i];

                    // Get voltage from port
                    const auto& val = port->value();
                    std::string val_str;
                    if (const float* f = std::get_if<float>(&val)) {
                        val_str = std::to_string(*f);
                    } else if (const bool* b = std::get_if<bool>(&val)) {
                        val_str = *b ? "true" : "false";
                    } else {
                        val_str = "-";
                    }

                    // Get current from circuit pin if available
                    std::string current_str = "N/A";
                    if (i < circuit_pins.size() && circuit_pins[i]) {
                        float current_a = circuit_pins[i]->current;
                        if (std::abs(current_a) < 1e-6f) {
                            current_str = "0 A";
                        } else if (std::abs(current_a) < 1e-3f) {
                            current_str = std::to_string(current_a * 1e6f) + " µA";
                        } else if (std::abs(current_a) < 1.0f) {
                            current_str = std::to_string(current_a * 1e3f) + " mA";
                        } else {
                            current_str = std::to_string(current_a) + " A";
                        }
                    }

                    ImGui::BulletText("%s [%s]: %s V | I: %s",
                        port->name().data(),
                        port->direction() == PortDirection::Input ? "IN" :
                        port->direction() == PortDirection::Output ? "OUT" : "BIDI",
                        val_str.c_str(),
                        current_str.c_str());
                }
            }
            ImGui::Unindent();
        }
    }

    ImGui::Spacing();
    ImGui::Separator();
    if (ImGui::Button("Delete Component", ImVec2(150, 0))) {
        remove_node(m_selected_node_index);
    }
}

#undef TRY_CAST

} // namespace mechatron
