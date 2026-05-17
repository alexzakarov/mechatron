#include "CircuitEditor.hpp"
#include "core/SimulationOrchestrator.hpp"
#include "core/Registry.hpp"
#include "core/Component.hpp"
#include "core/Port.hpp"
#include "actuators/Actuator.hpp"
#include "sensors/Sensor.hpp"
#include "electronics/CircuitComponentAdapter.hpp"
#include "electronics/CircuitSimulator.hpp"
#include "ui/SchematicSymbol.hpp"
#include <imgui.h>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

namespace mechatron {

// Helpers (defined below)
static nlohmann::json symbol_to_json(const SchematicSymbol& sym);

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
        {"oscilloscope",      {"instrument",         "oscilloscope"}},
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
                          cat == "estimator" || cat == "thermal" || cat == "magnetic" ||
                          cat == "instrument");

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
}

void CircuitEditor::add_esc_template() {
    if (!m_orchestrator) return;

    // Save current node count for positioning
    size_t start_node_count = m_nodes.size();

    // Helper to add component with auto-positioning
    auto add_comp = [this, &start_node_count](const char* plugin, const char* type,
                                                const char* id, float x, float y) -> Component* {
        Component* comp = m_orchestrator->create_component(plugin, type, id);
        if (!comp) {
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

        return comp;
    };

    // ESC Template Layout Configuration
    struct ESCTemplateLayout {
        // Origin position
        float start_x = 100.0f;
        float start_y = 100.0f;

        // Vertical spacing
        float power_supply_offset = 0.0f;
        float mosfet_row_offset = 200.0f;
        float gate_drive_offset = 400.0f;
        float gate_resistor_offset = 550.0f;
        float motor_winding_offset = 680.0f;

        // Horizontal spacing
        float phase_spacing = 150.0f;  // Space between A/B/C phases
        float high_low_spacing = 80.0f;  // Space between high/low MOSFETs
        float gate_source_spacing = 60.0f;  // Space between gate drive sources
        float gate_resistor_x_offset = 60.0f;  // X offset for gate resistors
        float gate_resistor_y_spacing = 40.0f;  // Y spacing for gate resistors
        float resistor_x_spacing = 120.0f;  // Additional X spacing for resistors
        float motor_winding_spacing = 120.0f;  // Space between motor windings
        float motor_winding_x_offset = 30.0f;  // X offset for motor windings
    } layout;

    // 1. Power Supply
    add_comp("elec_power", "dc_voltage", "BAT", layout.start_x, layout.start_y + layout.power_supply_offset);
    add_comp("elec_passive", "ground", "GND", layout.start_x, layout.start_y + layout.power_supply_offset + layout.high_low_spacing);

    // 2. 6 MOSFETs (3 half-bridges)
    float mosfet_y = layout.start_y + layout.mosfet_row_offset;
    add_comp("elec_semiconductor", "mosfet_n", "AH", layout.start_x, mosfet_y);  // High-side A
    add_comp("elec_semiconductor", "mosfet_n", "AL", layout.start_x, mosfet_y + layout.high_low_spacing);  // Low-side A
    add_comp("elec_semiconductor", "mosfet_n", "BH", layout.start_x + layout.phase_spacing, mosfet_y);  // High-side B
    add_comp("elec_semiconductor", "mosfet_n", "BL", layout.start_x + layout.phase_spacing, mosfet_y + layout.high_low_spacing);  // Low-side B
    add_comp("elec_semiconductor", "mosfet_n", "CH", layout.start_x + 2.0f * layout.phase_spacing, mosfet_y);  // High-side C
    add_comp("elec_semiconductor", "mosfet_n", "CL", layout.start_x + 2.0f * layout.phase_spacing, mosfet_y + layout.high_low_spacing);  // Low-side C

    // 3. Gate Drive Sources
    float gate_y = layout.start_y + layout.gate_drive_offset;
    add_comp("elec_power", "dc_voltage", "G_AH", layout.start_x, gate_y);
    add_comp("elec_power", "dc_voltage", "G_AL", layout.start_x, gate_y + layout.gate_source_spacing);
    add_comp("elec_power", "dc_voltage", "G_BH", layout.start_x + layout.phase_spacing, gate_y);
    add_comp("elec_power", "dc_voltage", "G_BL", layout.start_x + layout.phase_spacing, gate_y + layout.gate_source_spacing);
    add_comp("elec_power", "dc_voltage", "G_CH", layout.start_x + 2.0f * layout.phase_spacing, gate_y);
    add_comp("elec_power", "dc_voltage", "G_CL", layout.start_x + 2.0f * layout.phase_spacing, gate_y + layout.gate_source_spacing);

    // 4. Gate Resistors
    float rg_y = layout.start_y + layout.gate_resistor_offset;
    add_comp("elec_passive", "resistor", "RgAH", layout.start_x + layout.gate_resistor_x_offset, rg_y);
    add_comp("elec_passive", "resistor", "RgAL", layout.start_x + layout.gate_resistor_x_offset, rg_y + layout.gate_resistor_y_spacing);
    add_comp("elec_passive", "resistor", "RgBH", layout.start_x + layout.phase_spacing + layout.gate_resistor_x_offset, rg_y);
    add_comp("elec_passive", "resistor", "RgBL", layout.start_x + layout.phase_spacing + layout.gate_resistor_x_offset, rg_y + layout.gate_resistor_y_spacing);
    add_comp("elec_passive", "resistor", "RgCH", layout.start_x + 2.0f * layout.phase_spacing + layout.gate_resistor_x_offset, rg_y);
    add_comp("elec_passive", "resistor", "RgCL", layout.start_x + 2.0f * layout.phase_spacing + layout.gate_resistor_x_offset, rg_y + layout.gate_resistor_y_spacing);

    // 5. Motor Windings (Y-connected)
    float load_y = layout.start_y + layout.motor_winding_offset;
    add_comp("elec_passive", "resistor", "R_A", layout.start_x + layout.motor_winding_x_offset, load_y);
    add_comp("elec_passive", "resistor", "R_B", layout.start_x + layout.phase_spacing, load_y);
    add_comp("elec_passive", "resistor", "R_C", layout.start_x + 2.0f * layout.phase_spacing - layout.motor_winding_x_offset, load_y);

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

    // Select the first MOSFET to show properties
    m_selected_node_index = static_cast<int>(start_node_count);  // Select AH
    m_selected_node_id = "AH";
    m_selected_wire_index = -1;
}

void CircuitEditor::remove_node(int index) {
    if (!m_orchestrator || index < 0 || index >= static_cast<int>(m_nodes.size())) return;

    std::string removed_id = m_nodes[index].id;

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
            } else {
                spdlog::warn("Port not found for wire connection: {}[{}] -> {}[{}]", src_node, src_pin, tgt_node, tgt_pin);
            }
        }
    }

    return true;
}

void CircuitEditor::remove_wire(int index) {
    if (index < 0 || index >= static_cast<int>(m_wires.size())) {
        return;
    }

    const auto& wire = m_wires[index];

    // Remove the actual Port Connection from orchestrator
    if (m_orchestrator) {
        Component* src_comp = m_orchestrator->registry().get(wire.from_node);
        Component* tgt_comp = m_orchestrator->registry().get(wire.to_node);

        if (src_comp && tgt_comp) {
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

            if (src_port && tgt_port) {
                for (auto* conn : src_port->connections()) {
                    if ((conn->source == src_port && conn->target == tgt_port) ||
                        (conn->source == tgt_port && conn->target == src_port)) {
                        m_orchestrator->disconnect(conn);
                        break;
                    }
                }
            }
        }
    }

    m_wires.erase(m_wires.begin() + index);
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

    // If a schematic symbol exists for this type (or is currently being edited), use its pin anchors.
    static SchematicSymbolLibrary sym_lib;
    SchematicSymbol sym;
    bool has_sym = sym_lib.load_for_type(node.type, sym);
    if (m_symbol_editor_open && m_symbol_edit_inplace && m_symbol_loaded && m_symbol_state && m_symbol_edit_type == node.type) {
        sym = m_symbol_state->sym;
        has_sym = true;
    }

    if (has_sym) {
        // Compute node height like render does.
        int input_count = 0, output_count = 0;
        for (const auto& pin : node.pins) {
            if (pin.is_input) input_count++;
            else output_count++;
        }
        if (input_count == 0) input_count = 1;
        if (output_count == 0) output_count = 1;
        int max_side = std::max(input_count, output_count);
        float node_height = label_height + static_cast<float>(max_side) * pin_spacing + 10.0f;

        auto sym_to_local = [&](float sx, float sy) -> ImVec2 {
            const float pad_x = -10.0f;
            const float pad_y = -10.0f;
            const float avail_w = node_width - 2.0f * pad_x;
            const float avail_h = node_height - 2.0f * pad_y;
            const float scale = std::min(avail_w / std::max(1.0f, sym.width), avail_h / std::max(1.0f, sym.height));
            const float ox = node.position[0] + pad_x + (avail_w - sym.width * scale) * 0.5f;
            const float oy = node.position[1] + pad_y + (avail_h - sym.height * scale) * 0.5f;
            return ImVec2(ox + sx * scale, oy + sy * scale);
        };

        for (const auto& pin : node.pins) {
            auto it = sym.pins.find(pin.name);
            if (it == sym.pins.end()) continue;
            ImVec2 p = sym_to_local(it->second.first, it->second.second);
            float dist = std::sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y));
            if (dist < hit_radius) return &pin;
        }
        return nullptr;
    }

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

    // Symbol anchor mapping if present.
    static SchematicSymbolLibrary sym_lib;
    SchematicSymbol sym;
    bool has_sym = sym_lib.load_for_type(node.type, sym);
    if (m_symbol_editor_open && m_symbol_edit_inplace && m_symbol_loaded && m_symbol_state && m_symbol_edit_type == node.type) {
        sym = m_symbol_state->sym;
        has_sym = true;
    }

    if (has_sym) {
        // Compute node height like render does.
        int input_count = 0, output_count = 0;
        for (const auto& pin : node.pins) {
            if (pin.is_input) input_count++;
            else output_count++;
        }
        if (input_count == 0) input_count = 1;
        if (output_count == 0) output_count = 1;
        int max_side = std::max(input_count, output_count);
        float node_height = label_height + static_cast<float>(max_side) * pin_spacing + 10.0f;

        auto itp = sym.pins.find(pin_name);
        if (itp != sym.pins.end()) {
            const float pad_x = -10.0f;
            const float pad_y = -10.0f;
            const float avail_w = node_width - 2.0f * pad_x;
            const float avail_h = node_height - 2.0f * pad_y;
            const float scale = std::min(avail_w / std::max(1.0f, sym.width), avail_h / std::max(1.0f, sym.height));
            const float ox = node.position[0] + pad_x + (avail_w - sym.width * scale) * 0.5f;
            const float oy = node.position[1] + pad_y + (avail_h - sym.height * scale) * 0.5f;
            const float px = ox + itp->second.first * scale;
            const float py = oy + itp->second.second * scale;
            return ImVec2(canvas_pos.x + px, canvas_pos.y + py);
        }
    }

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
            continue;
        }

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

    // No Begin - we're inside a tab

    // Toolbar buttons (instead of menu bar)
    if (ImGui::Button("Clear All")) {
        // Remove all components and wires
        for (int i = static_cast<int>(m_nodes.size()) - 1; i >= 0; --i) {
            m_orchestrator->remove_component(m_nodes[i].id);
        }
        m_nodes.clear();
        m_wires.clear();
        m_selected_node_index = -1;
        m_selected_wire_index = -1;
        m_selected_node_id.clear();
        m_orchestrator->set_selected_component("");
    }
    ImGui::SameLine();
    if (ImGui::Button("Delete Selected")) {
        if (m_selected_wire_index >= 0) {
            remove_wire(m_selected_wire_index);
            m_selected_wire_index = -1;
        } else if (m_selected_node_index >= 0) {
            remove_node(m_selected_node_index);
        }
    }
    ImGui::SameLine();
    if (m_selected_node_index >= 0 && m_selected_node_index < (int)m_nodes.size()) {
        if (ImGui::Button("Edit Symbol")) {
            m_symbol_editor_open = !m_symbol_editor_open;
            if (m_symbol_editor_open) m_symbol_loaded = false;
        }
        ImGui::SameLine();
        ImGui::Checkbox("In-Place", &m_symbol_edit_inplace);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Edit directly on the component container");
    } else {
        ImGui::BeginDisabled();
        ImGui::Button("Edit Symbol");
        ImGui::SameLine();
        ImGui::Checkbox("In-Place", &m_symbol_edit_inplace);
        ImGui::EndDisabled();
    }
    ImGui::SameLine();
    ImGui::Checkbox("Components", &m_show_component_list);
    ImGui::SameLine();
    ImGui::Checkbox("Properties", &m_show_properties);

    ImGui::Separator();

    // In-place save (per selected type)
    if (m_symbol_editor_open && m_symbol_edit_inplace && m_symbol_dirty && m_symbol_state && !m_symbol_edit_type.empty()) {
        ImGui::TextDisabled("Symbol modified (%s).", m_symbol_edit_type.c_str());
        ImGui::SameLine();
        if (ImGui::Checkbox("Container", &m_symbol_state->sym.container.enabled)) {
            m_symbol_dirty = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Save Symbol")) {
            namespace fs = std::filesystem;
            std::error_code ec;
            fs::create_directories(SchematicSymbolLibrary::user_dir(), ec);
            const fs::path outp = fs::path(SchematicSymbolLibrary::user_dir()) / (m_symbol_edit_type + ".json");
            std::ofstream out(outp.string());
            if (out.is_open()) {
                out << symbol_to_json(m_symbol_state->sym).dump(2);
                m_symbol_dirty = false;
            }
        }
        ImGui::Separator();
    }

    // Layout
    if (m_show_component_list) {
        ImGui::BeginChild("ComponentPalette", ImVec2(150, 0), true);
        render_component_palette();
        ImGui::EndChild();
        ImGui::SameLine();
    }

    ImGui::BeginChild("CircuitCanvas", ImVec2(m_show_properties ? -200 : 0, 0), true);
    // In-place mode edits directly in the circuit canvas. Full-canvas symbol editor is available when In-Place is off.
    if (m_symbol_editor_open && !m_symbol_edit_inplace) render_symbol_editor();
    else render_circuit_canvas();
    ImGui::EndChild();

    if (m_show_properties) {
        ImGui::SameLine();
        ImGui::BeginChild("PropertiesPanel", ImVec2(200, 0), true);
        render_properties_panel();
        ImGui::EndChild();
    }
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
    ImGui::TextDisabled("Instruments");
    add_btn("Oscilloscope", "oscilloscope");

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

            // Double-click: open oscilloscope tab
            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                if (m_nodes[clicked].type == "oscilloscope") {
                    m_oscilloscope_open_id = m_nodes[clicked].id;
                }
            }
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
    static SchematicSymbolLibrary sym_lib;
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
        // Schematic symbol (may define whether the outer container is shown).
        SchematicSymbol sym;
        const bool has_sym = sym_lib.load_for_type(node.type, sym);

        const bool edit_active = m_symbol_editor_open && m_symbol_edit_inplace && (static_cast<int>(i) == m_selected_node_index);
        if (edit_active) {
            // When editing, use the live edited symbol as the source of truth for container visibility.
            if (m_symbol_state && m_symbol_loaded && m_symbol_edit_type == node.type) {
                sym = m_symbol_state->sym;
            }
        }

        const bool container_enabled = edit_active ? sym.container.enabled : (has_sym ? sym.container.enabled : true);
        const float container_radius = edit_active ? sym.container.corner_radius : (has_sym ? sym.container.corner_radius : 4.0f);

        if (container_enabled) {
            ImU32 bg_color = is_selected ?
                ImGui::GetColorU32(ImVec4(0.3f, 0.5f, 0.7f, 0.8f)) :
                ImGui::GetColorU32(ImVec4(0.2f, 0.2f, 0.2f, 0.9f));

            draw_list->AddRectFilled(node_pos, ImVec2(node_pos.x + node_width, node_pos.y + node_height), bg_color, container_radius);
            draw_list->AddRect(node_pos, ImVec2(node_pos.x + node_width, node_pos.y + node_height),
                              is_selected ? IM_COL32(100, 200, 255, 255) : IM_COL32(255, 255, 255, 255),
                              container_radius, 0, is_selected ? 2.5f : 1.5f);
        } else if (is_selected) {
            // Frameless, but keep a subtle selection outline so it stays usable.
            draw_list->AddRect(node_pos, ImVec2(node_pos.x + node_width, node_pos.y + node_height),
                              IM_COL32(100, 200, 255, 160), 3.0f, 0, 2.0f);
        }

        // Label
        draw_list->AddText(ImVec2(node_pos.x + 5, node_pos.y + 5),
                          IM_COL32(255, 255, 255, 255), node.type.c_str());

        // Schematic symbol render (if available). If missing, keep the legacy pin layout.
        auto sym_to_canvas = [&](float sx, float sy) -> ImVec2 {
            // Map symbol coordinates into an area slightly larger than the node container (outward padding),
            // so drawings can extend beyond the blue container.
            const float pad_x = -10.0f;
            const float pad_y = -10.0f;
            const float avail_w = node_width - 2.0f * pad_x;
            const float avail_h = node_height - 2.0f * pad_y;
            const float scale = std::min(avail_w / std::max(1.0f, sym.width), avail_h / std::max(1.0f, sym.height));
            const float ox = node_pos.x + pad_x + (avail_w - sym.width * scale) * 0.5f;
            const float oy = node_pos.y + pad_y + (avail_h - sym.height * scale) * 0.5f;
            return ImVec2(ox + sx * scale, oy + sy * scale);
        };

        if (has_sym) {
            for (const auto& p : sym.body) {
                ImVec2 a = sym_to_canvas(p.x1, p.y1);
                ImVec2 b = sym_to_canvas(p.x2, p.y2);
                if (p.type == SchematicSymbolPrimitive::Type::Rect) {
                    draw_list->AddRect(a, b, IM_COL32(230, 230, 230, 220), 2.0f, 0, p.thickness);
                } else {
                    draw_list->AddLine(a, b, IM_COL32(230, 230, 230, 220), p.thickness);
                }
            }
        }

        // In-place symbol edit: allow dragging pins/endpoints directly on the component container.
        // This edits the per-type symbol (saved to assets/symbols_user/<type>.json from the Symbol Editor).
        // edit_active already computed above
        if (edit_active) {
            if (!m_symbol_state) m_symbol_state = std::make_unique<SymbolEditState>();
            if (m_symbol_edit_type != node.type) {
                m_symbol_edit_type = node.type;
                m_symbol_loaded = false;
                m_symbol_dirty = false;
                *m_symbol_state = SymbolEditState{};
            }
            if (!m_symbol_loaded) {
                if (has_sym) m_symbol_state->sym = sym;
                else {
                    m_symbol_state->sym = SchematicSymbol{};
                    m_symbol_state->sym.width = 100;
                    m_symbol_state->sym.height = 70;
                    m_symbol_state->sym.body.push_back({SchematicSymbolPrimitive::Type::Rect, 15, 15, 85, 55, 2.0f});
                    int in_i = 0, out_i = 0;
                    for (const auto& p : node.pins) {
                        if (p.is_input) { m_symbol_state->sym.pins[p.name] = {10.0f, 22.0f + in_i * 12.0f}; in_i++; }
                        else { m_symbol_state->sym.pins[p.name] = {90.0f, 22.0f + out_i * 12.0f}; out_i++; }
                    }
                }
                m_symbol_loaded = true;
            }
            // In edit mode we always render from the live edited symbol.
            sym = m_symbol_state->sym;

            // sync missing pins
            for (const auto& p : node.pins) {
                if (m_symbol_state->sym.pins.find(p.name) == m_symbol_state->sym.pins.end()) {
                    m_symbol_state->sym.pins[p.name] = {m_symbol_state->sym.width * 0.5f, m_symbol_state->sym.height * 0.5f};
                    m_symbol_dirty = true;
                }
            }

            auto from_canvas = [&](ImVec2 cp, float& sx, float& sy) {
                // Inverse of sym_to_canvas above (same layout constants).
                const float pad_x = -10.0f;
                const float pad_y = -10.0f;
                const float avail_w = node_width - 2.0f * pad_x;
                const float avail_h = node_height - 2.0f * pad_y;
                const float scale = std::min(avail_w / std::max(1.0f, m_symbol_state->sym.width), avail_h / std::max(1.0f, m_symbol_state->sym.height));
                const float ox = node_pos.x + pad_x + (avail_w - m_symbol_state->sym.width * scale) * 0.5f;
                const float oy = node_pos.y + pad_y + (avail_h - m_symbol_state->sym.height * scale) * 0.5f;
                sx = (cp.x - ox) / scale;
                sy = (cp.y - oy) / scale;
                // Allow editing slightly outside the nominal symbol bounds.
                const float margin = 25.0f;
                sx = std::clamp(sx, -margin, m_symbol_state->sym.width + margin);
                sy = std::clamp(sy, -margin, m_symbol_state->sym.height + margin);
            };

            auto dist2 = [](ImVec2 a, ImVec2 b) {
                float dx = a.x - b.x, dy = a.y - b.y;
                return dx * dx + dy * dy;
            };

            ImVec2 mp = ImGui::GetMousePos();
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                m_symbol_state->drag_pin.clear();
                m_symbol_state->drag_prim = -1;
                m_symbol_state->drag_point = -1;

                // pin hit
                const float pin_r = 9.0f;
                for (const auto& [name, pt] : m_symbol_state->sym.pins) {
                    ImVec2 p = sym_to_canvas(pt.first, pt.second);
                    if (dist2(p, mp) <= pin_r * pin_r) {
                        m_symbol_state->drag_pin = name;
                        break;
                    }
                }
                // endpoint hit
                if (m_symbol_state->drag_pin.empty()) {
                    const float ep_r = 8.0f;
                    for (int pi = 0; pi < (int)m_symbol_state->sym.body.size(); ++pi) {
                        const auto& pr = m_symbol_state->sym.body[pi];
                        ImVec2 p0 = sym_to_canvas(pr.x1, pr.y1);
                        ImVec2 p1 = sym_to_canvas(pr.x2, pr.y2);
                        if (dist2(p0, mp) <= ep_r * ep_r) { m_symbol_state->selected_prim = pi; m_symbol_state->drag_prim = pi; m_symbol_state->drag_point = 0; break; }
                        if (dist2(p1, mp) <= ep_r * ep_r) { m_symbol_state->selected_prim = pi; m_symbol_state->drag_prim = pi; m_symbol_state->drag_point = 1; break; }
                    }
                }
            }

            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                float sx, sy;
                from_canvas(mp, sx, sy);
                if (!m_symbol_state->drag_pin.empty()) {
                    m_symbol_state->sym.pins[m_symbol_state->drag_pin] = {sx, sy};
                    m_symbol_dirty = true;
                } else if (m_symbol_state->drag_prim >= 0 && m_symbol_state->drag_prim < (int)m_symbol_state->sym.body.size()) {
                    auto& pr = m_symbol_state->sym.body[m_symbol_state->drag_prim];
                    if (m_symbol_state->drag_point == 0) { pr.x1 = sx; pr.y1 = sy; }
                    else { pr.x2 = sx; pr.y2 = sy; }
                    m_symbol_dirty = true;
                }
            }
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                m_symbol_state->drag_pin.clear();
                m_symbol_state->drag_prim = -1;
                m_symbol_state->drag_point = -1;
            }

            // overlay: highlight pins and endpoints when editing
            for (const auto& [name, pt] : m_symbol_state->sym.pins) {
                ImVec2 p = sym_to_canvas(pt.first, pt.second);
                draw_list->AddCircle(p, 8.0f, IM_COL32(255, 255, 255, 90), 12, 1.5f);
            }
            for (int pi = 0; pi < (int)m_symbol_state->sym.body.size(); ++pi) {
                const auto& pr = m_symbol_state->sym.body[pi];
                ImVec2 p0 = sym_to_canvas(pr.x1, pr.y1);
                ImVec2 p1 = sym_to_canvas(pr.x2, pr.y2);
                ImU32 col = (pi == m_symbol_state->selected_prim) ? IM_COL32(120, 220, 255, 200) : IM_COL32(255, 255, 255, 90);
                draw_list->AddCircleFilled(p0, 3.5f, col);
                draw_list->AddCircleFilled(p1, 3.5f, col);
            }
        }

        // Draw input pins (left side)
        int input_idx = 0;
        for (const auto& pin : node.pins) {
            if (!pin.is_input) continue;

            float pin_x = node_pos.x;
            float pin_y = node_pos.y + label_height + static_cast<float>(input_idx) * pin_spacing;
            if (has_sym) {
                auto it = sym.pins.find(pin.name);
                if (it != sym.pins.end()) {
                    ImVec2 ppos = sym_to_canvas(it->second.first, it->second.second);
                    pin_x = ppos.x;
                    pin_y = ppos.y;
                }
            }

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
            if (has_sym) {
                auto it = sym.pins.find(pin.name);
                if (it != sym.pins.end()) {
                    ImVec2 ppos = sym_to_canvas(it->second.first, it->second.second);
                    pin_x = ppos.x;
                    pin_y = ppos.y;
                }
            }

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

                    auto port_voltage = [](Component* comp, Port* port) {
                        if (!port) return 0.0f;
                        float voltage = 0.0f;
                        if (comp && comp->category() == "mcu" &&
                            comp->get_mcu_pin_output_voltage(port->name(), voltage)) {
                            return voltage;
                        }
                        if (const float* f = port->get_value<float>()) {
                            return *f;
                        }
                        if (const bool* b = port->get_value<bool>()) {
                            return *b ? 5.0f : 0.0f;
                        }
                        return 0.0f;
                    };

                    float src_voltage = port_voltage(src_comp, src_port);
                    float tgt_voltage = port_voltage(tgt_comp, tgt_port);

                    ImGui::Text("Voltage: %.2f V -> %.2f V", src_voltage, tgt_voltage);

                    // Get current from the terminal connected to this wire.
                    // Prefer the target terminal so a shared supply/ground node
                    // does not show total source current on every branch.
                    float current_a = 0.0f;
                    bool current_found = false;
                    std::string current_source_label;

                    auto get_actuator_current = [this](Component* comp, std::string_view pin_name, float& current) {
                        return m_orchestrator &&
                               comp &&
                               comp->category() == "actuator" &&
                               m_orchestrator->get_actuator_terminal_current(comp->id(), pin_name, current);
                    };

                    #define GET_CURRENT(AdapterType, comp_ptr, pin_name, out_current, out_found) \
                        if (auto* adapted = dynamic_cast<AdapterType*>(comp_ptr)) { \
                            auto pins = adapted->circuit_component()->get_pins(); \
                            for (auto* pin : pins) { \
                                if (pin && pin->id == pin_name) { \
                                    out_current = pin->current; \
                                    out_found = true; \
                                    break; \
                                } \
                            } \
                        }

                    auto get_component_pin_current = [&](Component* comp, std::string_view pin_name, float& current) {
                        bool found = false;
                        if (!comp) return false;

                        if (get_actuator_current(comp, pin_name, current)) {
                            return true;
                        }

                        std::string_view cat = comp->category();
                        std::string_view ctype = comp->component_type();

                        if (cat == "passive" || cat == "semiconductor" || cat == "power") {
                            if (ctype == "resistor") { GET_CURRENT(ResistorComponent, comp, pin_name, current, found); }
                            else if (ctype == "capacitor") { GET_CURRENT(CapacitorComponent, comp, pin_name, current, found); }
                            else if (ctype == "inductor") { GET_CURRENT(InductorComponent, comp, pin_name, current, found); }
                            else if (ctype == "diode") { GET_CURRENT(DiodeComponent, comp, pin_name, current, found); }
                            else if (ctype == "led") { GET_CURRENT(LEDComponent, comp, pin_name, current, found); }
                            else if (ctype == "zener_diode") { GET_CURRENT(ZenerDiodeComponent, comp, pin_name, current, found); }
                            else if (ctype == "bjt_npn" || ctype == "bjt_pnp") { GET_CURRENT(BJTComponent, comp, pin_name, current, found); }
                            else if (ctype == "mosfet_n" || ctype == "mosfet_p") { GET_CURRENT(MOSFETComponent, comp, pin_name, current, found); }
                            else if (ctype == "dc_voltage") { GET_CURRENT(DCVoltageComponent, comp, pin_name, current, found); }
                            else if (ctype == "ground") { GET_CURRENT(GroundComponent, comp, pin_name, current, found); }
                            else if (ctype == "h_bridge") { GET_CURRENT(HBridgeComponent, comp, pin_name, current, found); }
                            else if (ctype == "buck_converter") { GET_CURRENT(BuckConverterComponent, comp, pin_name, current, found); }
                            else if (ctype == "boost_converter") { GET_CURRENT(BoostConverterComponent, comp, pin_name, current, found); }
                            else if (ctype == "motor_driver") { GET_CURRENT(MotorDriverComponent, comp, pin_name, current, found); }
                        }

                        return found;
                    };

                    auto current_endpoint_priority = [](Component* comp) {
                        if (!comp) return 0;

                        std::string_view category = comp->category();
                        std::string_view type = comp->component_type();

                        if (category == "power") {
                            if (type == "dc_voltage" || type == "ground") {
                                return 1;
                            }
                            return 3;
                        }

                        if (category == "passive" || category == "semiconductor" || category == "electronic" || category == "optoelectronic") {
                            return 3;
                        }

                        if (category == "actuator") {
                            return 2;
                        }

                        return 0;
                    };

                    int src_priority = current_endpoint_priority(src_comp);
                    int tgt_priority = current_endpoint_priority(tgt_comp);

                    if (src_priority > tgt_priority) {
                        current_found = get_component_pin_current(src_comp, wire.from_pin_name, current_a);
                        if (!current_found) {
                            current_found = get_component_pin_current(tgt_comp, wire.to_pin_name, current_a);
                            if (current_found && tgt_comp) {
                                current_source_label = std::string(tgt_comp->id()) + "." + wire.to_pin_name;
                            }
                        } else if (src_comp) {
                            current_source_label = std::string(src_comp->id()) + "." + wire.from_pin_name;
                        }
                    } else {
                        current_found = get_component_pin_current(tgt_comp, wire.to_pin_name, current_a);
                        if (!current_found) {
                            current_found = get_component_pin_current(src_comp, wire.from_pin_name, current_a);
                            if (current_found && src_comp) {
                                current_source_label = std::string(src_comp->id()) + "." + wire.from_pin_name;
                            }
                        } else if (tgt_comp) {
                            current_source_label = std::string(tgt_comp->id()) + "." + wire.to_pin_name;
                        }
                    }

                    #undef GET_CURRENT

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
                        ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.5f, 1.0f), "Terminal Current: %s", current_str.c_str());
                        if (!current_source_label.empty()) {
                            ImGui::TextDisabled("Source: %s", current_source_label.c_str());
                        }
                    } else {
                        ImGui::TextDisabled("Terminal Current: N/A");
                    }
                }
            }

            ImGui::Separator();
            ImGui::Spacing();

            if (ImGui::Button("Delete", ImVec2(100, 0))) {
                int idx = m_selected_wire_index;
                remove_wire(idx);
                m_selected_wire_index = -1;
                ImGui::CloseCurrentPopup();
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

static nlohmann::json symbol_to_json(const SchematicSymbol& sym) {
    nlohmann::json j;
    j["width"] = sym.width;
    j["height"] = sym.height;
    j["container"] = {
        {"enabled", sym.container.enabled},
        {"corner_radius", sym.container.corner_radius}
    };
    j["body"] = nlohmann::json::array();
    for (const auto& p : sym.body) {
        nlohmann::json pj;
        pj["type"] = (p.type == SchematicSymbolPrimitive::Type::Rect) ? "rect" : "line";
        pj["x1"] = p.x1; pj["y1"] = p.y1; pj["x2"] = p.x2; pj["y2"] = p.y2;
        pj["thickness"] = p.thickness;
        j["body"].push_back(pj);
    }
    nlohmann::json pins = nlohmann::json::object();
    for (const auto& [name, pt] : sym.pins) {
        pins[name] = nlohmann::json::array({pt.first, pt.second});
    }
    j["pins"] = pins;
    return j;
}

void CircuitEditor::render_symbol_editor() {
    if (m_selected_node_index < 0 || m_selected_node_index >= static_cast<int>(m_nodes.size())) return;
    const auto& node = m_nodes[m_selected_node_index];
    const std::string& type = node.type;

    if (!m_symbol_state) m_symbol_state = std::make_unique<SymbolEditState>();

    if (m_symbol_edit_type != type) {
        m_symbol_edit_type = type;
        m_symbol_loaded = false;
        m_symbol_dirty = false;
        *m_symbol_state = SymbolEditState{};
    }

    if (!m_symbol_loaded) {
        SchematicSymbolLibrary lib;
        SchematicSymbol sym;
        if (lib.load_for_type(type, sym)) {
            m_symbol_state->sym = sym;
        } else {
            m_symbol_state->sym = SchematicSymbol{};
            m_symbol_state->sym.width = 100;
            m_symbol_state->sym.height = 70;
            m_symbol_state->sym.body.push_back({SchematicSymbolPrimitive::Type::Rect, 15, 15, 85, 55, 2.0f});
            int in_i = 0, out_i = 0;
            for (const auto& p : node.pins) {
                if (p.is_input) {
                    m_symbol_state->sym.pins[p.name] = {10.0f, 22.0f + in_i * 12.0f};
                    in_i++;
                } else {
                    m_symbol_state->sym.pins[p.name] = {90.0f, 22.0f + out_i * 12.0f};
                    out_i++;
                }
            }
        }
        m_symbol_loaded = true;
    }

    // Ensure pins exist for current ports (component schema may differ).
    for (const auto& p : node.pins) {
        if (m_symbol_state->sym.pins.find(p.name) == m_symbol_state->sym.pins.end()) {
            m_symbol_state->sym.pins[p.name] = {m_symbol_state->sym.width * 0.5f, m_symbol_state->sym.height * 0.5f};
            m_symbol_dirty = true;
        }
    }

    ImGui::TextColored(ImVec4(0.7f, 0.9f, 1.0f, 1.0f), "Symbol Editor");
    ImGui::SameLine();
    ImGui::TextDisabled("(Type: %s)", type.c_str());
    ImGui::SameLine();
    if (ImGui::Button("Back to Circuit")) {
        m_symbol_editor_open = false;
        return;
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("Container", &m_symbol_state->sym.container.enabled)) {
        m_symbol_dirty = true;
    }

    if (ImGui::Button("Add Line")) {
        m_symbol_state->add_line_mode = true;
        m_symbol_state->add_rect_mode = false;
        m_symbol_state->add_has_first = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("Add Rect")) {
        m_symbol_state->add_rect_mode = true;
        m_symbol_state->add_line_mode = false;
        m_symbol_state->add_has_first = false;
    }
    ImGui::SameLine();
    const bool can_delete = (m_symbol_state->selected_prim >= 0 && m_symbol_state->selected_prim < (int)m_symbol_state->sym.body.size());
    if (!can_delete) ImGui::BeginDisabled();
    if (ImGui::Button("Delete Shape")) {
        m_symbol_state->sym.body.erase(m_symbol_state->sym.body.begin() + m_symbol_state->selected_prim);
        m_symbol_state->selected_prim = -1;
        m_symbol_dirty = true;
    }
    if (!can_delete) ImGui::EndDisabled();

    ImGui::SameLine();
    if (m_symbol_dirty) {
        if (ImGui::Button("Save")) {
            namespace fs = std::filesystem;
            std::error_code ec;
            fs::create_directories(SchematicSymbolLibrary::user_dir(), ec);
            const fs::path outp = fs::path(SchematicSymbolLibrary::user_dir()) / (type + ".json");
            std::ofstream out(outp.string());
            if (out.is_open()) {
                out << symbol_to_json(m_symbol_state->sym).dump(2);
                m_symbol_dirty = false;
            }
        }
    } else {
        ImGui::BeginDisabled();
        ImGui::Button("Saved");
        ImGui::EndDisabled();
    }

    // Full available area under the toolbar becomes the drawing canvas.
    ImVec2 canvas_size(0, 0);
    ImGui::BeginChild("SymbolCanvas", canvas_size, true, ImGuiWindowFlags_NoScrollWithMouse);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 cpos = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    ImGui::InvisibleButton("symbol_canvas_btn", avail, ImGuiButtonFlags_MouseButtonLeft);
    const bool hovered = ImGui::IsItemHovered();
    ImVec2 mp = ImGui::GetMousePos();
    const float mx = mp.x - cpos.x;
    const float my = mp.y - cpos.y;

    auto to_canvas = [&](float sx, float sy) -> ImVec2 {
        const float pad = 8.0f;
        const float w = std::max(1.0f, avail.x - 2 * pad);
        const float h = std::max(1.0f, avail.y - 2 * pad);
        const float scale = std::min(w / std::max(1.0f, m_symbol_state->sym.width), h / std::max(1.0f, m_symbol_state->sym.height));
        const float ox = cpos.x + pad + (w - m_symbol_state->sym.width * scale) * 0.5f;
        const float oy = cpos.y + pad + (h - m_symbol_state->sym.height * scale) * 0.5f;
        return ImVec2(ox + sx * scale, oy + sy * scale);
    };
    auto from_canvas = [&](float cx, float cy, float& sx, float& sy) {
        const float pad = 8.0f;
        const float w = std::max(1.0f, avail.x - 2 * pad);
        const float h = std::max(1.0f, avail.y - 2 * pad);
        const float scale = std::min(w / std::max(1.0f, m_symbol_state->sym.width), h / std::max(1.0f, m_symbol_state->sym.height));
        const float ox = pad + (w - m_symbol_state->sym.width * scale) * 0.5f;
        const float oy = pad + (h - m_symbol_state->sym.height * scale) * 0.5f;
        sx = (cx - ox) / scale;
        sy = (cy - oy) / scale;
        sx = std::clamp(sx, 0.0f, m_symbol_state->sym.width);
        sy = std::clamp(sy, 0.0f, m_symbol_state->sym.height);
    };

    // Frame
    ImVec2 a0 = to_canvas(0, 0);
    ImVec2 a1 = to_canvas(m_symbol_state->sym.width, m_symbol_state->sym.height);
    dl->AddRect(a0, a1, IM_COL32(255, 255, 255, 60), 2.0f);

    auto dist2 = [](ImVec2 a, ImVec2 b) {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        return dx * dx + dy * dy;
    };

    if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        m_symbol_state->drag_pin.clear();
        m_symbol_state->drag_prim = -1;
        m_symbol_state->drag_point = -1;

        if (m_symbol_state->add_line_mode || m_symbol_state->add_rect_mode) {
            float sx, sy;
            from_canvas(mx, my, sx, sy);
            if (!m_symbol_state->add_has_first) {
                m_symbol_state->add_has_first = true;
                m_symbol_state->add_x0 = sx;
                m_symbol_state->add_y0 = sy;
            } else {
                SchematicSymbolPrimitive prim;
                prim.type = m_symbol_state->add_rect_mode ? SchematicSymbolPrimitive::Type::Rect : SchematicSymbolPrimitive::Type::Line;
                prim.x1 = m_symbol_state->add_x0;
                prim.y1 = m_symbol_state->add_y0;
                prim.x2 = sx;
                prim.y2 = sy;
                prim.thickness = 2.0f;
                m_symbol_state->sym.body.push_back(prim);
                m_symbol_state->selected_prim = (int)m_symbol_state->sym.body.size() - 1;
                m_symbol_dirty = true;
                m_symbol_state->add_has_first = false;
                m_symbol_state->add_line_mode = false;
                m_symbol_state->add_rect_mode = false;
            }
        } else {
            const float pin_r = 7.0f;
            for (const auto& [name, pt] : m_symbol_state->sym.pins) {
                ImVec2 p = to_canvas(pt.first, pt.second);
                if (dist2(p, mp) <= pin_r * pin_r) {
                    m_symbol_state->drag_pin = name;
                    break;
                }
            }
            if (m_symbol_state->drag_pin.empty()) {
                const float ep_r = 6.0f;
                for (int i = 0; i < (int)m_symbol_state->sym.body.size(); ++i) {
                    const auto& p = m_symbol_state->sym.body[i];
                    ImVec2 p0 = to_canvas(p.x1, p.y1);
                    ImVec2 p1 = to_canvas(p.x2, p.y2);
                    if (dist2(p0, mp) <= ep_r * ep_r) {
                        m_symbol_state->selected_prim = i;
                        m_symbol_state->drag_prim = i;
                        m_symbol_state->drag_point = 0;
                        break;
                    }
                    if (dist2(p1, mp) <= ep_r * ep_r) {
                        m_symbol_state->selected_prim = i;
                        m_symbol_state->drag_prim = i;
                        m_symbol_state->drag_point = 1;
                        break;
                    }
                }
            }
        }
    }

    if (ImGui::IsMouseDragging(ImGuiMouseButton_Left) && hovered) {
        float sx, sy;
        from_canvas(mx, my, sx, sy);
        if (!m_symbol_state->drag_pin.empty()) {
            m_symbol_state->sym.pins[m_symbol_state->drag_pin] = {sx, sy};
            m_symbol_dirty = true;
        } else if (m_symbol_state->drag_prim >= 0 && m_symbol_state->drag_prim < (int)m_symbol_state->sym.body.size()) {
            auto& p = m_symbol_state->sym.body[m_symbol_state->drag_prim];
            if (m_symbol_state->drag_point == 0) { p.x1 = sx; p.y1 = sy; }
            else if (m_symbol_state->drag_point == 1) { p.x2 = sx; p.y2 = sy; }
            m_symbol_dirty = true;
        }
    }
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        m_symbol_state->drag_pin.clear();
        m_symbol_state->drag_prim = -1;
        m_symbol_state->drag_point = -1;
    }

    // Draw primitives
    for (int i = 0; i < (int)m_symbol_state->sym.body.size(); ++i) {
        const auto& p = m_symbol_state->sym.body[i];
        ImVec2 p0 = to_canvas(p.x1, p.y1);
        ImVec2 p1 = to_canvas(p.x2, p.y2);
        const bool sel = (i == m_symbol_state->selected_prim);
        const ImU32 col = sel ? IM_COL32(120, 220, 255, 240) : IM_COL32(230, 230, 230, 220);
        if (p.type == SchematicSymbolPrimitive::Type::Rect) dl->AddRect(p0, p1, col, 2.0f, 0, p.thickness);
        else dl->AddLine(p0, p1, col, p.thickness);
        dl->AddCircleFilled(p0, 3.5f, col);
        dl->AddCircleFilled(p1, 3.5f, col);
    }

    // Draw pins
    for (const auto& [name, pt] : m_symbol_state->sym.pins) {
        ImVec2 p = to_canvas(pt.first, pt.second);
        dl->AddCircleFilled(p, 5.0f, IM_COL32(255, 200, 100, 255));
        dl->AddCircle(p, 5.0f, IM_COL32(255, 255, 255, 200), 12, 1.0f);
        dl->AddText(ImVec2(p.x + 7, p.y - 7), IM_COL32(255, 255, 255, 220), name.c_str());
    }

    ImGui::EndChild();
}

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
                    ImGui::Text("Voltage Rating: %.1f V", motor->get_voltage_rating());
                    ImGui::Text("No-Load RPM: %.0f", motor->get_no_load_speed());
                    ImGui::Text("Stall Torque: %.2f Nm", motor->get_stall_torque());
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
                    float input_voltage = 0.0f;
                    float duty_cycle = 0.0f;
                    if (auto* buck_comp = dynamic_cast<BuckConverterComponent*>(comp)) {
                        duty_cycle = static_cast<float>(buck_comp->circuit_component()->get_parameter("duty_cycle"));
                        for (auto* pin : buck_comp->circuit_component()->get_pins()) {
                            if (pin && pin->id == "VIN") {
                                input_voltage = pin->voltage;
                                break;
                            }
                        }
                    }
                    ImGui::Text("Input Voltage: %.2f V", input_voltage);
                    ImGui::Text("Duty Cycle: %.0f%%", duty_cycle * 100.0f);
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
                    float mcu_voltage = 0.0f;
                    if (cat == "mcu" && comp->get_mcu_pin_output_voltage(port->name(), mcu_voltage)) {
                        val_str = std::to_string(mcu_voltage);
                    } else if (const float* f = std::get_if<float>(&val)) {
                        val_str = std::to_string(*f);
                    } else if (const double* d = std::get_if<double>(&val)) {
                        val_str = std::to_string(*d);
                    } else if (const bool* b = std::get_if<bool>(&val)) {
                        val_str = *b ? "5.000000" : "0.000000";
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
