#include "CircuitEditor.hpp"
#include "core/SimulationOrchestrator.hpp"
#include <imgui.h>
#include <spdlog/spdlog.h>

namespace mechatron {

void CircuitEditor::render(SimulationOrchestrator& orchestrator) {
    ImGui::Begin("Circuit Editor");

    // Menu bar
    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("Edit")) {
            if (ImGui::MenuItem("Clear All")) {
                m_nodes.clear();
                m_wires.clear();
                m_selected_node_index = -1;
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

    // Main layout: component palette | canvas | properties
    if (m_show_component_list) {
        ImGui::BeginChild("ComponentPalette", ImVec2(150, 0), true);
        render_component_palette();
        ImGui::EndChild();
        ImGui::SameLine();
    }

    // Circuit canvas
    ImGui::BeginChild("CircuitCanvas", ImVec2(m_show_properties ? -150 : 0, 0), true);
    render_circuit_canvas();
    ImGui::EndChild();

    if (m_show_properties) {
        ImGui::SameLine();
        ImGui::BeginChild("PropertiesPanel", ImVec2(150, 0), true);
        render_properties_panel();
        ImGui::EndChild();
    }

    ImGui::End();
}

void CircuitEditor::render_component_palette() {
    ImGui::Text("Components");
    ImGui::Separator();
    ImGui::InputText("Filter", m_component_filter, 64);

    ImGui::Spacing();

    // Basic components
    ImGui::TextDisabled("Passive");
    if (ImGui::Button("Resistor", ImVec2(120, 0))) add_node("resistor");
    if (ImGui::Button("Capacitor", ImVec2(120, 0))) add_node("capacitor");
    if (ImGui::Button("Inductor", ImVec2(120, 0))) add_node("inductor");

    ImGui::Spacing();
    ImGui::TextDisabled("Semiconductors");
    if (ImGui::Button("Diode", ImVec2(120, 0))) add_node("diode");
    if (ImGui::Button("LED", ImVec2(120, 0))) add_node("led");
    if (ImGui::Button("Transistor", ImVec2(120, 0))) add_node("transistor");

    ImGui::Spacing();
    ImGui::TextDisabled("Sources");
    if (ImGui::Button("DC Voltage", ImVec2(120, 0))) add_node("dc_voltage");
    if (ImGui::Button("Ground", ImVec2(120, 0))) add_node("ground");

    ImGui::Spacing();
    ImGui::TextDisabled("MCU");
    if (ImGui::Button("Arduino Uno", ImVec2(120, 0))) add_node("arduino_uno");

    ImGui::Spacing();
    ImGui::TextDisabled("Sensors");
    if (ImGui::Button("Limit Switch", ImVec2(120, 0))) add_node("limit_switch");
    if (ImGui::Button("Proximity", ImVec2(120, 0))) add_node("proximity_sensor");
}

void CircuitEditor::render_circuit_canvas() {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();

    // Create invisible button for canvas interaction
    ImGui::InvisibleButton("canvas", canvas_size,
        ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);

    const bool is_hovered = ImGui::IsItemHovered();
    const bool is_active = ImGui::IsItemActive();

    // Handle mouse input for node dragging
    if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        ImVec2 mouse_pos = ImGui::GetMousePos();
        float rel_x = mouse_pos.x - canvas_pos.x;
        float rel_y = mouse_pos.y - canvas_pos.y;

        int clicked_node = get_node_at_position(rel_x, rel_y);
        if (clicked_node >= 0) {
            m_dragging_node = true;
            m_dragged_node_index = clicked_node;
            m_drag_offset[0] = rel_x - m_nodes[clicked_node].position[0];
            m_drag_offset[1] = rel_y - m_nodes[clicked_node].position[1];
            m_selected_node_index = clicked_node;
        } else {
            // Check if clicking on a pin for wire creation
            for (size_t i = 0; i < m_nodes.size(); ++i) {
                int pin = get_pin_at_position(m_nodes[i], rel_x, rel_y);
                if (pin >= 0) {
                    m_creating_wire = true;
                    m_wire_start_node = m_nodes[i].id;
                    m_wire_start_pin = pin;
                    break;
                }
            }
        }
    }

    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        if (m_creating_wire) {
            ImVec2 mouse_pos = ImGui::GetMousePos();
            float rel_x = mouse_pos.x - canvas_pos.x;
            float rel_y = mouse_pos.y - canvas_pos.y;

            // Check if released on a pin
            for (const auto& node : m_nodes) {
                int pin = get_pin_at_position(node, rel_x, rel_y);
                if (pin >= 0 && node.id != m_wire_start_node) {
                    add_wire(m_wire_start_node, m_wire_start_pin, node.id, pin);
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

    // Draw grid
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

    // Draw wires
    ImU32 wire_color = ImGui::GetColorU32(ImVec4(0.8f, 0.8f, 0.2f, 1.0f));
    for (const auto& wire : m_wires) {
        // Find source and target nodes
        ImVec2 from_pos, to_pos;
        for (const auto& node : m_nodes) {
            if (node.id == wire.from_node) {
                from_pos = ImVec2(canvas_pos.x + node.position[0] + 60, canvas_pos.y + node.position[1] + 20 + wire.from_pin * 15);
            }
            if (node.id == wire.to_node) {
                to_pos = ImVec2(canvas_pos.x + node.position[0], canvas_pos.y + node.position[1] + 20 + wire.to_pin * 15);
            }
        }
        draw_list->AddLine(from_pos, to_pos, wire_color, 2.0f);
    }

    // Draw wire being created
    if (m_creating_wire) {
        ImVec2 mouse_pos = ImGui::GetMousePos();
        for (const auto& node : m_nodes) {
            if (node.id == m_wire_start_node) {
                ImVec2 start = ImVec2(canvas_pos.x + node.position[0] + 60, canvas_pos.y + node.position[1] + 20 + m_wire_start_pin * 15);
                draw_list->AddLine(start, mouse_pos, wire_color, 2.0f);
                break;
            }
        }
    }

    // Draw nodes
    for (size_t i = 0; i < m_nodes.size(); ++i) {
        const auto& node = m_nodes[i];
        ImVec2 node_pos(canvas_pos.x + node.position[0], canvas_pos.y + node.position[1]);

        // Node background
        ImU32 bg_color = (static_cast<int>(i) == m_selected_node_index) ?
            ImGui::GetColorU32(ImVec4(0.3f, 0.5f, 0.7f, 0.8f)) :
            ImGui::GetColorU32(ImVec4(0.2f, 0.2f, 0.2f, 0.9f));

        draw_list->AddRectFilled(node_pos, ImVec2(node_pos.x + 60, node_pos.y + 40), bg_color, 4.0f);
        draw_list->AddRect(node_pos, ImVec2(node_pos.x + 60, node_pos.y + 40), IM_COL32(255, 255, 255, 255), 4.0f, 0, 1.5f);

        // Node label
        const char* label = node.type.c_str();
        draw_list->AddText(ImVec2(node_pos.x + 5, node_pos.y + 5), IM_COL32(255, 255, 255, 255), label);

        // Draw pins (simplified)
        for (int p = 0; p < 2; ++p) {
            ImVec2 pin_pos(node_pos.x, node_pos.y + 20 + p * 15);
            draw_list->AddCircleFilled(pin_pos, 3.0f, IM_COL32(255, 200, 100, 255));
        }
        // Output pins on right
        for (int p = 0; p < 2; ++p) {
            ImVec2 pin_pos(node_pos.x + 60, node_pos.y + 20 + p * 15);
            draw_list->AddCircleFilled(pin_pos, 3.0f, IM_COL32(100, 200, 255, 255));
        }
    }
}

void CircuitEditor::render_properties_panel() {
    ImGui::Text("Properties");
    ImGui::Separator();

    if (m_selected_node_index < 0 || m_selected_node_index >= static_cast<int>(m_nodes.size())) {
        ImGui::TextDisabled("No component selected");
        return;
    }

    auto& node = m_nodes[m_selected_node_index];

    ImGui::Text("ID: %s", node.id.c_str());
    ImGui::Text("Type: %s", node.type.c_str());

    ImGui::Spacing();
    ImGui::Text("Position: %.1f, %.1f", node.position[0], node.position[1]);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Text("Parameters");

    // Type-specific parameters
    if (node.type == "resistor") {
        ImGui::InputDouble("Resistance (Ω)", &node.params.resistance, 100.0, 1000.0, "%.1f");
    } else if (node.type == "capacitor") {
        ImGui::InputDouble("Capacitance (µF)", &node.params.capacitance, 1e-7, 1e-6, "%.3f");
    } else if (node.type == "inductor") {
        ImGui::InputDouble("Inductance (mH)", &node.params.inductance, 1e-4, 1e-3, "%.3f");
    } else if (node.type == "led") {
        ImGui::InputDouble("Forward V", &node.params.forward_voltage, 0.1, 0.5, "%.2f");
        ImGui::InputDouble("Max Current", &node.params.max_current, 0.005, 0.01, "%.3f");
    }

    ImGui::Spacing();
    ImGui::Separator();
    if (ImGui::Button("Delete Component", ImVec2(130, 0))) {
        remove_node(m_selected_node_index);
    }
}

void CircuitEditor::add_node(const std::string& type) {
    CircuitNode node;
    node.id = type + "_" + std::to_string(m_nodes.size());
    node.type = type;
    node.position[0] = 50.0f + (m_nodes.size() % 5) * 80.0f;
    node.position[1] = 50.0f + (m_nodes.size() / 5) * 60.0f;

    // Set default parameters based on type
    if (type == "resistor") {
        node.params.resistance = 1000.0;
    } else if (type == "led") {
        node.params.forward_voltage = 2.0;
        node.params.max_current = 0.02;
    }

    m_nodes.push_back(node);
    spdlog::info("Added component: {} ({})", node.id, type);
}

void CircuitEditor::remove_node(int index) {
    if (index >= 0 && index < static_cast<int>(m_nodes.size())) {
        spdlog::info("Removed component: {}", m_nodes[index].id);

        // Remove connected wires
        auto it = std::remove_if(m_wires.begin(), m_wires.end(),
            [&index, this](const WireConnection& w) {
                return w.from_node == m_nodes[index].id || w.to_node == m_nodes[index].id;
            });
        m_wires.erase(it, m_wires.end());

        m_nodes.erase(m_nodes.begin() + index);
        m_selected_node_index = -1;
    }
}

void CircuitEditor::add_wire(const std::string& from_node, int from_pin,
                             const std::string& to_node, int to_pin) {
    WireConnection wire;
    wire.from_node = from_node;
    wire.from_pin = from_pin;
    wire.to_node = to_node;
    wire.to_pin = to_pin;
    m_wires.push_back(wire);
    spdlog::info("Added wire: {}[pin{}] -> {}[pin{}]", from_node, from_pin, to_node, to_pin);
}

void CircuitEditor::remove_wire(int index) {
    if (index >= 0 && index < static_cast<int>(m_wires.size())) {
        spdlog::info("Removed wire");
        m_wires.erase(m_wires.begin() + index);
    }
}

int CircuitEditor::get_node_at_position(float x, float y) {
    for (size_t i = 0; i < m_nodes.size(); ++i) {
        const auto& node = m_nodes[i];
        if (x >= node.position[0] && x <= node.position[0] + 60 &&
            y >= node.position[1] && y <= node.position[1] + 40) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

int CircuitEditor::get_pin_at_position(const CircuitNode& node, float x, float y) {
    // Check input pins (left side)
    for (int p = 0; p < 2; ++p) {
        float pin_x = node.position[0];
        float pin_y = node.position[1] + 20 + p * 15;
        float dist = sqrt((x - pin_x) * (x - pin_x) + (y - pin_y) * (y - pin_y));
        if (dist < 8.0f) return p;
    }
    // Check output pins (right side)
    for (int p = 0; p < 2; ++p) {
        float pin_x = node.position[0] + 60;
        float pin_y = node.position[1] + 20 + p * 15;
        float dist = sqrt((x - pin_x) * (x - pin_x) + (y - pin_y) * (y - pin_y));
        if (dist < 8.0f) return p + 100;  // Offset for output pins
    }
    return -1;
}

} // namespace mechatron
