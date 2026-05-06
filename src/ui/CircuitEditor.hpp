#pragma once

#include <string>
#include <vector>
#include <map>
#include <imgui.h>

namespace mechatron {

class SimulationOrchestrator;

class CircuitEditor {
public:
    void render(SimulationOrchestrator& orchestrator);

private:
    enum class PinDirection { None, Input, Output };

    // Pin info for rendering and interaction
    struct PinInfo {
        std::string name;          // Port name (e.g. "anode", "cathode")
        int port_index;            // Index in component's port list
        bool is_input;             // true = input, false = output
    };

    // Circuit node for 2D canvas
    struct CircuitNode {
        std::string id;
        std::string type;
        std::string plugin_name;
        float position[2] = {0, 0};
        bool selected = false;
        std::vector<PinInfo> pins; // Actual pins from component ports
        bool pins_dirty = true;    // Need to refresh pins from Registry
    };

    // Wire between nodes
    struct WireConnection {
        std::string uid;  // Unique identifier for wire
        std::string from_node;
        std::string from_pin_name;
        std::string to_node;
        std::string to_pin_name;
    };

    std::vector<CircuitNode> m_nodes;
    std::vector<WireConnection> m_wires;

    // Selection
    int m_selected_node_index = -1;
    std::string m_selected_node_id;
    int m_selected_wire_index = -1;   // Wire selection

    // Dragging
    bool m_dragging_node = false;
    int m_dragged_node_index = -1;
    float m_drag_offset[2] = {0, 0};

    // Wire creation
    bool m_creating_wire = false;
    std::string m_wire_start_node;
    std::string m_wire_start_pin;
    PinDirection m_wire_start_pin_dir = PinDirection::None;

    // Wire context menu state
    bool m_wire_context_menu_pending = false;
    ImVec2 m_wire_context_menu_pos = ImVec2(0, 0);

    // Orchestrator pointer (set during render)
    SimulationOrchestrator* m_orchestrator = nullptr;

    // UI state
    bool m_show_component_list = true;
    bool m_show_properties = true;
    char m_component_filter[64] = "";
    int m_next_node_num = 0;
    int m_next_wire_uid = 0;  // Wire UID generator

    void render_component_palette();
    void render_circuit_canvas();
    void render_properties_panel();

    void refresh_node_pins(CircuitNode& node);

    void add_node(const std::string& type);
    void add_esc_template();  // ESC 3-phase inverter template
    void remove_node(int index);
    bool add_wire(const std::string& from_node, const std::string& from_pin,
                  const std::string& to_node, const std::string& to_pin,
                  PinDirection from_dir, PinDirection to_dir);
    void remove_wire(int index);

    int get_node_at_position(float x, float y);
    const PinInfo* get_pin_at_position(const CircuitNode& node, float x, float y);
    ImVec2 get_pin_canvas_pos(const CircuitNode& node, const std::string& pin_name, ImVec2 canvas_pos);
    int get_wire_at_position(float x, float y, ImVec2 canvas_pos);
    int find_node_by_id(const std::string& id) const;

    // Registry sync
    void sync_from_registry();
    void sync_selection_to_orchestrator();
    void sync_selection_from_orchestrator();

    static std::pair<std::string, std::string> map_type_to_plugin(const std::string& type);
};

} // namespace mechatron
