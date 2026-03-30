#pragma once

#include <string>
#include <vector>
#include <memory>

namespace mechatron {

class SimulationOrchestrator;

class CircuitEditor {
public:
    void render(SimulationOrchestrator& orchestrator);

private:
    // Circuit node representation for UI
    struct CircuitNode {
        std::string id;
        std::string type;  // "resistor", "capacitor", "led", etc.
        float position[2] = {0, 0};  // Screen position
        bool selected = false;

        // Component-specific parameters
        struct Params {
            double resistance = 1000.0;    // ohms (resistor)
            double capacitance = 1e-6;     // farads (capacitor)
            double inductance = 1e-3;      // henries (inductor)
            double forward_voltage = 2.0;  // volts (LED)
            double max_current = 0.02;     // amps (LED)
        } params;
    };

    // Wire connection between nodes
    struct WireConnection {
        std::string from_node;
        int from_pin;     // Pin index on source node
        std::string to_node;
        int to_pin;       // Pin index on target node
    };

    std::vector<CircuitNode> m_nodes;
    std::vector<WireConnection> m_wires;

    // UI state
    bool m_show_component_list = true;
    bool m_show_properties = true;
    char m_component_filter[64] = "";
    int m_selected_node_index = -1;

    // Dragging state
    bool m_dragging_node = false;
    int m_dragged_node_index = -1;
    float m_drag_offset[2] = {0, 0};

    // Wire creation state
    bool m_creating_wire = false;
    std::string m_wire_start_node;
    int m_wire_start_pin = -1;

    void render_component_palette();
    void render_circuit_canvas();
    void render_properties_panel();

    void add_node(const std::string& type);
    void remove_node(int index);
    void add_wire(const std::string& from_node, int from_pin, const std::string& to_node, int to_pin);
    void remove_wire(int index);

    int get_node_at_position(float x, float y);
    int get_pin_at_position(const CircuitNode& node, float x, float y);
};

} // namespace mechatron
