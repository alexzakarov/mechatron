#pragma once

#include "Viewport3D.hpp"
#include "PropertiesPanel.hpp"
#include "TimelinePanel.hpp"
#include "CircuitEditor.hpp"
#include "CodeEditor.hpp"
#include "ModelEditor.hpp"
#include "SerialMonitor.hpp"
#include <memory>

struct GLFWwindow;

namespace mechatron {

class Renderer;
class SimulationOrchestrator;

class UIApplication {
public:
    UIApplication();
    ~UIApplication();

    bool init();
    void shutdown();
    void run();

private:
    void render_menu_bar();
    void render_component_tree();
    bool render_splitter_v(float* size, float min_size, float max_size, float height = 0.0f);  // Vertical splitter (left/right)
    bool render_splitter_h(float* size, float min_size, float max_size, float width = 0.0f);  // Horizontal splitter (top/bottom)

    GLFWwindow* m_window = nullptr;
    std::unique_ptr<Renderer> m_renderer;
    std::unique_ptr<SimulationOrchestrator> m_orchestrator;

    Viewport3D m_viewport;
    PropertiesPanel m_properties;
    TimelinePanel m_timeline;
    CircuitEditor m_circuit_editor;
    CodeEditor m_code_editor;
    ModelEditor m_model_editor;
    SerialMonitor m_serial_monitor;

    bool m_running = false;
    bool m_show_demo = false;

    // Panel sizes (for splitter)
    float m_left_panel_width = 250.0f;
    float m_right_panel_width = 280.0f;
    float m_bottom_panel_height = 250.0f;  // Height of bottom panel area
};

} // namespace mechatron
