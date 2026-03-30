#pragma once

#include "Viewport3D.hpp"
#include "PropertiesPanel.hpp"
#include "TimelinePanel.hpp"
#include "CircuitEditor.hpp"
#include "CodeEditor.hpp"
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
    void setup_dockspace();
    void render_menu_bar();
    void render_component_tree();

    GLFWwindow* m_window = nullptr;
    std::unique_ptr<Renderer> m_renderer;
    std::unique_ptr<SimulationOrchestrator> m_orchestrator;

    Viewport3D m_viewport;
    PropertiesPanel m_properties;
    TimelinePanel m_timeline;
    CircuitEditor m_circuit_editor;
    CodeEditor m_code_editor;
    SerialMonitor m_serial_monitor;

    bool m_running = false;
    bool m_show_demo = false;
    bool m_show_circuit_editor = false;
    bool m_show_code_editor = false;
    bool m_show_serial_monitor = false;
};

} // namespace mechatron
