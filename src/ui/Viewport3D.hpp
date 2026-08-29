#pragma once

#include <string>
#include <vector>
#include <imgui.h>
#include "core/Types.hpp"
#include "OscilloscopePanel.hpp"

namespace mechatron {

class Renderer;
class SimulationOrchestrator;
class GridRenderer;
class ComponentRenderer;
class GizmoRenderer;
class CircuitEditor;
class CodeEditor;
class ModelEditor;

/**
 * @brief 3D Viewport panel for scene visualization
 *
 * Features:
 * - Grid and axes display
 * - Component rendering
 * - Camera controls (orbit, pan, zoom)
 * - Transform gizmo for selected object
 * - Toolbar for common actions
 */
class Viewport3D {
public:
    Viewport3D();
    ~Viewport3D();

    void init();
    void shutdown();

    void render(Renderer& renderer, SimulationOrchestrator& orchestrator);

    bool is_hovered() const { return m_hovered; }
    bool is_focused() const { return m_focused; }

    void get_size(int& w, int& h) const { w = m_width; h = m_height; }

    // Grid settings
    void set_grid_visible(bool visible) { m_grid_visible = visible; }
    void set_axes_visible(bool visible) { m_axes_visible = visible; }

    // Set editor references
    void set_circuit_editor(CircuitEditor* editor) { m_circuit_editor = editor; }
    void set_code_editor(CodeEditor* editor) { m_code_editor = editor; }
    void set_model_editor(ModelEditor* editor) { m_model_editor = editor; }

private:
    void render_3d_viewport(Renderer& renderer, SimulationOrchestrator& orchestrator);
    void render_circuit_editor_tab(SimulationOrchestrator& orchestrator);
    void render_code_editor_tab(SimulationOrchestrator& orchestrator);
    void render_model_editor_tab(SimulationOrchestrator& orchestrator);
    void render_toolbar();
    void render_oscilloscope_tab(SimulationOrchestrator& orchestrator);
    void handle_camera_input();
    void render_gizmo(Renderer& renderer, SimulationOrchestrator& orchestrator);
    void handle_gizmo_input_after_image(Renderer& renderer, SimulationOrchestrator& orchestrator);
    void handle_selection();
    void render_context_menu();

    // Viewport state
    int m_width = 800;
    int m_height = 600;
    bool m_hovered = false;
    bool m_focused = false;

    // Display settings
    bool m_grid_visible = true;
    bool m_axes_visible = true;
    bool m_wireframe = false;

    // Gizmo state
    bool m_gizmo_active = false;
    int m_gizmo_mode = 0; // 0=Translate, 1=Rotate, 2=Scale
    float m_image_pos_x = 0, m_image_pos_y = 0; // Position of rendered image for input calculation
    float m_image_size_x = 0, m_image_size_y = 0; // Actual size of rendered image (may differ from viewport due to toolbar)
    Vec3 m_gizmo_start_position{}; // Component position at drag start
    Vec3 m_gizmo_start_scale{}; // Component scale at drag start
    Quat m_gizmo_start_rotation{}; // Component rotation at drag start

    // Stored renderer pointer for camera access
    Renderer* m_renderer = nullptr;
    SimulationOrchestrator* m_orchestrator = nullptr;

    // Editor references
    CircuitEditor* m_circuit_editor = nullptr;
    CodeEditor* m_code_editor = nullptr;
    ModelEditor* m_model_editor = nullptr;

    // Context menu state
    bool m_show_context_menu = false;

    // Oscilloscope panels (one per opened scope)
    std::vector<OscilloscopePanel> m_oscilloscope_panels;
};

} // namespace mechatron
