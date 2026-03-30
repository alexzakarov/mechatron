#pragma once

#include <string>

namespace mechatron {

class Renderer;
class SimulationOrchestrator;
class GridRenderer;
class ComponentRenderer;
class GizmoRenderer;

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

private:
    void render_toolbar();
    void handle_camera_input();
    void handle_gizmo_input(Renderer& renderer, SimulationOrchestrator& orchestrator);
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

    // Stored renderer pointer for camera access
    Renderer* m_renderer = nullptr;
    SimulationOrchestrator* m_orchestrator = nullptr;

    // Context menu state
    bool m_show_context_menu = false;
};

} // namespace mechatron
