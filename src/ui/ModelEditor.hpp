#pragma once

#include "cad/CADKernel.hpp"
#include "cad/ModelAssetLibrary.hpp"
#include "cad/ModifierEngine.hpp"
#include "cad/Modifiers.hpp"
#include "cad/EditableMesh.hpp"
#include <string>
#include <unordered_map>
#include <array>
#include <vector>

namespace mechatron {

class SimulationOrchestrator;

// ============================================================================
// Blender-like Model Editor Tab
//
// Layout:
//   Top: Toolbar (select mode, transform tool, mesh ops, undo/redo)
//   Left: Tool Shelf (create primitives, import)
//   Center: 3D Viewport (with overlay for wireframe, grid, selection highlights)
//   Right: Properties Panel (mesh info, transform, modifiers, material)
//   Bottom: Status bar (vertex/edge/face counts, selection info)
// ============================================================================
class ModelEditor {
public:
    void render(SimulationOrchestrator& orchestrator);

private:
    CADKernel m_cad;
    ModelAssetLibrary m_lib;
    bool m_loaded = false;

    // Import
    std::string m_import_path;
    std::string m_asset_id;
    std::string m_error;

    // Selection
    std::string m_selected_component_type;
    std::string m_selected_asset_id;
    std::string m_selected_asset_scope = "user";
    std::string m_loaded_mesh_source_label;
    bool m_force_fallback_box = false;

    // Modifier stack editing (per selected asset, in-memory for now).
    ModifierEngine m_mod_engine;
    ModifierStack m_stack;
    bool m_preview_valid = false;
    MeshData m_preview_mesh;
    std::string m_preview_error;
    bool m_show_modifier_preview = true;

    // Editable mesh state (for selected asset)
    std::string m_edit_asset_id;
    EditableMesh m_edit_mesh;
    bool m_edit_loaded = false;

    // Transform tool state
    enum class TransformMode { Translate = 0, Rotate = 1, Scale = 2 };
    TransformMode m_transform_mode = TransformMode::Translate;
    float m_transform_delta[3] = {0, 0, 0};
    float m_rotate_angle = 0.0f;
    float m_scale_factor[3] = {1, 1, 1};
    Vec3 m_transform_pivot{0, 0, 0};
    bool m_snap_enabled = false;
    float m_snap_increment = 0.1f;
    int m_axis_constraint = 0; // 0 none, 1 X, 2 Y, 3 Z

    // Viewport state
    struct ViewportState {
        bool inited = false;
        // FBO
        unsigned int fbo = 0;
        unsigned int tex = 0;
        unsigned int rbo = 0;
        int w = 0, h = 0;
        // Camera
        float yaw = 0.6f;
        float pitch = 0.35f;
        float dist = 3.0f;
        float target[3] = {0, 0, 0};
        // Projection cache for picking
        float view_matrix[16] = {};
        float proj_matrix[16] = {};
        int vp[4] = {};
        // Shader/mesh
        void* shader = nullptr;
        void* mesh = nullptr;
        void* wire_mesh = nullptr;
        bool mesh_valid = false;
        // 2D view
        float pan_x = 0.0f;
        float pan_y = 0.0f;
        float zoom_2d = 60.0f;
        // Display options
        bool show_wireframe = true;
        bool show_grid = true;
        bool show_normals = false;
        // View mode: 0=3D, 1=2D (top), 2=2D (front), 3=2D (side)
        int view_mode = 0;
    };
    ViewportState m_vp;

    // Tool state
    int m_active_tool = 0; // 0=select, 1=box select, 2=circle select, 3=lasso select
    bool m_box_selecting = false;
    float m_box_start[2] = {};
    float m_box_end[2] = {};
    bool m_circle_selecting = false;
    float m_circle_radius = 20.0f;
    bool m_lasso_selecting = false;
    std::vector<std::array<float, 2>> m_lasso_points;
    SelectionOp m_select_op = SelectionOp::Replace;
    bool m_selection_started = false;
    bool m_selection_cleared = false;

    // ---- Internal methods ----
    void render_toolbar();
    void render_tool_shelf();
    void render_viewport(SimulationOrchestrator& orchestrator);
    void render_properties_panel();
    void render_status_bar();
    void handle_hotkeys();

    void ensure_viewport_resources();
    void update_viewport_mesh();
    void render_3d_view(int pw, int ph);
    void render_2d_view(int pw, int ph);
    void frame_current_mesh();
    void frame_selection();

    // Picking
    int pick_vertex_3d(float screen_x, float screen_y);
    EdgeKey pick_edge_3d(float screen_x, float screen_y);
    int pick_face_3d(float screen_x, float screen_y);

    // Project world to screen (for 3D viewport)
    bool project_to_screen(const Vec3& world, float& sx, float& sy);

    // Load effective model for component type
    void load_effective_model(SimulationOrchestrator& orchestrator);
};

} // namespace mechatron
