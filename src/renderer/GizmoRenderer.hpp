#pragma once

#include "Shader.hpp"
#include "Camera.hpp"
#include <GL/glew.h>
#include <memory>

namespace mechatron {

/**
 * @brief Transform gizmo for object manipulation
 *
 * Provides visual handles for:
 * - Translation (arrows along X/Y/Z)
 * - Rotation (circles around axes)
 * - Scale (handles at axis ends)
 */
enum class GizmoMode {
    Translate,
    Rotate,
    Scale
};

enum class GizmoAxis {
    None,
    X,
    Y,
    Z,
    XY,
    XZ,
    YZ,
    XYZ
};

class GizmoRenderer {
public:
    GizmoRenderer();
    ~GizmoRenderer();

    bool init();
    void shutdown();

    /**
     * Render the gizmo at target position
     */
    void render(const Camera& camera, const Vec3& position, GizmoMode mode, float aspect);

    /**
     * Handle mouse input for gizmo interaction
     * @return True if gizmo is being manipulated
     */
    bool on_mouse_down(float x, float y, const Camera& camera, const Vec3& position);
    void on_mouse_move(float x, float y, const Camera& camera, const Vec3& position);
    void on_mouse_up();

    /**
     * Get current interaction state
     */
    GizmoAxis get_active_axis() const { return m_active_axis; }
    GizmoMode get_mode() const { return m_mode; }
    void set_mode(GizmoMode mode) { m_mode = mode; }

    /**
     * Get transformation result
     */
    Vec3 get_translation_delta() const { return m_translation_delta; }
    Vec3 get_rotation_delta() const { return m_rotation_delta; }
    Vec3 get_scale_delta() const { return m_scale_delta; }

    void set_size(float size) { m_size = size; }

private:
    void create_translate_gizmo();
    void create_rotate_gizmo();
    void create_scale_gizmo();

    bool ray_intersect_axis(const Vec3& ray_origin, const Vec3& ray_dir,
                           const Vec3& axis_pos, const Vec3& axis_dir,
                           float axis_length, float& t) const;

    Vec3 screen_to_world(float x, float y, const Camera& camera) const;

    Shader m_shader;
    GLuint m_vao = 0;
    GLuint m_vbo = 0;

    int m_vertex_count = 0;

    GizmoMode m_mode = GizmoMode::Translate;
    GizmoAxis m_active_axis = GizmoAxis::None;
    GizmoAxis m_hover_axis = GizmoAxis::None;

    Vec3 m_translation_delta{};
    Vec3 m_rotation_delta{};
    Vec3 m_scale_delta{};

    float m_size = 1.0f;
    float m_click_start_x = 0, m_click_start_y = 0;
    bool m_is_dragging = false;

    // Axis colors
    Vec3 m_color_x{1.0f, 0.2f, 0.2f};  // Red
    Vec3 m_color_y{0.2f, 1.0f, 0.2f};  // Green
    Vec3 m_color_z{0.2f, 0.2f, 1.0f};  // Blue
    Vec3 m_color_hover{1.0f, 1.0f, 0.2f}; // Yellow
    Vec3 m_color_active{1.0f, 0.8f, 0.0f}; // Orange
};

} // namespace mechatron
