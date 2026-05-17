#pragma once

#include "Shader.hpp"
#include "core/Types.hpp"
#include <GL/glew.h>
#include <memory>

namespace mechatron {

/**
 * @brief Grid and axes renderer for 3D viewport
 *
 * Renders:
 * - Ground grid with customizable size and divisions
 * - X/Y/Z axes with colors (Red=X, Green=Y, Blue=Z)
 * - Minor and major grid lines
 */
class GridRenderer {
public:
    GridRenderer();
    ~GridRenderer();

    bool init();
    void shutdown();

    /**
     * Render the grid and axes
     * @param view View matrix
     * @param proj Projection matrix
     * @param size Grid size in world units
     * @param divisions Number of divisions
     */
    void render(const glm::mat4& view, const glm::mat4& proj,
                float size = 20.0f, int divisions = 20);

    /**
     * Render only the coordinate axes
     */
    void render_axes(const glm::mat4& view, const glm::mat4& proj, float size = 1.0f);

    void set_grid_color(float r, float g, float b) { m_grid_color = {r, g, b}; }
    void set_axes_visible(bool visible) { m_axes_visible = visible; }
    void set_grid_visible(bool visible) { m_grid_visible = visible; }

    // Theme-configurable default grid color
    static Vec3& default_grid_color() {
        static Vec3 def{0.3f, 0.3f, 0.3f};
        return def;
    }
    static void set_default_grid_color(const Vec3& color) { default_grid_color() = color; }

private:
    void create_grid_mesh(float size, int divisions);
    void create_axes_mesh();

    Shader m_shader;
    GLuint m_grid_vao = 0;
    GLuint m_grid_vbo = 0;
    GLuint m_axes_vao = 0;
    GLuint m_axes_vbo = 0;

    int m_grid_vertex_count = 0;
    int m_axes_vertex_count = 0;

    Vec3 m_grid_color = default_grid_color();
    bool m_axes_visible = true;
    bool m_grid_visible = true;
};

} // namespace mechatron
