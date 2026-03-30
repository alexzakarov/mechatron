#pragma once

#include <GL/glew.h>
#include "Shader.hpp"
#include "Mesh.hpp"
#include "Camera.hpp"
#include "LineRenderer.hpp"
#include "GridRenderer.hpp"
#include "ComponentRenderer.hpp"
#include "GizmoRenderer.hpp"
#include <memory>

namespace mechatron {

struct RenderObject {
    std::string id;
    Mesh mesh;
    Vec3 position{};
    Vec3 scale{1, 1, 1};
    Vec3 color{0.8f, 0.8f, 0.8f};
};

class Renderer {
public:
    bool init();
    void shutdown();

    void begin_frame(float r = 0.1f, float g = 0.1f, float b = 0.15f, float a = 1.0f);
    void end_frame();

    void resize(int width, int height);

    // FBO for viewport rendering
    void create_framebuffer(int width, int height);
    void delete_framebuffer();
    void bind_framebuffer();
    void unbind_framebuffer();
    GLuint get_color_texture() const { return m_fbo_color_texture; }
    int fbo_width() const { return m_fbo_width; }
    int fbo_height() const { return m_fbo_height; }

    Camera& camera() { return m_camera; }

    // Object management
    RenderObject* add_object(std::string id, Mesh mesh, Vec3 position = {});
    void remove_object(std::string_view id);
    RenderObject* get_object(std::string_view id);

    void draw_grid(float size = 20.0f, int divisions = 20);

    LineRenderer& line_renderer() { return m_line_renderer; }
    GridRenderer* get_grid_renderer() { return &m_grid_renderer; }
    ComponentRenderer* get_component_renderer() { return &m_component_renderer; }
    GizmoRenderer* get_gizmo_renderer() { return &m_gizmo_renderer; }

    glm::mat4 view_matrix() const { return m_camera.view_matrix(); }
    glm::mat4 projection_matrix() const { return m_camera.projection_matrix(static_cast<float>(m_width) / std::max(1, m_height)); }

private:
    Camera m_camera;
    Shader m_main_shader;
    Shader m_line_shader;
    LineRenderer m_line_renderer;
    GridRenderer m_grid_renderer;
    ComponentRenderer m_component_renderer;
    GizmoRenderer m_gizmo_renderer;
    Mesh m_grid;
    std::unordered_map<std::string, std::unique_ptr<RenderObject>> m_objects;

    int m_width = 1280;
    int m_height = 720;

    // Framebuffer
    GLuint m_fbo = 0;
    GLuint m_fbo_color_texture = 0;
    GLuint m_fbo_depth_rbo = 0;
    int m_fbo_width = 0;
    int m_fbo_height = 0;
};

} // namespace mechatron
