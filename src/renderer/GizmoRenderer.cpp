#include "GizmoRenderer.hpp"
#include <spdlog/spdlog.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace mechatron {

static const char* gizmo_vertex_shader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

uniform mat4 uView;
uniform mat4 uProj;
uniform vec3 uPosition;
uniform float uSize;

out vec3 vColor;

void main() {
    vec3 worldPos = aPos * uSize + uPosition;
    gl_Position = uProj * uView * vec4(worldPos, 1.0);
    vColor = aColor;
}
)";

static const char* gizmo_fragment_shader = R"(
#version 330 core
in vec3 vColor;
out vec4 FragColor;

void main() {
    FragColor = vec4(vColor, 1.0);
}
)";

GizmoRenderer::GizmoRenderer() = default;

GizmoRenderer::~GizmoRenderer() {
    shutdown();
}

bool GizmoRenderer::init() {
    if (!m_shader.load_from_source(gizmo_vertex_shader, gizmo_fragment_shader)) {
        spdlog::error("GizmoRenderer: Failed to compile shader");
        return false;
    }

    create_translate_gizmo();
    create_rotate_gizmo();
    create_scale_gizmo();

    return true;
}

void GizmoRenderer::shutdown() {
    if (m_vao) glDeleteVertexArrays(1, &m_vao);
    if (m_vbo) glDeleteBuffers(1, &m_vbo);
    m_vao = m_vbo = 0;
}

void GizmoRenderer::create_translate_gizmo() {
    std::vector<float> vertices;

    auto add_arrow = [&](const Vec3& dir, const Vec3& color) {
        // Shaft
        vertices.insert(vertices.end(), {
            0, 0, 0,  color.x, color.y, color.z,
            dir.x, dir.y, dir.z,  color.x, color.y, color.z
        });

        // Arrow head
        Vec3 tip = dir;
        glm::vec3 glm_dir(dir.x, dir.y, dir.z);
        glm::vec3 glm_perp1 = std::abs(dir.x) > 0.5f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
        glm::vec3 glm_perp2 = glm::cross(glm_dir, glm_perp1);
        glm_perp1 = glm::cross(glm_dir, glm_perp2);

        glm::vec3 normPerp1 = glm::normalize(glm_perp1);
        glm::vec3 normPerp2 = glm::normalize(glm_perp2);
        glm::vec3 normDir = glm::normalize(glm_dir);

        Vec3 perp1{normPerp1.x * 0.1f, normPerp1.y * 0.1f, normPerp1.z * 0.1f};
        Vec3 perp2{normPerp2.x * 0.1f, normPerp2.y * 0.1f, normPerp2.z * 0.1f};
        Vec3 back{dir.x - normDir.x * 0.15f, dir.y - normDir.y * 0.15f, dir.z - normDir.z * 0.15f};

        // Arrow head pyramid
        vertices.insert(vertices.end(), {
            tip.x, tip.y, tip.z,  color.x, color.y, color.z,
            back.x + perp1.x, back.y + perp1.y, back.z + perp1.z,  color.x, color.y, color.z,
            back.x - perp1.x, back.y - perp1.y, back.z - perp1.z,  color.x, color.y, color.z,

            tip.x, tip.y, tip.z,  color.x, color.y, color.z,
            back.x + perp2.x, back.y + perp2.y, back.z + perp2.z,  color.x, color.y, color.z,
            back.x - perp2.x, back.y - perp2.y, back.z - perp2.z,  color.x, color.y, color.z
        });
    };

    add_arrow({1, 0, 0}, m_color_x);
    add_arrow({0, 1, 0}, m_color_y);
    add_arrow({0, 0, 1}, m_color_z);

    m_vertex_count = vertices.size() / 6;

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);

    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

    glBindVertexArray(0);
}

void GizmoRenderer::create_rotate_gizmo() {
    // TODO: Implement rotation circles
}

void GizmoRenderer::create_scale_gizmo() {
    // TODO: Implement scale handles
}

void GizmoRenderer::render(const Camera& camera, const Vec3& position, GizmoMode mode, float aspect) {
    m_shader.bind();
    m_shader.set_uniform("uView", camera.view_matrix());
    m_shader.set_uniform("uProj", camera.projection_matrix(aspect));
    m_shader.set_uniform("uPosition", glm::vec3(position.x, position.y, position.z));
    m_shader.set_uniform("uSize", m_size);

    glBindVertexArray(m_vao);
    glDrawArrays(GL_LINES, 0, m_vertex_count);
    glDrawArrays(GL_TRIANGLES, m_vertex_count - 12, 12);
    glBindVertexArray(0);
}

bool GizmoRenderer::on_mouse_down(float x, float y, const Camera& camera, const Vec3& position) {
    m_click_start_x = x;
    m_click_start_y = y;
    m_is_dragging = true;
    m_active_axis = m_hover_axis;

    // Reset deltas
    m_translation_delta = {};
    m_rotation_delta = {};
    m_scale_delta = {1, 1, 1};

    return m_active_axis != GizmoAxis::None;
}

void GizmoRenderer::on_mouse_move(float x, float y, const Camera& camera, const Vec3& position) {
    if (!m_is_dragging) {
        // Check for hover
        // TODO: Implement hover detection
        return;
    }

    float dx = x - m_click_start_x;
    float dy = y - m_click_start_y;

    float sensitivity = 0.01f;

    switch (m_mode) {
    case GizmoMode::Translate:
        switch (m_active_axis) {
        case GizmoAxis::X:
            m_translation_delta.x = (dx + dy) * sensitivity;
            break;
        case GizmoAxis::Y:
            m_translation_delta.y = (dx + dy) * sensitivity;
            break;
        case GizmoAxis::Z:
            m_translation_delta.z = (dx + dy) * sensitivity;
            break;
        default:
            break;
        }
        break;

    case GizmoMode::Rotate:
        switch (m_active_axis) {
        case GizmoAxis::X:
            m_rotation_delta.x = (dx + dy) * sensitivity;
            break;
        case GizmoAxis::Y:
            m_rotation_delta.y = (dx + dy) * sensitivity;
            break;
        case GizmoAxis::Z:
            m_rotation_delta.z = (dx + dy) * sensitivity;
            break;
        default:
            break;
        }
        break;

    case GizmoMode::Scale:
        switch (m_active_axis) {
        case GizmoAxis::X:
            m_scale_delta.x = 1.0f + (dx + dy) * sensitivity;
            break;
        case GizmoAxis::Y:
            m_scale_delta.y = 1.0f + (dx + dy) * sensitivity;
            break;
        case GizmoAxis::Z:
            m_scale_delta.z = 1.0f + (dx + dy) * sensitivity;
            break;
        default:
            break;
        }
        break;
    }
}

void GizmoRenderer::on_mouse_up() {
    m_is_dragging = false;
    m_active_axis = GizmoAxis::None;
}

bool GizmoRenderer::ray_intersect_axis(const Vec3& ray_origin, const Vec3& ray_dir,
                                       const Vec3& axis_pos, const Vec3& axis_dir,
                                       float axis_length, float& t) const {
    // TODO: Implement ray-axis intersection
    return false;
}

Vec3 GizmoRenderer::screen_to_world(float x, float y, const Camera& camera) const {
    // TODO: Implement screen to world conversion
    return {};
}

} // namespace mechatron
