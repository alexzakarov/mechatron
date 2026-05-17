#include "GizmoRenderer.hpp"
#include <spdlog/spdlog.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
    if (m_rotate_vao) glDeleteVertexArrays(1, &m_rotate_vao);
    if (m_rotate_vbo) glDeleteBuffers(1, &m_rotate_vbo);
    if (m_scale_vao) glDeleteVertexArrays(1, &m_scale_vao);
    if (m_scale_vbo) glDeleteBuffers(1, &m_scale_vbo);
    m_vao = m_vbo = 0;
    m_rotate_vao = m_rotate_vbo = 0;
    m_scale_vao = m_scale_vbo = 0;
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
    m_translate_vertices = vertices;  // Store for color updates

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);

    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);  // DYNAMIC_DRAW for color updates

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

    glBindVertexArray(0);
}

void GizmoRenderer::create_rotate_gizmo() {
    std::vector<float> vertices;

    // Create a circle in the specified plane
    auto add_circle = [&](const Vec3& axis, const Vec3& color) {
        constexpr int segments = 64;
        constexpr float radius = 0.8f;

        // Find perpendicular vectors for the circle plane
        glm::vec3 glm_axis(axis.x, axis.y, axis.z);
        glm::vec3 glm_perp1 = std::abs(axis.x) > 0.5f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
        glm::vec3 glm_perp2 = glm::cross(glm_axis, glm_perp1);
        glm_perp1 = glm::cross(glm_axis, glm_perp2);

        glm::vec3 normPerp1 = glm::normalize(glm_perp1);
        glm::vec3 normPerp2 = glm::normalize(glm_perp2);

        // Create circle as line segments
        for (int i = 0; i < segments; ++i) {
            float angle1 = (2.0f * M_PI * i) / segments;
            float angle2 = (2.0f * M_PI * (i + 1)) / segments;

            Vec3 p1{
                normPerp1.x * std::cos(angle1) * radius + normPerp2.x * std::sin(angle1) * radius,
                normPerp1.y * std::cos(angle1) * radius + normPerp2.y * std::sin(angle1) * radius,
                normPerp1.z * std::cos(angle1) * radius + normPerp2.z * std::sin(angle1) * radius
            };

            Vec3 p2{
                normPerp1.x * std::cos(angle2) * radius + normPerp2.x * std::sin(angle2) * radius,
                normPerp1.y * std::cos(angle2) * radius + normPerp2.y * std::sin(angle2) * radius,
                normPerp1.z * std::cos(angle2) * radius + normPerp2.z * std::sin(angle2) * radius
            };

            vertices.insert(vertices.end(), {
                p1.x, p1.y, p1.z,  color.x, color.y, color.z,
                p2.x, p2.y, p2.z,  color.x, color.y, color.z
            });
        }
    };

    // Add circles for each axis rotation
    add_circle({1, 0, 0}, m_color_x);  // Rotation around X (YZ plane)
    add_circle({0, 1, 0}, m_color_y);  // Rotation around Y (XZ plane)
    add_circle({0, 0, 1}, m_color_z);  // Rotation around Z (XY plane)

    // Store rotation gizmo data
    m_rotate_vertices = vertices;  // Store for color updates
    m_rotate_vertex_count = vertices.size() / 6;

    glGenVertexArrays(1, &m_rotate_vao);
    glGenBuffers(1, &m_rotate_vbo);

    glBindVertexArray(m_rotate_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_rotate_vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

    glBindVertexArray(0);
}

void GizmoRenderer::update_gizmo_colors() {
    // Determine which color to use for each axis
    auto get_color = [&](int axis_index) -> Vec3 {
        GizmoAxis axis_to_check = static_cast<GizmoAxis>(axis_index);
        if (m_hover_axis == axis_to_check || m_active_axis == axis_to_check) {
            return m_color_hover;  // Yellow for hover/active
        }
        switch (axis_index) {
            case 1: return m_color_x;  // Red for X
            case 2: return m_color_y;  // Green for Y
            case 3: return m_color_z;  // Blue for Z
            case 4: return m_color_hover;  // XYZ center
            default: return m_color_generic;
        }
    };

    // Update translate gizmo colors
    if (!m_translate_vertices.empty()) {
        std::vector<float> updated = m_translate_vertices;
        // Each axis has similar structure - update colors based on hover
        // X axis (red)
        Vec3 color = get_color(1);
        for (size_t i = 0; i < updated.size(); i += 6) {
            // First 2 vertices are X shaft, next 6 are X arrow head (8 vertices total for X)
            // This is simplified - we'll just color the appropriate sections
        }
        // For simplicity, regenerate entire buffer with correct colors
        // This is less efficient but cleaner code
        std::vector<float> new_verts;

        auto add_arrow_colored = [&](const Vec3& dir, const Vec3& base_color, bool is_hovered) {
            Vec3 color = is_hovered ? m_color_hover : base_color;
            // Shaft
            new_verts.insert(new_verts.end(), {
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

            new_verts.insert(new_verts.end(), {
                tip.x, tip.y, tip.z,  color.x, color.y, color.z,
                back.x + perp1.x, back.y + perp1.y, back.z + perp1.z,  color.x, color.y, color.z,
                back.x - perp1.x, back.y - perp1.y, back.z - perp1.z,  color.x, color.y, color.z,

                tip.x, tip.y, tip.z,  color.x, color.y, color.z,
                back.x + perp2.x, back.y + perp2.y, back.z + perp2.z,  color.x, color.y, color.z,
                back.x - perp2.x, back.y - perp2.y, back.z - perp2.z,  color.x, color.y, color.z
            });
        };

        add_arrow_colored({1, 0, 0}, m_color_x, m_hover_axis == GizmoAxis::X || m_active_axis == GizmoAxis::X);
        add_arrow_colored({0, 1, 0}, m_color_y, m_hover_axis == GizmoAxis::Y || m_active_axis == GizmoAxis::Y);
        add_arrow_colored({0, 0, 1}, m_color_z, m_hover_axis == GizmoAxis::Z || m_active_axis == GizmoAxis::Z);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, new_verts.size() * sizeof(float), new_verts.data());
    }

    // Update rotate gizmo colors
    if (!m_rotate_vertices.empty()) {
        std::vector<float> new_verts;

        auto add_circle_colored = [&](const Vec3& axis, const Vec3& base_color, bool is_hovered) {
            Vec3 color = is_hovered ? m_color_hover : base_color;
            constexpr int segments = 64;
            constexpr float radius = 0.8f;

            glm::vec3 glm_axis(axis.x, axis.y, axis.z);
            glm::vec3 glm_perp1 = std::abs(axis.x) > 0.5f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
            glm::vec3 glm_perp2 = glm::cross(glm_axis, glm_perp1);
            glm_perp1 = glm::cross(glm_axis, glm_perp2);

            glm::vec3 normPerp1 = glm::normalize(glm_perp1);
            glm::vec3 normPerp2 = glm::normalize(glm_perp2);

            for (int i = 0; i < segments; ++i) {
                float angle1 = (2.0f * M_PI * i) / segments;
                float angle2 = (2.0f * M_PI * (i + 1)) / segments;

                Vec3 p1{
                    normPerp1.x * std::cos(angle1) * radius + normPerp2.x * std::sin(angle1) * radius,
                    normPerp1.y * std::cos(angle1) * radius + normPerp2.y * std::sin(angle1) * radius,
                    normPerp1.z * std::cos(angle1) * radius + normPerp2.z * std::sin(angle1) * radius
                };

                Vec3 p2{
                    normPerp1.x * std::cos(angle2) * radius + normPerp2.x * std::sin(angle2) * radius,
                    normPerp1.y * std::cos(angle2) * radius + normPerp2.y * std::sin(angle2) * radius,
                    normPerp1.z * std::cos(angle2) * radius + normPerp2.z * std::sin(angle2) * radius
                };

                new_verts.insert(new_verts.end(), {
                    p1.x, p1.y, p1.z,  color.x, color.y, color.z,
                    p2.x, p2.y, p2.z,  color.x, color.y, color.z
                });
            }
        };

        add_circle_colored({1, 0, 0}, m_color_x, m_hover_axis == GizmoAxis::X || m_active_axis == GizmoAxis::X);
        add_circle_colored({0, 1, 0}, m_color_y, m_hover_axis == GizmoAxis::Y || m_active_axis == GizmoAxis::Y);
        add_circle_colored({0, 0, 1}, m_color_z, m_hover_axis == GizmoAxis::Z || m_active_axis == GizmoAxis::Z);

        glBindBuffer(GL_ARRAY_BUFFER, m_rotate_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, new_verts.size() * sizeof(float), new_verts.data());
    }

    // Update scale gizmo colors
    if (!m_scale_vertices.empty()) {
        std::vector<float> new_verts;

        auto add_scale_handle_colored = [&](const Vec3& dir, const Vec3& base_color, bool is_hovered) {
            Vec3 color = is_hovered ? m_color_hover : base_color;
            // Shaft line
            new_verts.insert(new_verts.end(), {
                0, 0, 0,  color.x, color.y, color.z,
                dir.x, dir.y, dir.z,  color.x, color.y, color.z
            });

            // Box handle at the end
            Vec3 center = dir;
            float box_size = 0.08f;

            glm::vec3 c(center.x, center.y, center.z);
            float hs = box_size * 0.5f;

            glm::vec3 glm_dir(dir.x, dir.y, dir.z);
            glm::vec3 glm_perp1 = std::abs(dir.x) > 0.5f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
            glm::vec3 glm_perp2 = glm::normalize(glm::cross(glm_dir, glm_perp1));
            glm_perp1 = glm::normalize(glm::cross(glm_dir, glm_perp2));

            std::vector<glm::vec3> box_verts = {
                c + glm_perp1 * hs + glm_perp2 * hs,
                c - glm_perp1 * hs + glm_perp2 * hs,
                c - glm_perp1 * hs - glm_perp2 * hs,
                c + glm_perp1 * hs - glm_perp2 * hs,
                c + glm_dir * hs,
                c - glm_dir * hs
            };

            for (int i = 0; i < 4; ++i) {
                int j = (i + 1) % 4;
                new_verts.insert(new_verts.end(), {
                    box_verts[4].x, box_verts[4].y, box_verts[4].z,  color.x, color.y, color.z,
                    box_verts[i].x, box_verts[i].y, box_verts[i].z,  color.x, color.y, color.z,
                    box_verts[j].x, box_verts[j].y, box_verts[j].z,  color.x, color.y, color.z
                });
                new_verts.insert(new_verts.end(), {
                    box_verts[5].x, box_verts[5].y, box_verts[5].z,  color.x, color.y, color.z,
                    box_verts[j].x, box_verts[j].y, box_verts[j].z,  color.x, color.y, color.z,
                    box_verts[i].x, box_verts[i].y, box_verts[i].z,  color.x, color.y, color.z
                });
            }
        };

        // Add scale handles with colors
        bool x_hover = m_hover_axis == GizmoAxis::X || m_active_axis == GizmoAxis::X;
        bool y_hover = m_hover_axis == GizmoAxis::Y || m_active_axis == GizmoAxis::Y;
        bool z_hover = m_hover_axis == GizmoAxis::Z || m_active_axis == GizmoAxis::Z;
        bool xyz_hover = m_hover_axis == GizmoAxis::XYZ || m_active_axis == GizmoAxis::XYZ;

        add_scale_handle_colored({1, 0, 0}, m_color_x, x_hover);
        add_scale_handle_colored({0, 1, 0}, m_color_y, y_hover);
        add_scale_handle_colored({0, 0, 1}, m_color_z, z_hover);

        // Center cube for uniform scale
        float cs = 0.06f;
        Vec3 c_color = xyz_hover ? m_color_hover : Vec3{1.0f, 1.0f, 0.2f};

        auto add_cube_face_colored = [&](const Vec3& normal, float offset) {
            glm::vec3 n(normal.x, normal.y, normal.z);
            glm::vec3 p1 = std::abs(normal.x) > 0.5f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
            glm::vec3 p2 = glm::normalize(glm::cross(n, p1));
            p1 = glm::cross(n, p2);

            glm::vec3 center = n * offset;
            std::vector<glm::vec3> corners = {
                center + (p1 + p2) * cs,
                center + (p1 - p2) * cs,
                center + (-p1 - p2) * cs,
                center + (-p1 + p2) * cs
            };

            new_verts.insert(new_verts.end(), {
                corners[0].x, corners[0].y, corners[0].z,  c_color.x, c_color.y, c_color.z,
                corners[1].x, corners[1].y, corners[1].z,  c_color.x, c_color.y, c_color.z,
                corners[2].x, corners[2].y, corners[2].z,  c_color.x, c_color.y, c_color.z,
                corners[0].x, corners[0].y, corners[0].z,  c_color.x, c_color.y, c_color.z,
                corners[2].x, corners[2].y, corners[2].z,  c_color.x, c_color.y, c_color.z,
                corners[3].x, corners[3].y, corners[3].z,  c_color.x, c_color.y, c_color.z
            });
        };

        add_cube_face_colored({1, 0, 0}, cs);
        add_cube_face_colored({-1, 0, 0}, cs);
        add_cube_face_colored({0, 1, 0}, cs);
        add_cube_face_colored({0, -1, 0}, cs);
        add_cube_face_colored({0, 0, 1}, cs);
        add_cube_face_colored({0, 0, -1}, cs);

        glBindBuffer(GL_ARRAY_BUFFER, m_scale_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, new_verts.size() * sizeof(float), new_verts.data());
    }
}

void GizmoRenderer::create_scale_gizmo() {
    std::vector<float> vertices;

    auto add_scale_handle = [&](const Vec3& dir, const Vec3& color) {
        // Shaft line
        vertices.insert(vertices.end(), {
            0, 0, 0,  color.x, color.y, color.z,
            dir.x, dir.y, dir.z,  color.x, color.y, color.z
        });

        // Box handle at the end
        Vec3 center = dir;
        float box_size = 0.08f;

        // Create box as triangles (12 triangles for a cube)
        glm::vec3 c(center.x, center.y, center.z);
        float hs = box_size * 0.5f;

        // Find perpendicular vectors
        glm::vec3 glm_dir(dir.x, dir.y, dir.z);
        glm::vec3 glm_perp1 = std::abs(dir.x) > 0.5f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
        glm::vec3 glm_perp2 = glm::normalize(glm::cross(glm_dir, glm_perp1));
        glm_perp1 = glm::normalize(glm::cross(glm_dir, glm_perp2));

        // Box vertices
        std::vector<glm::vec3> box_verts = {
            c + glm_perp1 * hs + glm_perp2 * hs,
            c - glm_perp1 * hs + glm_perp2 * hs,
            c - glm_perp1 * hs - glm_perp2 * hs,
            c + glm_perp1 * hs - glm_perp2 * hs,
            c + glm_dir * hs,
            c - glm_dir * hs
        };

        // Simple box approximation with 8 triangles pointing outward from center
        for (int i = 0; i < 4; ++i) {
            int j = (i + 1) % 4;
            // Triangle toward positive direction
            vertices.insert(vertices.end(), {
                box_verts[4].x, box_verts[4].y, box_verts[4].z,  color.x, color.y, color.z,
                box_verts[i].x, box_verts[i].y, box_verts[i].z,  color.x, color.y, color.z,
                box_verts[j].x, box_verts[j].y, box_verts[j].z,  color.x, color.y, color.z
            });
            // Triangle toward negative direction
            vertices.insert(vertices.end(), {
                box_verts[5].x, box_verts[5].y, box_verts[5].z,  color.x, color.y, color.z,
                box_verts[j].x, box_verts[j].y, box_verts[j].z,  color.x, color.y, color.z,
                box_verts[i].x, box_verts[i].y, box_verts[i].z,  color.x, color.y, color.z
            });
        }
    };

    add_scale_handle({1, 0, 0}, m_color_x);
    add_scale_handle({0, 1, 0}, m_color_y);
    add_scale_handle({0, 0, 1}, m_color_z);

    // Center cube for uniform scale
    float cs = 0.06f;  // Center cube size
    Vec3 c_color{1.0f, 1.0f, 0.2f};  // Yellow for center

    // Add center cube vertices (simplified as 12 triangles)
    auto add_cube_face = [&](const Vec3& normal, float offset) {
        glm::vec3 n(normal.x, normal.y, normal.z);
        glm::vec3 p1 = std::abs(normal.x) > 0.5f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
        glm::vec3 p2 = glm::normalize(glm::cross(n, p1));
        p1 = glm::cross(n, p2);

        glm::vec3 center = n * offset;
        std::vector<glm::vec3> corners = {
            center + (p1 + p2) * cs,
            center + (p1 - p2) * cs,
            center + (-p1 - p2) * cs,
            center + (-p1 + p2) * cs
        };

        vertices.insert(vertices.end(), {
            corners[0].x, corners[0].y, corners[0].z,  c_color.x, c_color.y, c_color.z,
            corners[1].x, corners[1].y, corners[1].z,  c_color.x, c_color.y, c_color.z,
            corners[2].x, corners[2].y, corners[2].z,  c_color.x, c_color.y, c_color.z,
            corners[0].x, corners[0].y, corners[0].z,  c_color.x, c_color.y, c_color.z,
            corners[2].x, corners[2].y, corners[2].z,  c_color.x, c_color.y, c_color.z,
            corners[3].x, corners[3].y, corners[3].z,  c_color.x, c_color.y, c_color.z
        });
    };

    // All 6 faces of center cube
    add_cube_face({1, 0, 0}, cs);
    add_cube_face({-1, 0, 0}, cs);
    add_cube_face({0, 1, 0}, cs);
    add_cube_face({0, -1, 0}, cs);
    add_cube_face({0, 0, 1}, cs);
    add_cube_face({0, 0, -1}, cs);

    m_scale_vertices = vertices;  // Store for color updates
    m_scale_vertex_count = vertices.size() / 6;

    glGenVertexArrays(1, &m_scale_vao);
    glGenBuffers(1, &m_scale_vbo);

    glBindVertexArray(m_scale_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_scale_vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

    glBindVertexArray(0);
}

void GizmoRenderer::render(const Camera& camera, const Vec3& position, GizmoMode mode, float aspect, float viewport_width, float viewport_height) {
    // Store viewport size for screen_to_world calculations
    m_viewport_width = viewport_width;
    m_viewport_height = viewport_height;

    // Update colors based on hover/active state
    update_gizmo_colors();

    m_shader.bind();
    m_shader.set_uniform("uView", camera.view_matrix());
    m_shader.set_uniform("uProj", camera.projection_matrix(aspect));
    m_shader.set_uniform("uPosition", glm::vec3(position.x, position.y, position.z));
    m_shader.set_uniform("uSize", m_size);

    switch (mode) {
    case GizmoMode::Translate:
        glBindVertexArray(m_vao);
        glDrawArrays(GL_LINES, 0, m_vertex_count);
        glDrawArrays(GL_TRIANGLES, m_vertex_count - 12, 12);
        glBindVertexArray(0);
        break;

    case GizmoMode::Rotate:
        if (m_rotate_vao) {
            glBindVertexArray(m_rotate_vao);
            glDrawArrays(GL_LINES, 0, m_rotate_vertex_count);
            glBindVertexArray(0);
        }
        break;

    case GizmoMode::Scale:
        if (m_scale_vao) {
            glBindVertexArray(m_scale_vao);
            // Draw lines first
            glDrawArrays(GL_LINES, 0, m_scale_vertex_count - 36);  // 36 triangles for boxes
            // Draw boxes
            glDrawArrays(GL_TRIANGLES, m_scale_vertex_count - 36, 36);
            glBindVertexArray(0);
        }
        break;
    }
}

bool GizmoRenderer::on_mouse_down(float x, float y, const Camera& camera, const Vec3& position) {
    m_click_start_x = x;
    m_click_start_y = y;
    m_is_dragging = true;
    m_active_axis = m_hover_axis;

    spdlog::debug("Gizmo on_mouse_down: x={:.1f} y={:.1f}, hover_axis={}, clicked={}",
                 x, y, static_cast<int>(m_hover_axis), m_active_axis != GizmoAxis::None);

    // Reset deltas
    m_translation_delta = {};
    m_rotation_delta = {};
    m_scale_delta = {1, 1, 1};

    return m_active_axis != GizmoAxis::None;
}

void GizmoRenderer::on_mouse_move(float x, float y, const Camera& camera, const Vec3& position) {
    if (!m_is_dragging) {
        // Check for hover
        m_hover_axis = GizmoAxis::None;

        // Get ray from mouse position
        Vec3 ray_dir = screen_to_world(x, y, camera);
        Vec3 ray_origin = camera.position();

        spdlog::debug("Gizmo on_mouse_move: x={:.1f} y={:.1f}, ray_origin=({:.2f},{:.2f},{:.2f}), ray_dir=({:.3f},{:.3f},{:.3f})",
                     x, y, ray_origin.x, ray_origin.y, ray_origin.z, ray_dir.x, ray_dir.y, ray_dir.z);

        float closest_t = std::numeric_limits<float>::max();
        float t;

        switch (m_mode) {
        case GizmoMode::Translate:
            // Check X axis
            if (ray_intersect_axis(ray_origin, ray_dir, position, Vec3(1, 0, 0), m_size, t)) {
                if (t < closest_t) { closest_t = t; m_hover_axis = GizmoAxis::X; }
            }
            // Check Y axis
            if (ray_intersect_axis(ray_origin, ray_dir, position, Vec3(0, 1, 0), m_size, t)) {
                if (t < closest_t) { closest_t = t; m_hover_axis = GizmoAxis::Y; }
            }
            // Check Z axis
            if (ray_intersect_axis(ray_origin, ray_dir, position, Vec3(0, 0, 1), m_size, t)) {
                if (t < closest_t) { closest_t = t; m_hover_axis = GizmoAxis::Z; }
            }
            spdlog::debug("Gizmo hover: mode=translate, hover_axis={}, closest_t={:.3f}", static_cast<int>(m_hover_axis), closest_t);
            break;

        case GizmoMode::Rotate: {
            constexpr float circle_radius = 0.8f;
            // Check X rotation circle (YZ plane)
            if (ray_intersect_circle(ray_origin, ray_dir, position, Vec3(1, 0, 0), circle_radius * m_size, t)) {
                if (t < closest_t) { closest_t = t; m_hover_axis = GizmoAxis::X; }
            }
            // Check Y rotation circle (XZ plane)
            if (ray_intersect_circle(ray_origin, ray_dir, position, Vec3(0, 1, 0), circle_radius * m_size, t)) {
                if (t < closest_t) { closest_t = t; m_hover_axis = GizmoAxis::Y; }
            }
            // Check Z rotation circle (XY plane)
            if (ray_intersect_circle(ray_origin, ray_dir, position, Vec3(0, 0, 1), circle_radius * m_size, t)) {
                if (t < closest_t) { closest_t = t; m_hover_axis = GizmoAxis::Z; }
            }
            break;
        }

        case GizmoMode::Scale:
            // Check X axis with box handle
            if (ray_intersect_axis(ray_origin, ray_dir, position, Vec3(1, 0, 0), m_size, t)) {
                if (t < closest_t) { closest_t = t; m_hover_axis = GizmoAxis::X; }
            }
            // Check box handle at end of X axis
            Vec3 x_box_pos = {position.x + m_size, position.y, position.z};
            if (ray_intersect_box(ray_origin, ray_dir, x_box_pos, 0.08f * m_size, t)) {
                if (t < closest_t) { closest_t = t; m_hover_axis = GizmoAxis::X; }
            }

            // Check Y axis with box handle
            if (ray_intersect_axis(ray_origin, ray_dir, position, Vec3(0, 1, 0), m_size, t)) {
                if (t < closest_t) { closest_t = t; m_hover_axis = GizmoAxis::Y; }
            }
            Vec3 y_box_pos = {position.x, position.y + m_size, position.z};
            if (ray_intersect_box(ray_origin, ray_dir, y_box_pos, 0.08f * m_size, t)) {
                if (t < closest_t) { closest_t = t; m_hover_axis = GizmoAxis::Y; }
            }

            // Check Z axis with box handle
            if (ray_intersect_axis(ray_origin, ray_dir, position, Vec3(0, 0, 1), m_size, t)) {
                if (t < closest_t) { closest_t = t; m_hover_axis = GizmoAxis::Z; }
            }
            Vec3 z_box_pos = {position.x, position.y, position.z + m_size};
            if (ray_intersect_box(ray_origin, ray_dir, z_box_pos, 0.08f * m_size, t)) {
                if (t < closest_t) { closest_t = t; m_hover_axis = GizmoAxis::Z; }
            }

            // Check center cube for uniform scale
            if (ray_intersect_box(ray_origin, ray_dir, position, 0.12f * m_size, t)) {
                if (t < closest_t) { closest_t = t; m_hover_axis = GizmoAxis::XYZ; }
            }
            break;
        }
        return;
    }

    float dx = x - m_click_start_x;
    float dy = y - m_click_start_y;

    float sensitivity = 0.05f; // Increased for better responsiveness
    float total_delta = std::sqrt(dx * dx + dy * dy);

    // Determine sign based on dominant direction
    float sign = (std::abs(dx) > std::abs(dy)) ? (dx > 0 ? 1.0f : -1.0f) : (dy > 0 ? 1.0f : -1.0f);

    spdlog::debug("Gizmo DRAG: dx={:.2f}, dy={:.2f}, mode={}, active_axis={}, sign={:.1f}, total_delta={:.3f}",
                 dx, dy, static_cast<int>(m_mode), static_cast<int>(m_active_axis), sign, total_delta);

    switch (m_mode) {
    case GizmoMode::Translate:
        switch (m_active_axis) {
        case GizmoAxis::X:
            // For X axis, use horizontal movement primarily
            m_translation_delta.x = dx * sensitivity;
            m_translation_delta.y = 0;
            m_translation_delta.z = 0;
            break;
        case GizmoAxis::Y:
            // For Y axis, use vertical movement
            m_translation_delta.x = 0;
            m_translation_delta.y = -dy * sensitivity;  // Negative because screen Y is down
            m_translation_delta.z = 0;
            break;
        case GizmoAxis::Z:
            // For Z axis, use combination (diagonal)
            m_translation_delta.x = 0;
            m_translation_delta.y = 0;
            m_translation_delta.z = (dx + dy) * sensitivity;
            break;
        default:
            break;
        }
        break;

    case GizmoMode::Rotate:
        switch (m_active_axis) {
        case GizmoAxis::X:
            m_rotation_delta.x = sign * total_delta * sensitivity * 2.0f;
            m_rotation_delta.y = 0;
            m_rotation_delta.z = 0;
            spdlog::debug("Gizmo ROTATION X: delta={:.4f}", m_rotation_delta.x);
            break;
        case GizmoAxis::Y:
            m_rotation_delta.x = 0;
            m_rotation_delta.y = sign * total_delta * sensitivity * 2.0f;
            m_rotation_delta.z = 0;
            spdlog::debug("Gizmo ROTATION Y: delta={:.4f}", m_rotation_delta.y);
            break;
        case GizmoAxis::Z:
            m_rotation_delta.x = 0;
            m_rotation_delta.y = 0;
            m_rotation_delta.z = sign * total_delta * sensitivity * 2.0f;
            spdlog::debug("Gizmo ROTATION Z: delta={:.4f}", m_rotation_delta.z);
            break;
        default:
            spdlog::debug("Gizmo ROTATION: No active axis");
            break;
        }
        break;

    case GizmoMode::Scale: {
        float uniform_scale = 1.0f + sign * total_delta * sensitivity;
        switch (m_active_axis) {
        case GizmoAxis::X:
            m_scale_delta.x = uniform_scale;
            m_scale_delta.y = 1.0f;
            m_scale_delta.z = 1.0f;
            break;
        case GizmoAxis::Y:
            m_scale_delta.x = 1.0f;
            m_scale_delta.y = uniform_scale;
            m_scale_delta.z = 1.0f;
            break;
        case GizmoAxis::Z:
            m_scale_delta.x = 1.0f;
            m_scale_delta.y = 1.0f;
            m_scale_delta.z = uniform_scale;
            break;
        case GizmoAxis::XYZ:
            m_scale_delta = {uniform_scale, uniform_scale, uniform_scale};
            break;
        default:
            break;
        }
        break;
    }
    }
}

void GizmoRenderer::on_mouse_up() {
    m_is_dragging = false;
    m_active_axis = GizmoAxis::None;
}

bool GizmoRenderer::ray_intersect_axis(const Vec3& ray_origin, const Vec3& ray_dir,
                                       const Vec3& axis_pos, const Vec3& axis_dir,
                                       float axis_length, float& t) const {
    // Ray-Line segment intersection
    // Find the minimum distance between ray and axis line segment
    // Using the closest points approach for two line segments

    glm::vec3 ro(ray_origin.x, ray_origin.y, ray_origin.z);
    glm::vec3 rd(ray_dir.x, ray_dir.y, ray_dir.z);
    glm::vec3 ap(axis_pos.x, axis_pos.y, axis_pos.z);
    glm::vec3 ad(axis_dir.x, axis_dir.y, axis_dir.z);

    spdlog::debug("ray_intersect_axis START: ro=({:.2f},{:.2f},{:.2f}), rd=({:.3f},{:.3f},{:.3f}), ap=({:.2f},{:.2f},{:.2f}), ad=({:.2f},{:.2f},{:.2f}), len={:.2f}",
                 ro.x, ro.y, ro.z, rd.x, rd.y, rd.z, ap.x, ap.y, ap.z, ad.x, ad.y, ad.z, axis_length);

    // Axis segment endpoints
    glm::vec3 axis_end = ap + ad * axis_length;

    // Vector from ray origin to axis start
    glm::vec3 w = ap - ro;

    float a = glm::dot(rd, rd);         // Should be 1.0 if normalized
    float b = glm::dot(rd, ad);         // Ray . Axis direction
    float c = glm::dot(ad, ad);         // Should be 1.0 if normalized
    float d = glm::dot(rd, w);
    float e = glm::dot(ad, w);
    float denom = a * c - b * b;

    float t_ray, t_axis;

    // If rays are almost parallel
    if (std::abs(denom) < 1e-6f) {
        // Use midpoint of axis segment as reference
        t_ray = d / a;  // Project onto ray
        t_axis = 0.5f * axis_length;  // Midpoint
    } else {
        t_ray = (b * e - c * d) / denom;
        t_axis = (a * e - b * d) / denom;
    }

    spdlog::trace("ray_intersect_axis: t_ray={:.3f}, t_axis={:.3f}", t_ray, t_axis);

    // Clamp t_axis to segment [0, axis_length]
    float original_t_axis = t_axis;
    if (t_axis < 0) t_axis = 0;
    if (t_axis > axis_length) t_axis = axis_length;

    // After clamping t_axis, we need to find the point on ray closest to the clamped axis point
    // The clamped point on the axis
    glm::vec3 clamped_axis_point = ap + ad * t_axis;
    // Project this point onto the ray: t_ray = dot(clamped_axis_point - ro, rd)
    t_ray = glm::dot(clamped_axis_point - ro, rd);

    spdlog::trace("ray_intersect_axis: clamped t_ray={:.3f}, t_axis={:.3f}", t_ray, t_axis);

    // Check if ray intersection point is in front
    if (t_ray < 0) {
        spdlog::trace("ray_intersect_axis: t_ray NEGATIVE after clamp");
        return false;
    }

    // Calculate closest points
    glm::vec3 closest_on_ray = ro + rd * t_ray;
    glm::vec3 closest_on_axis = ap + ad * t_axis;

    // Distance between closest points
    float distance = glm::length(closest_on_ray - closest_on_axis);

    // Threshold for "close enough" to click - scale with distance for perspective
    // Much more generous threshold for easier clicking
    float distance_to_gizmo = glm::length(ro - ap);
    float threshold = 0.4f * m_size * (distance_to_gizmo / 2.0f);

    spdlog::trace("ray_intersect_axis: distance={:.3f}, threshold={:.3f}, HIT={}", distance, threshold, distance < threshold);

    if (distance < threshold) {
        t = t_ray;
        return true;
    }

    return false;
}

bool GizmoRenderer::ray_intersect_circle(const Vec3& ray_origin, const Vec3& ray_dir,
                                        const Vec3& circle_center, const Vec3& circle_normal,
                                        float radius, float& t) const {
    // Ray-Plane intersection first
    glm::vec3 ro(ray_origin.x, ray_origin.y, ray_origin.z);
    glm::vec3 rd(ray_dir.x, ray_dir.y, ray_dir.z);
    glm::vec3 cc(circle_center.x, circle_center.y, circle_center.z);
    glm::vec3 cn(circle_normal.x, circle_normal.y, circle_normal.z);

    cn = glm::normalize(cn);

    // Plane: dot(p - cc, cn) = 0
    // Ray: p = ro + t * rd
    // dot(ro + t*rd - cc, cn) = 0
    // dot(ro - cc, cn) + t * dot(rd, cn) = 0
    // t = -dot(ro - cc, cn) / dot(rd, cn)

    float denom = glm::dot(rd, cn);
    if (std::abs(denom) < 1e-6f) {
        // Ray is parallel to plane
        return false;
    }

    float t_plane = -glm::dot(ro - cc, cn) / denom;

    if (t_plane < 0) {
        // Plane is behind ray
        return false;
    }

    // Intersection point on plane
    glm::vec3 intersection = ro + rd * t_plane;

    // Check if intersection is within circle radius (with threshold)
    float distance = glm::length(intersection - cc);
    float threshold = 0.2f * m_size;  // Fixed threshold for circle edge clicking

    spdlog::trace("ray_intersect_circle: distance={:.3f}, radius={:.3f}, threshold={:.3f}, HIT={}",
                 distance, radius, threshold, std::abs(distance - radius) < threshold);

    if (std::abs(distance - radius) < threshold) {
        t = t_plane;
        return true;
    }

    return false;
}

bool GizmoRenderer::ray_intersect_box(const Vec3& ray_origin, const Vec3& ray_dir,
                                     const Vec3& box_center, float box_size,
                                     float& t) const {
    // Ray-AABB intersection using slab method
    glm::vec3 ro(ray_origin.x, ray_origin.y, ray_origin.z);
    glm::vec3 rd(ray_dir.x, ray_dir.y, ray_dir.z);
    glm::vec3 bc(box_center.x, box_center.y, box_center.z);

    float hs = box_size * 0.5f;  // Half size

    glm::vec3 box_min = bc - glm::vec3(hs, hs, hs);
    glm::vec3 box_max = bc + glm::vec3(hs, hs, hs);

    float t_min = 0.0f;
    float t_max = std::numeric_limits<float>::max();

    for (int i = 0; i < 3; ++i) {
        if (std::abs(rd[i]) < 1e-6f) {
            // Ray is parallel to this slab
            if (ro[i] < box_min[i] || ro[i] > box_max[i]) {
                return false;
            }
        } else {
            float t1 = (box_min[i] - ro[i]) / rd[i];
            float t2 = (box_max[i] - ro[i]) / rd[i];

            if (t1 > t2) std::swap(t1, t2);

            t_min = std::max(t_min, t1);
            t_max = std::min(t_max, t2);

            if (t_min > t_max) {
                return false;
            }
        }
    }

    if (t_min < 0) {
        if (t_max < 0) {
            return false;
        }
        t = t_max;
    } else {
        t = t_min;
    }

    return true;
}

Vec3 GizmoRenderer::screen_to_world(float x, float y, const Camera& camera) const {
    // Convert screen coordinates to world space ray direction
    // Using the camera's actual view and projection matrices

    // Get camera properties
    Vec3 cam_pos = camera.position();
    Vec3 cam_target = camera.target();

    glm::vec3 pos(cam_pos.x, cam_pos.y, cam_pos.z);
    glm::vec3 target(cam_target.x, cam_target.y, cam_target.z);

    // Calculate camera basis vectors
    glm::vec3 forward = glm::normalize(target - pos);
    glm::vec3 world_up = {0, 1, 0};
    glm::vec3 right = glm::normalize(glm::cross(forward, world_up));
    glm::vec3 up = glm::cross(right, forward);

    // Use stored viewport size
    float width = m_viewport_width;
    float height = m_viewport_height;

    // Normalize to [-1, 1]
    float ndc_x = (2.0f * x) / width - 1.0f;
    float ndc_y = 1.0f - (2.0f * y) / height;  // Flip Y (screen Y is down, world Y is up)

    // Use actual camera FOV
    float fov = glm::radians(camera.fov());
    float tan_half_fov = std::tan(fov * 0.5f);
    float aspect = width / height;

    // Calculate ray direction in camera space (Z is negative into the screen in OpenGL)
    glm::vec3 ray_dir_camera{
        ndc_x * aspect * tan_half_fov,
        ndc_y * tan_half_fov,
        -1.0f  // Forward into the screen (negative Z in OpenGL camera space)
    };
    ray_dir_camera = glm::normalize(ray_dir_camera);

    // Transform from camera space to world space
    // In camera space: +X is right, +Y is up, -Z is forward (into screen)
    // We need to flip Z because camera looks down -Z
    glm::vec3 ray_dir_world = right * ray_dir_camera.x + up * ray_dir_camera.y + forward * (-ray_dir_camera.z);
    ray_dir_world = glm::normalize(ray_dir_world);

    spdlog::trace("screen_to_world: screen=({:.1f},{:.1f}), ndc=({:.3f},{:.3f}), ray_dir_world=({:.3f},{:.3f},{:.3f})",
                 x, y, ndc_x, ndc_y, ray_dir_world.x, ray_dir_world.y, ray_dir_world.z);

    return {ray_dir_world.x, ray_dir_world.y, ray_dir_world.z};
}

} // namespace mechatron
