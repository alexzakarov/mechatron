#include "GridRenderer.hpp"
#include "Camera.hpp"
#include "core/Types.hpp"
#include <spdlog/spdlog.h>
#include <vector>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace mechatron {

// Vertex shader
static const char* grid_vertex_shader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

uniform mat4 uView;
uniform mat4 uProj;

out vec3 vColor;

void main() {
    gl_Position = uProj * uView * vec4(aPos, 1.0);
    vColor = aColor;
}
)";

// Fragment shader
static const char* grid_fragment_shader = R"(
#version 330 core
in vec3 vColor;
out vec4 FragColor;

void main() {
    FragColor = vec4(vColor, 1.0);
}
)";

GridRenderer::GridRenderer() = default;

GridRenderer::~GridRenderer() {
    shutdown();
}

bool GridRenderer::init() {
    // Create shader
    if (!m_shader.load_from_source(grid_vertex_shader, grid_fragment_shader)) {
        spdlog::error("GridRenderer: Failed to compile shader");
        return false;
    }

    // Create initial grid mesh
    create_grid_mesh(20.0f, 20);
    create_axes_mesh();

    spdlog::info("GridRenderer initialized");
    return true;
}

void GridRenderer::shutdown() {
    if (m_grid_vao) glDeleteVertexArrays(1, &m_grid_vao);
    if (m_grid_vbo) glDeleteBuffers(1, &m_grid_vbo);
    if (m_axes_vao) glDeleteVertexArrays(1, &m_axes_vao);
    if (m_axes_vbo) glDeleteBuffers(1, &m_axes_vbo);

    m_grid_vao = m_grid_vbo = m_axes_vao = m_axes_vbo = 0;
}

void GridRenderer::create_grid_mesh(float size, int divisions) {
    std::vector<float> vertices;

    float half_size = size * 0.5f;
    float step = size / divisions;

    // Grid lines (X and Z directions)
    for (int i = 0; i <= divisions; ++i) {
        float pos = -half_size + i * step;

        // Line along X
        vertices.push_back(-half_size); vertices.push_back(0.0f); vertices.push_back(pos);
        vertices.push_back(m_grid_color.x); vertices.push_back(m_grid_color.y); vertices.push_back(m_grid_color.z);

        vertices.push_back(half_size); vertices.push_back(0.0f); vertices.push_back(pos);
        vertices.push_back(m_grid_color.x); vertices.push_back(m_grid_color.y); vertices.push_back(m_grid_color.z);

        // Line along Z
        vertices.push_back(pos); vertices.push_back(0.0f); vertices.push_back(-half_size);
        vertices.push_back(m_grid_color.x); vertices.push_back(m_grid_color.y); vertices.push_back(m_grid_color.z);

        vertices.push_back(pos); vertices.push_back(0.0f); vertices.push_back(half_size);
        vertices.push_back(m_grid_color.x); vertices.push_back(m_grid_color.y); vertices.push_back(m_grid_color.z);
    }

    m_grid_vertex_count = vertices.size() / 6;

    // Create VAO/VBO
    glGenVertexArrays(1, &m_grid_vao);
    glGenBuffers(1, &m_grid_vbo);

    glBindVertexArray(m_grid_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_grid_vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    // Position attribute
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);

    // Color attribute
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

    glBindVertexArray(0);
}

void GridRenderer::create_axes_mesh() {
    std::vector<float> vertices = {
        // X Axis (Red)
        0.0f, 0.0f, 0.0f,  1.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,  1.0f, 0.0f, 0.0f,

        // Y Axis (Green)
        0.0f, 0.0f, 0.0f,  0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,  0.0f, 1.0f, 0.0f,

        // Z Axis (Blue)
        0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f,  0.0f, 0.0f, 1.0f
    };

    m_axes_vertex_count = vertices.size() / 6;

    glGenVertexArrays(1, &m_axes_vao);
    glGenBuffers(1, &m_axes_vbo);

    glBindVertexArray(m_axes_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_axes_vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

    glBindVertexArray(0);
}

void GridRenderer::render(const glm::mat4& view, const glm::mat4& proj, float size, int divisions) {
    m_shader.bind();
    m_shader.set_uniform("uView", view);
    m_shader.set_uniform("uProj", proj);

    // Render grid
    if (m_grid_visible) {
        glBindVertexArray(m_grid_vao);
        glDrawArrays(GL_LINES, 0, m_grid_vertex_count);
    }

    // Render axes
    if (m_axes_visible) {
        render_axes(view, proj, size * 0.1f);
    }

    glBindVertexArray(0);
}

void GridRenderer::render_axes(const glm::mat4& view, const glm::mat4& proj, float size) {
    // Use scale to adjust axis size
    // For now, use fixed size axes
    glBindVertexArray(m_axes_vao);
    glLineWidth(2.0f);
    glDrawArrays(GL_LINES, 0, m_axes_vertex_count);
    glLineWidth(1.0f);
}

} // namespace mechatron
