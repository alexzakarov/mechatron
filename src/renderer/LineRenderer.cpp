#include "LineRenderer.hpp"
#include <GL/glew.h>

namespace mechatron {

LineRenderer::LineRenderer() {
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
}

LineRenderer::~LineRenderer() {
    if (m_vao) glDeleteVertexArrays(1, &m_vao);
    if (m_vbo) glDeleteBuffers(1, &m_vbo);
}

void LineRenderer::begin() {
    m_vertices.clear();
}

void LineRenderer::draw_line(const Vec3& a, const Vec3& b, uint32_t color) {
    m_vertices.push_back({a, color});
    m_vertices.push_back({b, color});
}

void LineRenderer::draw_box(const Vec3& min, const Vec3& max, uint32_t color) {
    // 12 edges of a box
    Vec3 corners[8] = {
        {min.x, min.y, min.z}, {max.x, min.y, min.z},
        {max.x, max.y, min.z}, {min.x, max.y, min.z},
        {min.x, min.y, max.z}, {max.x, min.y, max.z},
        {max.x, max.y, max.z}, {min.x, max.y, max.z}
    };
    int edges[24] = {
        0,1, 1,2, 2,3, 3,0,
        4,5, 5,6, 6,7, 7,4,
        0,4, 1,5, 2,6, 3,7
    };
    for (int i = 0; i < 24; i += 2) {
        draw_line(corners[edges[i]], corners[edges[i+1]], color);
    }
}

void LineRenderer::end(uint32_t shader_program, const float* view_proj) {
    if (m_vertices.empty()) return;

    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, m_vertices.size() * sizeof(LineVertex), m_vertices.data(), GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(LineVertex), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(LineVertex), (void*)offsetof(LineVertex, color));

    glUseProgram(shader_program);
    if (view_proj) {
        int loc = glGetUniformLocation(shader_program, "u_view_proj");
        if (loc >= 0) glUniformMatrix4fv(loc, 1, GL_FALSE, view_proj);
    }

    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(m_vertices.size()));
    glBindVertexArray(0);
}

} // namespace mechatron
