#include "Mesh.hpp"
#include <GL/glew.h>
#include <cmath>

namespace mechatron {

void Mesh::upload() {
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(uint32_t), indices.data(), GL_STATIC_DRAW);

    // Position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);

    // Normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

    glBindVertexArray(0);
}

void Mesh::draw() const {
    if (!vao) return;
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void Mesh::cleanup() {
    if (vao) glDeleteVertexArrays(1, &vao);
    if (vbo) glDeleteBuffers(1, &vbo);
    if (ebo) glDeleteBuffers(1, &ebo);
    vao = vbo = ebo = 0;
}

Mesh Mesh::create_box(float w, float h, float d) {
    Mesh mesh;
    float hw = w / 2, hh = h / 2, hd = d / 2;

    // 6 faces, 4 vertices each
    auto add_face = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 d, Vec3 n) {
        uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back({a, n});
        mesh.vertices.push_back({b, n});
        mesh.vertices.push_back({c, n});
        mesh.vertices.push_back({d, n});
        mesh.indices.insert(mesh.indices.end(), {base, base+1, base+2, base, base+2, base+3});
    };

    // Front
    add_face({-hw,-hh, hd}, { hw,-hh, hd}, { hw, hh, hd}, {-hw, hh, hd}, {0,0,1});
    // Back
    add_face({ hw,-hh,-hd}, {-hw,-hh,-hd}, {-hw, hh,-hd}, { hw, hh,-hd}, {0,0,-1});
    // Top
    add_face({-hw, hh, hd}, { hw, hh, hd}, { hw, hh,-hd}, {-hw, hh,-hd}, {0,1,0});
    // Bottom
    add_face({-hw,-hh,-hd}, { hw,-hh,-hd}, { hw,-hh, hd}, {-hw,-hh, hd}, {0,-1,0});
    // Right
    add_face({ hw,-hh, hd}, { hw,-hh,-hd}, { hw, hh,-hd}, { hw, hh, hd}, {1,0,0});
    // Left
    add_face({-hw,-hh,-hd}, {-hw,-hh, hd}, {-hw, hh, hd}, {-hw, hh,-hd}, {-1,0,0});

    mesh.upload();
    return mesh;
}

Mesh Mesh::create_sphere(float radius, int segments, int rings) {
    Mesh mesh;

    for (int ring = 0; ring <= rings; ++ring) {
        float phi = 3.14159265f * ring / rings;
        for (int seg = 0; seg <= segments; ++seg) {
            float theta = 2.0f * 3.14159265f * seg / segments;

            float x = radius * std::sin(phi) * std::cos(theta);
            float y = radius * std::cos(phi);
            float z = radius * std::sin(phi) * std::sin(theta);

            Vec3 normal = Vec3{x, y, z}.normalized();
            mesh.vertices.push_back({{x, y, z}, normal});
        }
    }

    for (int ring = 0; ring < rings; ++ring) {
        for (int seg = 0; seg < segments; ++seg) {
            uint32_t a = ring * (segments + 1) + seg;
            uint32_t b = a + segments + 1;
            mesh.indices.insert(mesh.indices.end(), {a, b, a+1, b, b+1, a+1});
        }
    }

    mesh.upload();
    return mesh;
}

Mesh Mesh::create_cylinder(float radius, float height, int segments) {
    Mesh mesh;
    float hh = height / 2;

    auto add_circle = [&](float y, float ny, bool ccw) {
        uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back({{0, y, 0}, {0, ny, 0}}); // center
        for (int i = 0; i <= segments; ++i) {
            float angle = 2.0f * 3.14159265f * i / segments;
            float x = radius * std::cos(angle);
            float z = radius * std::sin(angle);
            mesh.vertices.push_back({{x, y, z}, {0, ny, 0}});
        }
        for (int i = 1; i <= segments; ++i) {
            if (ccw)
                mesh.indices.insert(mesh.indices.end(), {base, base + i + 1, base + i});
            else
                mesh.indices.insert(mesh.indices.end(), {base, base + i, base + i + 1});
        }
    };

    add_circle(hh, 1, false);   // top cap
    add_circle(-hh, -1, true);  // bottom cap

    // Side
    uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * 3.14159265f * i / segments;
        float x = radius * std::cos(angle);
        float z = radius * std::sin(angle);
        Vec3 n = Vec3{x, 0, z}.normalized();
        mesh.vertices.push_back({{x, hh, z}, n});
        mesh.vertices.push_back({{x, -hh, z}, n});
    }
    for (int i = 0; i < segments; ++i) {
        uint32_t a = base + i * 2;
        uint32_t b = a + 2;
        mesh.indices.insert(mesh.indices.end(), {a, b, a+1, a+1, b, b+1});
    }

    mesh.upload();
    return mesh;
}

Mesh Mesh::create_grid(float size, int divisions) {
    Mesh mesh;
    float half = size / 2;
    float step = size / divisions;

    for (int i = 0; i <= divisions; ++i) {
        float pos = -half + i * step;
        mesh.vertices.push_back({{pos, 0, -half}, {0, 1, 0}});
        mesh.vertices.push_back({{pos, 0,  half}, {0, 1, 0}});
        mesh.vertices.push_back({{-half, 0, pos}, {0, 1, 0}});
        mesh.vertices.push_back({{ half, 0, pos}, {0, 1, 0}});
    }

    mesh.upload();
    return mesh;
}

} // namespace mechatron
