#pragma once

#include "Types.hpp"
#include <vector>
#include <cstdint>

namespace mechatron {

struct Vertex {
    Vec3 position;
    Vec3 normal;
    // Vec2 texcoord; // TODO: texture support
};

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    uint32_t vao = 0;
    uint32_t vbo = 0;
    uint32_t ebo = 0;

    Mesh() = default;
    ~Mesh();
    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&& other) noexcept;
    Mesh& operator=(Mesh&& other) noexcept;

    void upload();
    void draw() const;
    void draw_lines() const;
    void draw_points(float point_size = 1.0f) const;
    void cleanup();

    static Mesh create_box(float w, float h, float d);
    static Mesh create_sphere(float radius, int segments = 16, int rings = 16);
    static Mesh create_cylinder(float radius, float height, int segments = 32);
    static Mesh create_grid(float size, int divisions);
};

} // namespace mechatron
