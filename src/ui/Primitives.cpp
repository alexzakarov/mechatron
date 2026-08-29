// ============================================================================
// Primitives.cpp — implementations for Blender-style primitive meshes.
// See Primitives.hpp for design notes.
// ============================================================================

#include "Primitives.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace mechatron {

MeshData make_torus_mesh(int major_segments, int minor_segments,
                         float major_radius, float minor_radius) {
    MeshData mesh;
    major_segments = std::clamp(major_segments, 3, 256);
    minor_segments = std::clamp(minor_segments, 3, 256);
    major_radius = std::max(1e-3f, major_radius);
    minor_radius = std::max(1e-3f, minor_radius);
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTau = 2.0f * kPi;
    mesh.vertices.reserve(static_cast<size_t>(major_segments) * minor_segments);
    for (int i = 0; i < major_segments; ++i) {
        const float major_angle =
            static_cast<float>(i) / static_cast<float>(major_segments) * kTau;
        const float cx = std::cos(major_angle) * major_radius;
        const float cz = std::sin(major_angle) * major_radius;
        const float cos_ma = std::cos(major_angle);
        const float sin_ma = std::sin(major_angle);
        for (int j = 0; j < minor_segments; ++j) {
            const float minor_angle =
                static_cast<float>(j) / static_cast<float>(minor_segments) * kTau;
            const float ring_x = std::cos(minor_angle) * minor_radius;
            const float ring_y = std::sin(minor_angle) * minor_radius;
            const Vec3 vertex{
                cx + ring_x * cos_ma,
                ring_y,
                cz + ring_x * sin_ma,
            };
            mesh.vertices.push_back(vertex);
        }
    }
    const auto idx = [&](int i, int j) -> uint32_t {
        i = ((i % major_segments) + major_segments) % major_segments;
        j = ((j % minor_segments) + minor_segments) % minor_segments;
        return static_cast<uint32_t>(i * minor_segments + j);
    };
    mesh.triangles.reserve(static_cast<size_t>(major_segments) * minor_segments * 2);
    for (int i = 0; i < major_segments; ++i) {
        for (int j = 0; j < minor_segments; ++j) {
            const uint32_t a = idx(i, j);
            const uint32_t b = idx(i + 1, j);
            const uint32_t c = idx(i + 1, j + 1);
            const uint32_t d = idx(i, j + 1);
            mesh.triangles.push_back({a, b, c});
            mesh.triangles.push_back({a, c, d});
        }
    }
    mesh.calculate_normals();
    return mesh;
}

MeshData make_monkey_mesh() {
    MeshData mesh;
    mesh.vertices = {
        {0.00f,  0.00f,  1.00f}, {0.70f,  0.00f,  0.70f}, {-0.70f,  0.00f,  0.70f},
        {0.90f,  0.00f,  0.00f}, {-0.90f,  0.00f,  0.00f}, {0.70f,  0.00f, -0.70f},
        {-0.70f,  0.00f, -0.70f}, {0.00f,  0.00f, -1.00f},
        {-0.50f,  0.50f,  0.75f}, {0.50f,  0.50f,  0.75f}, {0.00f,  0.65f,  0.80f},
        {-0.35f,  0.25f,  0.85f}, {0.35f,  0.25f,  0.85f},
        {-0.35f,  0.25f,  0.95f}, {0.35f,  0.25f,  0.95f},
        {0.00f,  0.10f,  1.00f}, {0.00f, -0.20f,  1.05f}, {0.00f, -0.30f,  1.00f},
        {-0.12f, -0.30f,  0.95f}, {0.12f, -0.30f,  0.95f},
        {-0.40f, -0.55f,  0.80f}, {0.40f, -0.55f,  0.80f}, {0.00f, -0.65f,  0.85f},
        {-0.55f, -0.80f,  0.40f}, {0.55f, -0.80f,  0.40f}, {0.00f, -0.95f,  0.40f},
        { 1.00f,  0.10f,  0.10f}, {-1.00f,  0.10f,  0.10f},
        { 1.05f,  0.30f,  0.00f}, {-1.05f,  0.30f,  0.00f},
        { 1.00f, -0.10f,  0.10f}, {-1.00f, -0.10f,  0.10f},
        {0.00f,  0.95f,  0.30f}, {0.00f,  0.95f, -0.30f},
        {0.00f,  0.30f, -0.95f}, {0.00f, -0.30f, -0.95f},
    };
    mesh.triangles = {
        {14, 0, 11}, {14, 11, 8}, {14, 8, 10}, {14, 10, 9}, {14, 9, 12},
        {14, 12, 0},
        {8, 11, 0}, {9, 0, 12},
        {8, 10, 29}, {10, 9, 29}, {29, 31, 8}, {29, 31, 9},
        {15, 16, 17}, {15, 18, 16},
        {16, 21, 19}, {16, 19, 20}, {16, 20, 21},
        {19, 21, 22}, {21, 20, 22},
        {22, 19, 23}, {22, 23, 24}, {24, 23, 25}, {25, 23, 26},
        {22, 24, 25},
        {0, 1, 3}, {0, 3, 23}, {0, 23, 19},
        {0, 2, 4}, {0, 4, 26}, {0, 26, 20},
        {13, 11, 12},
        {27, 1, 3}, {28, 4, 2},
        {29, 31, 6}, {29, 6, 5}, {29, 5, 1}, {29, 1, 27},
        {31, 7, 6}, {31, 32, 7},
        {32, 23, 7}, {32, 26, 23},
        {7, 23, 26},
    };
    mesh.calculate_normals();
    return mesh;
}

MeshData make_bezier_curve_mesh(int samples, int tube_segments) {
    MeshData mesh;
    samples = std::clamp(samples, 4, 512);
    tube_segments = std::clamp(tube_segments, 3, 32);
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTau = 2.0f * kPi;
    const Vec3 p0{-1.0f, 0.0f, -0.8f};
    const Vec3 p1{-0.4f, 0.0f,  0.9f};
    const Vec3 p2{ 0.4f, 0.0f, -0.9f};
    const Vec3 p3{ 1.0f, 0.0f,  0.8f};
    const float tube_radius = 0.05f;
    auto bezier_eval = [&](float t) {
        const float u = 1.0f - t;
        return p0 * (u * u * u) + p1 * (3.0f * u * u * t) +
               p2 * (3.0f * u * t * t) + p3 * (t * t * t);
    };
    std::vector<Vec3> curve(samples);
    std::vector<Vec3> tangent(samples);
    for (int i = 0; i < samples; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(samples - 1);
        curve[i] = bezier_eval(t);
    }
    for (int i = 0; i < samples; ++i) {
        const Vec3 prev = curve[std::max(0, i - 1)];
        const Vec3 next = curve[std::min(samples - 1, i + 1)];
        Vec3 t = next - prev;
        const float len = std::max(1e-8f, t.length());
        t = t * (1.0f / len);
        tangent[i] = t;
    }
    mesh.vertices.reserve(static_cast<size_t>(samples) * tube_segments);
    for (int i = 0; i < samples; ++i) {
        const Vec3 t = tangent[i];
        Vec3 ref = (std::fabs(t.y) < 0.9f) ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
        const float dot = ref.x * t.x + ref.y * t.y + ref.z * t.z;
        ref.x -= t.x * dot; ref.y -= t.y * dot; ref.z -= t.z * dot;
        const float rl = std::max(1e-8f, std::sqrt(ref.x * ref.x + ref.y * ref.y + ref.z * ref.z));
        ref = ref * (1.0f / rl);
        const Vec3 bit{
            t.y * ref.z - t.z * ref.y,
            t.z * ref.x - t.x * ref.z,
            t.x * ref.y - t.y * ref.x
        };
        for (int j = 0; j < tube_segments; ++j) {
            const float a = static_cast<float>(j) / static_cast<float>(tube_segments) * kTau;
            const float ca = std::cos(a), sa = std::sin(a);
            const Vec3 offset{
                ref.x * ca + bit.x * sa,
                ref.y * ca + bit.y * sa,
                ref.z * ca + bit.z * sa,
            };
            mesh.vertices.push_back(curve[i] + offset * tube_radius);
        }
    }
    const auto idx = [&](int ring, int side) -> uint32_t {
        ring = std::clamp(ring, 0, samples - 1);
        side = ((side % tube_segments) + tube_segments) % tube_segments;
        return static_cast<uint32_t>(ring * tube_segments + side);
    };
    mesh.triangles.reserve(static_cast<size_t>(samples - 1) * tube_segments * 2);
    for (int i = 0; i < samples - 1; ++i) {
        for (int j = 0; j < tube_segments; ++j) {
            const uint32_t a = idx(i, j);
            const uint32_t b = idx(i + 1, j);
            const uint32_t c = idx(i + 1, j + 1);
            const uint32_t d = idx(i, j + 1);
            mesh.triangles.push_back({a, b, c});
            mesh.triangles.push_back({a, c, d});
        }
    }
    mesh.calculate_normals();
    return mesh;
}

MeshData make_nurbs_curve_mesh(int samples, int tube_segments) {
    MeshData mesh;
    samples = std::clamp(samples, 4, 512);
    tube_segments = std::clamp(tube_segments, 3, 32);
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTau = 2.0f * kPi;
    const Vec3 cp[] = {
        {-1.0f, 0.0f,  0.0f},
        {-0.6f, 0.0f,  0.6f},
        {-0.2f, 0.0f, -0.6f},
        { 0.2f, 0.0f,  0.6f},
        { 0.6f, 0.0f, -0.6f},
        { 1.0f, 0.0f,  0.0f},
    };
    const int n = 6;
    const float tube_radius = 0.05f;
    // Cubic uniform B-spline spans 4 control points per span: cp[i-1..i+2].
    // Valid interior span indices are i ∈ [1, n-3], i.e. the parameter range
    // maps to u ∈ [1, n-3]. We clamp to avoid out-of-bounds access at the
    // endpoints while staying inside the basis's support window.
    auto nurbs_eval = [&](float t) -> Vec3 {
        const int i_lo = 1;
        const int i_hi = n - 3;  // = 3 for n=6
        float u = static_cast<float>(i_lo) + t * static_cast<float>(i_hi - i_lo);
        int i = static_cast<int>(std::floor(u));
        if (i >= i_hi) { i = i_hi; u = static_cast<float>(i); }
        const float f = std::clamp(u - static_cast<float>(i), 0.0f, 1.0f);
        const float b0 = (1.0f - f) * (1.0f - f) * (1.0f - f) / 6.0f;
        const float b1 = (3.0f * f * f * f - 6.0f * f * f + 4.0f) / 6.0f;
        const float b2 = (-3.0f * f * f * f + 3.0f * f * f + 3.0f * f + 1.0f) / 6.0f;
        const float b3 = f * f * f / 6.0f;
        return cp[i - 1] * b0 + cp[i] * b1 + cp[i + 1] * b2 + cp[i + 2] * b3;
    };
    std::vector<Vec3> curve(samples);
    std::vector<Vec3> tangent(samples);
    for (int i = 0; i < samples; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(samples - 1);
        curve[i] = nurbs_eval(t);
    }
    for (int i = 0; i < samples; ++i) {
        const Vec3 prev = curve[std::max(0, i - 1)];
        const Vec3 next = curve[std::min(samples - 1, i + 1)];
        Vec3 t = next - prev;
        const float len = std::max(1e-8f, t.length());
        t = t * (1.0f / len);
        tangent[i] = t;
    }
    mesh.vertices.reserve(static_cast<size_t>(samples) * tube_segments);
    for (int i = 0; i < samples; ++i) {
        const Vec3 t = tangent[i];
        Vec3 ref = (std::fabs(t.y) < 0.9f) ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
        const float dot = ref.x * t.x + ref.y * t.y + ref.z * t.z;
        ref.x -= t.x * dot; ref.y -= t.y * dot; ref.z -= t.z * dot;
        const float rl = std::max(1e-8f, std::sqrt(ref.x * ref.x + ref.y * ref.y + ref.z * ref.z));
        ref = ref * (1.0f / rl);
        const Vec3 bit{
            t.y * ref.z - t.z * ref.y,
            t.z * ref.x - t.x * ref.z,
            t.x * ref.y - t.y * ref.x
        };
        for (int j = 0; j < tube_segments; ++j) {
            const float a = static_cast<float>(j) / static_cast<float>(tube_segments) * kTau;
            const float ca = std::cos(a), sa = std::sin(a);
            const Vec3 offset{
                ref.x * ca + bit.x * sa,
                ref.y * ca + bit.y * sa,
                ref.z * ca + bit.z * sa,
            };
            mesh.vertices.push_back(curve[i] + offset * tube_radius);
        }
    }
    const auto idx = [&](int ring, int side) -> uint32_t {
        ring = std::clamp(ring, 0, samples - 1);
        side = ((side % tube_segments) + tube_segments) % tube_segments;
        return static_cast<uint32_t>(ring * tube_segments + side);
    };
    mesh.triangles.reserve(static_cast<size_t>(samples - 1) * tube_segments * 2);
    for (int i = 0; i < samples - 1; ++i) {
        for (int j = 0; j < tube_segments; ++j) {
            const uint32_t a = idx(i, j);
            const uint32_t b = idx(i + 1, j);
            const uint32_t c = idx(i + 1, j + 1);
            const uint32_t d = idx(i, j + 1);
            mesh.triangles.push_back({a, b, c});
            mesh.triangles.push_back({a, c, d});
        }
    }
    mesh.calculate_normals();
    return mesh;
}

MeshData make_plane_mesh() {
    MeshData mesh;
    mesh.vertices = {
        {-1.0f, 0.0f, -1.0f},
        { 1.0f, 0.0f, -1.0f},
        { 1.0f, 0.0f,  1.0f},
        {-1.0f, 0.0f,  1.0f},
    };
    mesh.triangles = {{0, 2, 1}, {0, 3, 2}};
    mesh.calculate_normals();
    return mesh;
}

MeshData make_circle_mesh(int segments) {
    MeshData mesh;
    segments = std::clamp(segments, 3, 256);
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTau = 2.0f * kPi;
    mesh.vertices.reserve(static_cast<size_t>(segments) + 1);
    mesh.vertices.push_back({0.0f, 0.0f, 0.0f});
    for (int i = 0; i < segments; ++i) {
        const float a = static_cast<float>(i) / static_cast<float>(segments) * kTau;
        mesh.vertices.push_back({std::cos(a), 0.0f, std::sin(a)});
    }
    mesh.triangles.reserve(segments);
    for (int i = 0; i < segments; ++i) {
        const uint32_t a = 0;
        const uint32_t b = 1 + static_cast<uint32_t>(i);
        const uint32_t c = 1 + static_cast<uint32_t>((i + 1) % segments);
        mesh.triangles.push_back({a, c, b});
    }
    mesh.calculate_normals();
    return mesh;
}

MeshData make_grid_mesh(int divisions) {
    MeshData mesh;
    divisions = std::clamp(divisions, 1, 64);
    const int verts_per_side = divisions + 1;
    mesh.vertices.reserve(static_cast<size_t>(verts_per_side) * verts_per_side);
    for (int j = 0; j < verts_per_side; ++j) {
        for (int i = 0; i < verts_per_side; ++i) {
            const float x = -1.0f + 2.0f * static_cast<float>(i) / static_cast<float>(divisions);
            const float z = -1.0f + 2.0f * static_cast<float>(j) / static_cast<float>(divisions);
            mesh.vertices.push_back({x, 0.0f, z});
        }
    }
    const auto idx = [&](int i, int j) -> uint32_t {
        return static_cast<uint32_t>(j * verts_per_side + i);
    };
    mesh.triangles.reserve(static_cast<size_t>(divisions) * divisions * 2);
    for (int j = 0; j < divisions; ++j) {
        for (int i = 0; i < divisions; ++i) {
            const uint32_t a = idx(i, j);
            const uint32_t b = idx(i + 1, j);
            const uint32_t c = idx(i + 1, j + 1);
            const uint32_t d = idx(i, j + 1);
            mesh.triangles.push_back({a, c, b});
            mesh.triangles.push_back({a, d, c});
        }
    }
    mesh.calculate_normals();
    return mesh;
}

} // namespace mechatron
