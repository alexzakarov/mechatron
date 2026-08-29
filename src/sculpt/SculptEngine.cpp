// ============================================================================
// SculptEngine.cpp — Stage 4 baseline implementation.
// See SculptEngine.hpp for the brush-algorithm-to-Blender-brush mapping.
// ============================================================================

#include "SculptEngine.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace mechatron {

namespace {

constexpr float kPi = 3.14159265358979323846f;

// Möller-Trumbore ray-triangle intersection. Returns barycentric hit if any.
bool ray_triangle_intersect(const Vec3& origin,
                            const Vec3& dir,
                            const Vec3& v0,
                            const Vec3& v1,
                            const Vec3& v2,
                            float& out_t) {
    constexpr float kEps = 1e-7f;
    const Vec3 edge1 = v1 - v0;
    const Vec3 edge2 = v2 - v0;
    const Vec3 pvec = Vec3{
        dir.y * edge2.z - dir.z * edge2.y,
        dir.z * edge2.x - dir.x * edge2.z,
        dir.x * edge2.y - dir.y * edge2.x,
    };
    const float det = edge1.x * pvec.x + edge1.y * pvec.y + edge1.z * pvec.z;
    if (std::fabs(det) < kEps) return false;
    const float inv_det = 1.0f / det;
    const Vec3 tvec = origin - v0;
    const float u = (tvec.x * pvec.x + tvec.y * pvec.y + tvec.z * pvec.z) * inv_det;
    if (u < 0.0f || u > 1.0f) return false;
    const Vec3 qvec = Vec3{
        tvec.y * edge1.z - tvec.z * edge1.y,
        tvec.z * edge1.x - tvec.x * edge1.z,
        tvec.x * edge1.y - tvec.y * edge1.x,
    };
    const float v = (dir.x * qvec.x + dir.y * qvec.y + dir.z * qvec.z) * inv_det;
    if (v < 0.0f || u + v > 1.0f) return false;
    const float t = (edge2.x * qvec.x + edge2.y * qvec.y + edge2.z * qvec.z) * inv_det;
    if (t < 0.0f) return false;
    out_t = t;
    return true;
}

// Triangle area (used by Smooth / Laplacian). Returns 0 for degenerate tris.
float triangle_area(const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 e1 = b - a;
    const Vec3 e2 = c - a;
    const Vec3 cross{
        e1.y * e2.z - e1.z * e2.y,
        e1.z * e2.x - e1.x * e2.z,
        e1.x * e2.y - e1.y * e2.x,
    };
    return 0.5f * cross.length();
}

} // namespace

// ---------------------------------------------------------------------------
// SpatialHash
// ---------------------------------------------------------------------------
void SpatialHash::build(const std::vector<Vec3>& vertices, float cell_size) {
    m_cell_size = std::max(1e-3f, cell_size);
    m_inv_cell = 1.0f / m_cell_size;
    m_cells.clear();
    m_points = vertices;
    for (uint32_t i = 0; i < static_cast<uint32_t>(vertices.size()); ++i) {
        const Vec3& v = vertices[i];
        const int cx = static_cast<int>(std::floor(v.x * m_inv_cell));
        const int cy = static_cast<int>(std::floor(v.y * m_inv_cell));
        const int cz = static_cast<int>(std::floor(v.z * m_inv_cell));
        m_cells[cell_key(cx, cy, cz)].push_back(i);
    }
}

void SpatialHash::clear() {
    m_cells.clear();
    m_points.clear();
}

void SpatialHash::query_sphere(const Vec3& point, float radius,
                               std::vector<uint32_t>& out_indices) const {
    out_indices.clear();
    const int r = static_cast<int>(std::ceil(radius * m_inv_cell));
    const int cx = static_cast<int>(std::floor(point.x * m_inv_cell));
    const int cy = static_cast<int>(std::floor(point.y * m_inv_cell));
    const int cz = static_cast<int>(std::floor(point.z * m_inv_cell));
    const float r2 = radius * radius;
    for (int dz = -r; dz <= r; ++dz) {
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                auto it = m_cells.find(cell_key(cx + dx, cy + dy, cz + dz));
                if (it == m_cells.end()) continue;
                for (uint32_t idx : it->second) {
                    if (idx >= m_points.size()) continue;
                    const Vec3& p = m_points[idx];
                    const float ddx = p.x - point.x;
                    const float ddy = p.y - point.y;
                    const float ddz = p.z - point.z;
                    const float d2 = ddx * ddx + ddy * ddy + ddz * ddz;
                    if (d2 > r2) continue;
                    out_indices.push_back(idx);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Ray-mesh intersection
// ---------------------------------------------------------------------------
RayHit SculptEngine::ray_mesh_intersect(const EditableMesh& mesh,
                                       const Vec3& origin,
                                       const Vec3& direction) {
    RayHit result;
    result.hit = false;
    result.t = std::numeric_limits<float>::max();
    const std::vector<Triangle>& tris = mesh.triangles();
    const std::vector<Vec3>& verts = mesh.vertices();
    Vec3 dir = direction;
    const float len = std::max(1e-8f, dir.length());
    dir = dir * (1.0f / len);

    for (uint32_t i = 0; i < static_cast<uint32_t>(tris.size()); ++i) {
        const Triangle& t = tris[i];
        if (t.v0 >= verts.size() || t.v1 >= verts.size() || t.v2 >= verts.size()) continue;
        float tt;
        if (ray_triangle_intersect(origin, dir, verts[t.v0], verts[t.v1], verts[t.v2], tt)) {
            if (tt < result.t) {
                result.t = tt;
                result.hit = true;
                result.triangle_index = i;
            }
        }
    }
    if (result.hit) {
        result.point = Vec3{origin.x + dir.x * result.t,
                            origin.y + dir.y * result.t,
                            origin.z + dir.z * result.t};
    }
    return result;
}

// ---------------------------------------------------------------------------
// Falloff
// ---------------------------------------------------------------------------
float SculptEngine::falloff_weight(BrushFalloff curve, float radius, float distance) {
    if (radius <= 1e-6f) return 0.0f;
    if (distance > radius) return 0.0f;
    const float t = std::clamp(distance / radius, 0.0f, 1.0f);
    switch (curve) {
        case BrushFalloff::Smooth:
            return 0.5f * (std::cos(t * kPi) + 1.0f);
        case BrushFalloff::Sphere:
            return 1.0f - t * t;
        case BrushFalloff::Root:
            return std::sqrt(std::max(0.0f, 1.0f - t * t));
        case BrushFalloff::InverseSquare:
            return 1.0f / (1.0f + t * t * 8.0f);
        case BrushFalloff::Sharp:
            return std::pow(1.0f - t, 4.0f);
        case BrushFalloff::Linear:
            return 1.0f - t;
        case BrushFalloff::Constant:
            return 1.0f;
        case BrushFalloff::Random:
            return (std::sin(distance * 991.0f) * 0.5f + 0.5f) * (1.0f - t);
    }
    return 1.0f - t;
}

// ---------------------------------------------------------------------------
// Mask helpers — these go through EditableMesh's public mask accessors so the
// private custom-layer API stays internal.
// ---------------------------------------------------------------------------
float SculptEngine::get_vertex_mask(const EditableMesh& mesh, uint32_t vertex) {
    return mesh.get_vertex_mask(vertex);
}

void SculptEngine::set_vertex_mask(EditableMesh& mesh, uint32_t vertex, float value) {
    mesh.set_vertex_mask(vertex, value);
}

// ---------------------------------------------------------------------------
// Stroke lifecycle
// ---------------------------------------------------------------------------
void SculptEngine::begin_stroke(SculptSession& session,
                                EditableMesh& mesh,
                                const Vec3& start_point) {
    session.active = true;
    session.first_sample = true;
    session.last_point = start_point;
    session.current_point = start_point;

    // Cache masks.
    session.mask_cache.assign(mesh.vertex_count(), 0.0f);
    for (size_t i = 0; i < mesh.vertex_count(); ++i) {
        session.mask_cache[i] = mesh.get_vertex_mask(static_cast<uint32_t>(i));
    }

    // Capture grab anchor for Grab-family brushes.
    session.grab_anchor.clear();
    if (session.brush.type == BrushType::Grab) {
        // Anchor is filled lazily on first stroke_to.
    }
}

bool SculptEngine::stroke_to(SculptSession& session,
                             EditableMesh& mesh,
                             const SpatialHash& spatial,
                             const Vec3& new_point) {
    if (!session.active) return false;
    session.current_point = new_point;

    const float radius = std::max(1e-3f, session.brush.radius);
    const float strength = std::clamp(session.brush.strength, 0.0f, 1.0f);
    const BrushType type = session.brush.type;
    const bool invert = session.brush.invert;
    const BrushFalloff falloff = session.brush.falloff;

    std::vector<uint32_t> hit_indices;
    spatial.query_sphere(new_point, radius, hit_indices);
    if (hit_indices.empty()) {
        session.last_point = new_point;
        session.first_sample = false;
        return false;
    }

    const std::vector<Vec3>& verts = mesh.vertices();
    std::vector<Vec3>& verts_mut = mesh.vertices_mut();

    // Snapshot original positions for Grab (anchor only on first sample).
    bool first_sample = session.first_sample;

    // Helper: apply mask weight to a falloff value.
    auto mask_weight = [&](uint32_t idx, float w) {
        if (idx < session.mask_cache.size()) {
            const float m = session.mask_cache[idx];
            return w * (1.0f - m);
        }
        return w;
    };

    // Helper: find mirror partner across symmetry axis. Returns the closest
    // vertex on the opposite side within a small tolerance.
    auto find_mirror_partner = [&](uint32_t idx, int axis) -> uint32_t {
        if (axis < 0 || axis > 2) return UINT32_MAX;
        const Vec3& v = verts[idx];
        Vec3 mirrored = v;
        if (axis == 0) mirrored.x = 2.0f * session.symmetry_origin[0] - v.x;
        if (axis == 1) mirrored.y = 2.0f * session.symmetry_origin[1] - v.y;
        if (axis == 2) mirrored.z = 2.0f * session.symmetry_origin[2] - v.z;
        uint32_t best = UINT32_MAX;
        float best_d2 = (radius * 0.1f) * (radius * 0.1f);
        std::vector<uint32_t> cand;
        spatial.query_sphere(mirrored, radius * 0.1f, cand);
        for (uint32_t c : cand) {
            if (c == idx) continue;
            const Vec3& cv = verts[c];
            const float d2 = (cv.x - mirrored.x) * (cv.x - mirrored.x) +
                             (cv.y - mirrored.y) * (cv.y - mirrored.y) +
                             (cv.z - mirrored.z) * (cv.z - mirrored.z);
            if (d2 < best_d2) {
                best_d2 = d2;
                best = c;
            }
        }
        return best;
    };

    bool any_changed = false;

    switch (type) {
        case BrushType::Draw: {
            // Deform vertices along their normals by strength * falloff.
            for (uint32_t idx : hit_indices) {
                if (idx >= verts.size()) continue;
                const Vec3& v = verts[idx];
                const float d = std::sqrt((v.x - new_point.x) * (v.x - new_point.x) +
                                          (v.y - new_point.y) * (v.y - new_point.y) +
                                          (v.z - new_point.z) * (v.z - new_point.z));
                if (d > radius) continue;
                float w = falloff_weight(falloff, radius, d);
                w = mask_weight(idx, w);
                const Vec3 n = mesh.vertex_normal(idx);
                const float sign = invert ? -1.0f : 1.0f;
                const float delta = sign * strength * w * radius * 0.2f;
                verts_mut[idx] = Vec3{v.x + n.x * delta, v.y + n.y * delta, v.z + n.z * delta};
                any_changed = true;
            }
            break;
        }
        case BrushType::Inflate: {
            // Like Draw but pushes vertices along their normals with bigger
            // displacement (inflation rather than translation).
            for (uint32_t idx : hit_indices) {
                if (idx >= verts.size()) continue;
                const Vec3& v = verts[idx];
                const float d = std::sqrt((v.x - new_point.x) * (v.x - new_point.x) +
                                          (v.y - new_point.y) * (v.y - new_point.y) +
                                          (v.z - new_point.z) * (v.z - new_point.z));
                if (d > radius) continue;
                float w = falloff_weight(falloff, radius, d);
                w = mask_weight(idx, w);
                const Vec3 n = mesh.vertex_normal(idx);
                const float sign = invert ? -1.0f : 1.0f;
                const float delta = sign * strength * w * radius * 0.4f;
                verts_mut[idx] = Vec3{v.x + n.x * delta, v.y + n.y * delta, v.z + n.z * delta};
                any_changed = true;
            }
            break;
        }
        case BrushType::Pinch: {
            // Pull vertices toward brush center (or push out if inverted).
            for (uint32_t idx : hit_indices) {
                if (idx >= verts.size()) continue;
                const Vec3& v = verts[idx];
                const float d = std::sqrt((v.x - new_point.x) * (v.x - new_point.x) +
                                          (v.y - new_point.y) * (v.y - new_point.y) +
                                          (v.z - new_point.z) * (v.z - new_point.z));
                if (d > radius) continue;
                float w = falloff_weight(falloff, radius, d);
                w = mask_weight(idx, w);
                const float sign = invert ? -1.0f : 1.0f;
                const float step = sign * strength * w * 0.3f;
                Vec3 dir{
                    (new_point.x - v.x) * step,
                    (new_point.y - v.y) * step,
                    (new_point.z - v.z) * step,
                };
                verts_mut[idx] = Vec3{v.x + dir.x, v.y + dir.y, v.z + dir.z};
                any_changed = true;
            }
            break;
        }
        case BrushType::Flatten: {
            // Compute average plane through vertices in brush; push verts
            // toward that plane. Invert = Fill (positive side only).
            Vec3 centroid{0, 0, 0};
            Vec3 normal{0, 0, 0};
            int count = 0;
            for (uint32_t idx : hit_indices) {
                if (idx >= verts.size()) continue;
                const Vec3& v = verts[idx];
                const float d = std::sqrt((v.x - new_point.x) * (v.x - new_point.x) +
                                          (v.y - new_point.y) * (v.y - new_point.y) +
                                          (v.z - new_point.z) * (v.z - new_point.z));
                if (d > radius) continue;
                centroid.x += v.x; centroid.y += v.y; centroid.z += v.z;
                const Vec3 n = mesh.vertex_normal(idx);
                normal.x += n.x; normal.y += n.y; normal.z += n.z;
                ++count;
            }
            if (count == 0) break;
            const float inv_n = 1.0f / static_cast<float>(count);
            centroid = centroid * inv_n;
            const float nlen = std::max(1e-8f, normal.length());
            normal = normal * (1.0f / nlen);
            for (uint32_t idx : hit_indices) {
                if (idx >= verts.size()) continue;
                const Vec3& v = verts[idx];
                const float d = std::sqrt((v.x - new_point.x) * (v.x - new_point.x) +
                                          (v.y - new_point.y) * (v.y - new_point.y) +
                                          (v.z - new_point.z) * (v.z - new_point.z));
                if (d > radius) continue;
                float w = falloff_weight(falloff, radius, d);
                w = mask_weight(idx, w);
                const Vec3 rel{v.x - centroid.x, v.y - centroid.y, v.z - centroid.z};
                const float dist = rel.x * normal.x + rel.y * normal.y + rel.z * normal.z;
                // Inverted (Ctrl) = Fill: only push vertices on the same side
                // as the original normal direction; non-inverted = Flatten.
                if (invert && dist < 0.0f) continue;
                const float step = -dist * strength * w * 0.5f;
                verts_mut[idx] = Vec3{
                    v.x + normal.x * step,
                    v.y + normal.y * step,
                    v.z + normal.z * step,
                };
                any_changed = true;
            }
            break;
        }
        case BrushType::Smooth: {
            // Laplacian smoothing: move vertex toward average of its neighbors.
            for (uint32_t idx : hit_indices) {
                if (idx >= verts.size()) continue;
                const Vec3& v = verts[idx];
                const float d = std::sqrt((v.x - new_point.x) * (v.x - new_point.x) +
                                          (v.y - new_point.y) * (v.y - new_point.y) +
                                          (v.z - new_point.z) * (v.z - new_point.z));
                if (d > radius) continue;
                float w = falloff_weight(falloff, radius, d);
                w = mask_weight(idx, w);
                // Find neighbors via vertex-edge adjacency.
                std::vector<EdgeKey> edges = mesh.get_vertex_edges(idx);
                if (edges.empty()) continue;
                Vec3 avg{0, 0, 0};
                int n = 0;
                for (const EdgeKey& ek : edges) {
                    const uint32_t other = (ek.v0 == idx) ? ek.v1 : (ek.v1 == idx ? ek.v0 : UINT32_MAX);
                    if (other >= verts.size()) continue;
                    avg.x += verts[other].x;
                    avg.y += verts[other].y;
                    avg.z += verts[other].z;
                    ++n;
                }
                if (n == 0) continue;
                avg = avg * (1.0f / static_cast<float>(n));
                const float step = strength * w * 0.5f;
                verts_mut[idx] = Vec3{
                    v.x + (avg.x - v.x) * step,
                    v.y + (avg.y - v.y) * step,
                    v.z + (avg.z - v.z) * step,
                };
                any_changed = true;
            }
            break;
        }
        case BrushType::Grab: {
            // Move anchor vertices by stroke delta.
            if (first_sample) {
                for (uint32_t idx : hit_indices) {
                    if (idx >= verts.size()) continue;
                    const Vec3& v = verts[idx];
                    const float d = std::sqrt((v.x - new_point.x) * (v.x - new_point.x) +
                                              (v.y - new_point.y) * (v.y - new_point.y) +
                                              (v.z - new_point.z) * (v.z - new_point.z));
                    if (d > radius) continue;
                    const float w = falloff_weight(falloff, radius, d);
                    session.grab_anchor.push_back({idx, v});
                    (void)w;  // falloff applied during stroke
                }
            }
            const Vec3 delta{
                new_point.x - session.last_point.x,
                new_point.y - session.last_point.y,
                new_point.z - session.last_point.z,
            };
            for (auto& [idx, base] : session.grab_anchor) {
                if (idx >= verts.size()) continue;
                const Vec3& v = verts[idx];
                const float d = std::sqrt((v.x - new_point.x) * (v.x - new_point.x) +
                                          (v.y - new_point.y) * (v.y - new_point.y) +
                                          (v.z - new_point.z) * (v.z - new_point.z));
                if (d > radius * 2.0f) continue;
                const float w = mask_weight(idx, falloff_weight(falloff, radius, d));
                verts_mut[idx] = Vec3{
                    base.x + delta.x * w,
                    base.y + delta.y * w,
                    base.z + delta.z * w,
                };
                any_changed = true;
            }
            break;
        }
        case BrushType::Mask: {
            // Paint mask values. Invert decreases mask.
            for (uint32_t idx : hit_indices) {
                if (idx >= verts.size()) continue;
                const Vec3& v = verts[idx];
                const float d = std::sqrt((v.x - new_point.x) * (v.x - new_point.x) +
                                          (v.y - new_point.y) * (v.y - new_point.y) +
                                          (v.z - new_point.z) * (v.z - new_point.z));
                if (d > radius) continue;
                float w = falloff_weight(falloff, radius, d);
                const float current = (idx < session.mask_cache.size())
                    ? session.mask_cache[idx] : 0.0f;
                const float next = invert
                    ? std::max(0.0f, current - strength * w * 0.25f)
                    : std::min(1.0f, current + strength * w * 0.25f);
                set_vertex_mask(mesh, idx, next);
                if (idx < session.mask_cache.size()) session.mask_cache[idx] = next;
                any_changed = true;
            }
            break;
        }
        default:
            break;
    }

    // Symmetry: re-apply stroke for mirrored vertex set.
    if ((session.symmetry_mask & static_cast<int>(SculptSymmetry::X)) != 0 && type != BrushType::Grab) {
        // Apply a mirrored stroke. We approximate by mirroring the current
        // point and re-running the deform for vertices near the mirror.
        Vec3 mirrored_point = new_point;
        mirrored_point.x = 2.0f * session.symmetry_origin[0] - new_point.x;
        std::vector<uint32_t> m_indices;
        spatial.query_sphere(mirrored_point, radius, m_indices);
        for (uint32_t idx : m_indices) {
            if (idx >= verts.size()) continue;
            const Vec3& v = verts[idx];
            const float d = std::sqrt((v.x - mirrored_point.x) * (v.x - mirrored_point.x) +
                                      (v.y - mirrored_point.y) * (v.y - mirrored_point.y) +
                                      (v.z - mirrored_point.z) * (v.z - mirrored_point.z));
            if (d > radius) continue;
            const float w = falloff_weight(falloff, radius, d);
            const Vec3 n = mesh.vertex_normal(idx);
            const float sign = invert ? -1.0f : 1.0f;
            const float delta = (type == BrushType::Mask) ? 0.0f
                               : sign * strength * w * radius * 0.2f;
            if (type != BrushType::Mask) {
                verts_mut[idx] = Vec3{v.x + n.x * delta, v.y + n.y * delta, v.z + n.z * delta};
                any_changed = true;
            }
        }
    }

    session.last_point = new_point;
    session.first_sample = false;
    return any_changed;
}

void SculptEngine::end_stroke(SculptSession& session) {
    session.active = false;
    session.first_sample = true;
    session.grab_anchor.clear();
}

// ---------------------------------------------------------------------------
// SCULPT_OT_* operators
// ---------------------------------------------------------------------------
void SculptEngine::symmetrize(EditableMesh& mesh, int axis) {
    if (axis < 0 || axis > 2) return;
    mesh.symmetrize_tool(axis, true);
}

void SculptEngine::mesh_filter(EditableMesh& mesh, MeshFilterMode mode,
                               int iterations, float strength) {
    if (iterations <= 0) return;
    for (int it = 0; it < iterations; ++it) {
        const std::vector<Vec3> verts_copy = mesh.vertices();
        std::vector<Vec3>& verts_mut = mesh.vertices_mut();
        for (size_t i = 0; i < verts_copy.size() && i < verts_mut.size(); ++i) {
            std::vector<EdgeKey> edges = mesh.get_vertex_edges(static_cast<uint32_t>(i));
            if (edges.empty()) continue;
            Vec3 avg{0, 0, 0};
            int n = 0;
            for (const EdgeKey& ek : edges) {
                const uint32_t other = (ek.v0 == i) ? ek.v1 : (ek.v1 == i ? ek.v0 : UINT32_MAX);
                if (other >= verts_copy.size()) continue;
                avg.x += verts_copy[other].x;
                avg.y += verts_copy[other].y;
                avg.z += verts_copy[other].z;
                ++n;
            }
            if (n == 0) continue;
            avg = avg * (1.0f / static_cast<float>(n));
            const float w = std::clamp(strength, 0.0f, 1.0f);
            switch (mode) {
                case MeshFilterMode::Smooth:
                case MeshFilterMode::Relax:
                case MeshFilterMode::EnhanceDetails:
                    verts_mut[i] = Vec3{
                        verts_copy[i].x + (avg.x - verts_copy[i].x) * w,
                        verts_copy[i].y + (avg.y - verts_copy[i].y) * w,
                        verts_copy[i].z + (avg.z - verts_copy[i].z) * w,
                    };
                    break;
                case MeshFilterMode::Sharpen:
                    verts_mut[i] = Vec3{
                        verts_copy[i].x - (avg.x - verts_copy[i].x) * w * 0.5f,
                        verts_copy[i].y - (avg.y - verts_copy[i].y) * w * 0.5f,
                        verts_copy[i].z - (avg.z - verts_copy[i].z) * w * 0.5f,
                    };
                    break;
                case MeshFilterMode::Inflate: {
                    const Vec3 nrm = mesh.vertex_normal(static_cast<uint32_t>(i));
                    verts_mut[i] = Vec3{
                        verts_copy[i].x + nrm.x * w * 0.05f,
                        verts_copy[i].y + nrm.y * w * 0.05f,
                        verts_copy[i].z + nrm.z * w * 0.05f,
                    };
                    break;
                }
            }
        }
    }
}

void SculptEngine::mask_filter(EditableMesh& mesh, MaskFilterMode mode) {
    const size_t n = mesh.vertex_count();

    // If there's no mask layer, Fill creates one; others are no-ops.
    if (mode != MaskFilterMode::Fill) {
        // Test existence by reading a vertex; if there's no layer the accessor
        // returns 0. We treat a fully-zero read as "no layer".
        // This is the same test EditableMesh uses internally; if mesh_filter
        // is called without prior masking, that's fine.
        bool any_mask = false;
        for (size_t i = 0; i < n && !any_mask; ++i) {
            if (mesh.get_vertex_mask(static_cast<uint32_t>(i)) > 0.0f) any_mask = true;
        }
        if (!any_mask && mode != MaskFilterMode::Invert && mode != MaskFilterMode::Fill) {
            return;
        }
    }

    switch (mode) {
        case MaskFilterMode::Fill:
            for (size_t i = 0; i < n; ++i) mesh.set_vertex_mask(static_cast<uint32_t>(i), 1.0f);
            break;
        case MaskFilterMode::Clear:
            for (size_t i = 0; i < n; ++i) mesh.set_vertex_mask(static_cast<uint32_t>(i), 0.0f);
            break;
        case MaskFilterMode::Invert:
            for (size_t i = 0; i < n; ++i) {
                const float v = mesh.get_vertex_mask(static_cast<uint32_t>(i));
                mesh.set_vertex_mask(static_cast<uint32_t>(i), 1.0f - v);
            }
            break;
        case MaskFilterMode::Increase:
            for (size_t i = 0; i < n; ++i) {
                const float v = mesh.get_vertex_mask(static_cast<uint32_t>(i));
                mesh.set_vertex_mask(static_cast<uint32_t>(i), std::min(1.0f, v + 0.1f));
            }
            break;
        case MaskFilterMode::Decrease:
            for (size_t i = 0; i < n; ++i) {
                const float v = mesh.get_vertex_mask(static_cast<uint32_t>(i));
                mesh.set_vertex_mask(static_cast<uint32_t>(i), std::max(0.0f, v - 0.1f));
            }
            break;
        case MaskFilterMode::Smooth: {
            std::vector<float> copy(n);
            for (size_t i = 0; i < n; ++i) copy[i] = mesh.get_vertex_mask(static_cast<uint32_t>(i));
            for (size_t i = 0; i < n; ++i) {
                std::vector<EdgeKey> edges = mesh.get_vertex_edges(static_cast<uint32_t>(i));
                if (edges.empty()) continue;
                float avg = copy[i];
                int count = 1;
                for (const EdgeKey& ek : edges) {
                    const uint32_t other = (ek.v0 == i) ? ek.v1 : (ek.v1 == i ? ek.v0 : UINT32_MAX);
                    if (other >= n) continue;
                    avg += copy[other];
                    ++count;
                }
                mesh.set_vertex_mask(static_cast<uint32_t>(i), avg / static_cast<float>(count));
            }
            break;
        }
        case MaskFilterMode::Grow: {
            std::vector<float> copy(n);
            for (size_t i = 0; i < n; ++i) copy[i] = mesh.get_vertex_mask(static_cast<uint32_t>(i));
            for (size_t i = 0; i < n; ++i) {
                std::vector<EdgeKey> edges = mesh.get_vertex_edges(static_cast<uint32_t>(i));
                for (const EdgeKey& ek : edges) {
                    const uint32_t other = (ek.v0 == i) ? ek.v1 : (ek.v1 == i ? ek.v0 : UINT32_MAX);
                    if (other >= n) continue;
                    if (copy[other] > copy[i]) {
                        mesh.set_vertex_mask(static_cast<uint32_t>(i),
                                             std::min(1.0f, copy[i] + 0.05f));
                        break;
                    }
                }
            }
            break;
        }
        case MaskFilterMode::Shrink: {
            std::vector<float> copy(n);
            for (size_t i = 0; i < n; ++i) copy[i] = mesh.get_vertex_mask(static_cast<uint32_t>(i));
            for (size_t i = 0; i < n; ++i) {
                std::vector<EdgeKey> edges = mesh.get_vertex_edges(static_cast<uint32_t>(i));
                for (const EdgeKey& ek : edges) {
                    const uint32_t other = (ek.v0 == i) ? ek.v1 : (ek.v1 == i ? ek.v0 : UINT32_MAX);
                    if (other >= n) continue;
                    if (copy[other] < copy[i]) {
                        mesh.set_vertex_mask(static_cast<uint32_t>(i),
                                             std::max(0.0f, copy[i] - 0.05f));
                        break;
                    }
                }
            }
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// Brush names (TR + EN)
// ---------------------------------------------------------------------------
const char* SculptEngine::brush_name_tr(BrushType type) {
    switch (type) {
        case BrushType::Draw:     return "Çiz (Draw)";
        case BrushType::Grab:     return "Tut (Grab)";
        case BrushType::Smooth:   return "Pürüzsüzleştir (Smooth)";
        case BrushType::Pinch:    return "Sıkıştır (Pinch)";
        case BrushType::Inflate:  return "Şişir (Inflate)";
        case BrushType::Flatten:  return "Düzleştir (Flatten)";
        case BrushType::Mask:     return "Maske (Mask)";
        default: return "?";
    }
}

const char* SculptEngine::brush_name_en(BrushType type) {
    switch (type) {
        case BrushType::Draw:     return "Draw";
        case BrushType::Grab:     return "Grab";
        case BrushType::Smooth:   return "Smooth";
        case BrushType::Pinch:    return "Pinch";
        case BrushType::Inflate:  return "Inflate";
        case BrushType::Flatten:  return "Flatten";
        case BrushType::Mask:     return "Mask";
        default: return "?";
    }
}

} // namespace mechatron
