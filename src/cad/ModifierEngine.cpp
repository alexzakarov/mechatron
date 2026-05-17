#include "ModifierEngine.hpp"
#include "EditableMesh.hpp"
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace mechatron {

// ============================================================================
// Helper: Mirror modifier (pure mesh-level, no OCCT needed)
// ============================================================================
static bool apply_mirror(const MeshData& base, const ModifierMirror& m, MeshData& out) {
    out = base;

    // Count original vertices for offset
    size_t n = out.vertices.size();

    // Determine axis bitmask
    bool mx = m.x, my = m.y, mz = m.z;
    if (!mx && !my && !mz) return true; // nothing to mirror

    // Generate mirrored copies
    // We handle single-axis mirror; multi-axis is applied iteratively
    std::vector<Vec3> mirrored_verts;
    std::vector<Triangle> mirrored_tris;

    // Generate all axis combinations (for multi-axis mirror)
    struct MirrorCombo { bool x, y, z; };
    std::vector<MirrorCombo> combos;
    for (int fx = (mx ? 0 : 1); fx <= 1; ++fx) {
        for (int fy = (my ? 0 : 1); fy <= 1; ++fy) {
            for (int fz = (mz ? 0 : 1); fz <= 1; ++fz) {
                if (fx == 1 && fy == 1 && fz == 1) continue; // skip identity
                combos.push_back({fx == 0, fy == 0, fz == 0});
            }
        }
    }

    for (const auto& combo : combos) {
        // Create mirrored vertices
        for (size_t i = 0; i < n; ++i) {
            Vec3 v = out.vertices[i];
            if (combo.x) v.x = -v.x;
            if (combo.y) v.y = -v.y;
            if (combo.z) v.z = -v.z;
            mirrored_verts.push_back(v);
        }

        // Determine if we need to flip winding
        int flips = (combo.x ? 1 : 0) + (combo.y ? 1 : 0) + (combo.z ? 1 : 0);
        bool flip = (flips % 2 == 1);

        // Create mirrored triangles
        uint32_t offset = (uint32_t)(out.vertices.size() + mirrored_verts.size() - n);
        for (const auto& t : out.triangles) {
            if (flip) mirrored_tris.push_back(Triangle{offset + t.v0, offset + t.v2, offset + t.v1});
            else mirrored_tris.push_back(Triangle{offset + t.v0, offset + t.v1, offset + t.v2});
        }
    }

    // Append mirrored data
    out.vertices.insert(out.vertices.end(), mirrored_verts.begin(), mirrored_verts.end());
    out.triangles.insert(out.triangles.end(), mirrored_tris.begin(), mirrored_tris.end());

    out.normals.clear();
    out.calculate_normals();
    return true;
}

// ============================================================================
// Helper: Subdivision Surface modifier (Catmull-Clark approximation on triangle mesh)
// ============================================================================
static bool apply_subdivision(const MeshData& base, const ModifierSubdivision& sub, MeshData& out) {
    out = base;

    const int levels = std::clamp(sub.levels, 0, 6);
    for (int level = 0; level < levels; ++level) {
        MeshData refined;
        refined.vertices = out.vertices;

        struct EdgeKey {
            uint32_t v0, v1;
            EdgeKey(uint32_t a, uint32_t b) : v0(std::min(a, b)), v1(std::max(a, b)) {}
            bool operator==(const EdgeKey& o) const { return v0 == o.v0 && v1 == o.v1; }
            struct Hash {
                size_t operator()(const EdgeKey& k) const { return (size_t)k.v0 * 2654435761u ^ (size_t)k.v1; }
            };
        };

        std::unordered_map<EdgeKey, uint32_t, EdgeKey::Hash> edge_midpoint;
        auto midpoint = [&](uint32_t a, uint32_t b) -> uint32_t {
            EdgeKey key(a, b);
            auto it = edge_midpoint.find(key);
            if (it != edge_midpoint.end()) return it->second;
            if (a >= out.vertices.size() || b >= out.vertices.size()) return 0;
            Vec3 mid = (out.vertices[a] + out.vertices[b]) * 0.5f;
            uint32_t idx = static_cast<uint32_t>(refined.vertices.size());
            refined.vertices.push_back(mid);
            edge_midpoint.emplace(key, idx);
            return idx;
        };

        for (const auto& t : out.triangles) {
            if (t.v0 >= out.vertices.size() || t.v1 >= out.vertices.size() || t.v2 >= out.vertices.size()) {
                continue;
            }
            uint32_t a = t.v0, b = t.v1, c = t.v2;
            uint32_t ab = midpoint(a, b);
            uint32_t bc = midpoint(b, c);
            uint32_t ca = midpoint(c, a);

            refined.triangles.push_back(Triangle{a, ab, ca});
            refined.triangles.push_back(Triangle{ab, b, bc});
            refined.triangles.push_back(Triangle{ca, bc, c});
            refined.triangles.push_back(Triangle{ab, bc, ca});
        }

        out = std::move(refined);
    }

    out.normals.clear();
    out.calculate_normals();
    return true;
}

// ============================================================================
// Helper: Bevel modifier (mesh-level all-edge bevel, backed by EditableMesh)
// ============================================================================
static bool apply_bevel(const MeshData& base, const ModifierBevel& bev, MeshData& out) {
    out = base;

    if (bev.amount <= 0.0f) return true;
    if (base.is_empty()) return true;

    EditableMesh editable;
    if (!editable.from_meshdata(base)) {
        return true;
    }

    auto edges = editable.get_edges();
    editable.bevel_edges(edges, bev.amount, 1);
    out = editable.to_meshdata();
    out.normals.clear();
    out.calculate_normals();
    return true;
}

// ============================================================================
// Helper: Weld modifier
// ============================================================================
static bool apply_weld(const MeshData& base, const ModifierWeld& weld, MeshData& out) {
    out = base;
    if (out.is_empty()) return true;

    const float threshold = std::clamp(weld.threshold, 0.0f, 1.0f);
    if (threshold <= 0.0f) return true;

    out.unify_vertices(threshold);
    out.calculate_normals();
    return true;
}

// ============================================================================
// Helper: Array modifier
// ============================================================================
static bool apply_array(const MeshData& base, const ModifierArray& arr, MeshData& out) {
    out.clear();
    if (base.is_empty()) return true;

    const int count = std::clamp(arr.count, 1, 1024);
    out.vertices.reserve(base.vertices.size() * static_cast<size_t>(count));
    out.triangles.reserve(base.triangles.size() * static_cast<size_t>(count));

    for (int i = 0; i < count; ++i) {
        const uint32_t vertex_offset = static_cast<uint32_t>(out.vertices.size());
        const Vec3 delta = arr.offset * static_cast<float>(i);
        for (const Vec3& v : base.vertices) {
            out.vertices.push_back(v + delta);
        }
        for (const Triangle& t : base.triangles) {
            if (t.v0 >= base.vertices.size() || t.v1 >= base.vertices.size() || t.v2 >= base.vertices.size()) continue;
            out.triangles.push_back(Triangle{
                vertex_offset + t.v0,
                vertex_offset + t.v1,
                vertex_offset + t.v2
            });
        }
    }

    out.calculate_normals();
    return true;
}

// ============================================================================
// Helper: Solidify modifier
// ============================================================================
static bool apply_solidify(const MeshData& base, const ModifierSolidify& solid, MeshData& out) {
    out.clear();
    if (base.is_empty()) return true;

    MeshData source = base;
    if (source.normals.size() != source.vertices.size()) {
        source.calculate_normals();
    }

    const float thickness = std::max(0.0f, solid.thickness);
    if (thickness <= 0.0f) {
        out = source;
        return true;
    }

    const float offset = std::clamp(solid.offset, -1.0f, 1.0f);
    const float outer_shift = thickness * (offset + 1.0f) * 0.5f;
    const float inner_shift = thickness * (offset - 1.0f) * 0.5f;
    const uint32_t n = static_cast<uint32_t>(source.vertices.size());

    out.vertices.reserve(source.vertices.size() * 2);
    out.triangles.reserve(source.triangles.size() * 2);
    for (uint32_t i = 0; i < n; ++i) {
        const Vec3 normal = (i < source.normals.size()) ? source.normals[i] : Vec3{0, 1, 0};
        out.vertices.push_back(source.vertices[i] + normal * outer_shift);
    }
    for (uint32_t i = 0; i < n; ++i) {
        const Vec3 normal = (i < source.normals.size()) ? source.normals[i] : Vec3{0, 1, 0};
        out.vertices.push_back(source.vertices[i] + normal * inner_shift);
    }

    struct EdgeCount {
        uint32_t a = 0;
        uint32_t b = 0;
        uint32_t count = 0;
    };
    struct LocalEdge {
        uint32_t a = 0;
        uint32_t b = 0;
        bool operator==(const LocalEdge& o) const { return a == o.a && b == o.b; }
        struct Hash {
            size_t operator()(const LocalEdge& e) const {
                return static_cast<size_t>(e.a) * 2654435761u ^ static_cast<size_t>(e.b);
            }
        };
    };
    std::unordered_map<LocalEdge, EdgeCount, LocalEdge::Hash> edge_counts;
    auto add_edge = [&](uint32_t a, uint32_t b) {
        LocalEdge key{std::min(a, b), std::max(a, b)};
        auto& e = edge_counts[key];
        e.a = a;
        e.b = b;
        e.count++;
    };

    for (const Triangle& t : source.triangles) {
        if (t.v0 >= n || t.v1 >= n || t.v2 >= n) continue;
        out.triangles.push_back(t);
        out.triangles.push_back(Triangle{n + t.v2, n + t.v1, n + t.v0});
        add_edge(t.v0, t.v1);
        add_edge(t.v1, t.v2);
        add_edge(t.v2, t.v0);
    }

    for (const auto& [_, edge] : edge_counts) {
        if (edge.count != 1) continue;
        const uint32_t a = edge.a;
        const uint32_t b = edge.b;
        out.triangles.push_back(Triangle{a, b, n + b});
        out.triangles.push_back(Triangle{a, n + b, n + a});
    }

    out.calculate_normals();
    return true;
}

// ============================================================================
// Main modifier engine
// ============================================================================

bool ModifierEngine::apply(CADKernel& cad, const MeshData& base, const ModifierStack& stack,
                           const std::function<bool(const std::string& asset_id, const std::string& scope, MeshData&)>& load_asset_mesh,
                           MeshData& out, std::string* out_error) {
    out = base;

    for (const auto& m : stack.modifiers) {
        if (std::holds_alternative<ModifierBoolean>(m)) {
            const auto& b = std::get<ModifierBoolean>(m);
            MeshData other;
            if (!load_asset_mesh(b.other_asset_id, b.other_scope, other)) {
                if (out_error) *out_error = "Failed to load boolean operand asset mesh.";
                return false;
            }
            MeshData r;
            bool ok = false;
            switch (b.op) {
                case ModifierBoolean::Op::Union: ok = cad.union_meshes(out, other, r); break;
                case ModifierBoolean::Op::Subtract: ok = cad.subtract_meshes(out, other, r); break;
                case ModifierBoolean::Op::Intersect: ok = cad.intersect_meshes(out, other, r); break;
            }
            if (!ok) {
                if (out_error) *out_error = cad.error().empty() ? "Boolean operation failed." : cad.error();
                return false;
            }
            out = std::move(r);
            continue;
        }

        if (std::holds_alternative<ModifierMirror>(m)) {
            const auto& mm = std::get<ModifierMirror>(m);
            MeshData r;
            if (!apply_mirror(out, mm, r)) {
                if (out_error) *out_error = "Mirror modifier failed.";
                return false;
            }
            out = std::move(r);
            continue;
        }

        if (std::holds_alternative<ModifierSubdivision>(m)) {
            const auto& s = std::get<ModifierSubdivision>(m);
            MeshData r;
            if (!apply_subdivision(out, s, r)) {
                if (out_error) *out_error = "Subdivision modifier failed.";
                return false;
            }
            out = std::move(r);
            continue;
        }

        if (std::holds_alternative<ModifierBevel>(m)) {
            const auto& b = std::get<ModifierBevel>(m);
            MeshData r;
            if (!apply_bevel(out, b, r)) {
                if (out_error) *out_error = "Bevel modifier failed.";
                return false;
            }
            out = std::move(r);
            continue;
        }

        if (std::holds_alternative<ModifierWeld>(m)) {
            const auto& w = std::get<ModifierWeld>(m);
            MeshData r;
            if (!apply_weld(out, w, r)) {
                if (out_error) *out_error = "Weld modifier failed.";
                return false;
            }
            out = std::move(r);
            continue;
        }

        if (std::holds_alternative<ModifierArray>(m)) {
            const auto& a = std::get<ModifierArray>(m);
            MeshData r;
            if (!apply_array(out, a, r)) {
                if (out_error) *out_error = "Array modifier failed.";
                return false;
            }
            out = std::move(r);
            continue;
        }

        if (std::holds_alternative<ModifierSolidify>(m)) {
            const auto& s = std::get<ModifierSolidify>(m);
            MeshData r;
            if (!apply_solidify(out, s, r)) {
                if (out_error) *out_error = "Solidify modifier failed.";
                return false;
            }
            out = std::move(r);
            continue;
        }
    }

    if (out.normals.empty()) out.calculate_normals();
    return true;
}

} // namespace mechatron
