#pragma once

// ============================================================================
// SculptEngine — Blender-style sculpt engine for EditableMesh.
//
// Stage 4 baseline. This module owns:
//   - SculptSession: per-stroke state (active stroke, mask cache, symmetry)
//   - BrushType: enum covering 7 first-class brush algorithms; the remaining
//     22 of Blender's 29 brushes are aliases / parameter presets of these
//     (e.g. "Clay" ≈ Draw with plane projection, "Scrape" ≈ Flatten negative,
//     "Crease" ≈ Pinch+Draw). The full 29-brush mapping is documented in
//     SculptEngine.cpp.
//   - SpatialHash: a uniform-grid acceleration structure for nearest-vertex
//     queries (PBVH-lite).
//   - Ray-mesh intersection: hit-test for brush cursor placement.
//
// Design notes:
//   - Brushes deform EditableMesh vertices directly. The host (ModelEditor)
//     pushes a single undo snapshot per stroke (begin), so undo granularity
//     matches Blender (one stroke = one undo step).
//   - SculptSession caches per-vertex masks in an EditableMesh Float custom
//     layer named "sculpt_mask". Face sets live in an Int custom layer named
//     "sculpt_face_sets".
//   - Symmetry is mirrored at the engine level: when a stroke touches vertex
//     v, the engine also touches v's mirror partner across the configured axis
//     plane (X by default).
//
// This baseline covers Blender's 29 brushes as follows:
//
//   Algorithm (BrushType)         | Blender brush IDs covered
//   ------------------------------+-------------------------------------------
//   Draw                          | Draw, Draw Sharp, Clay, Clay Strips,
//                                 | Clay Thumb, Layer
//   Grab                          | Grab, Grab Silhouette, Snake Hook,
//                                 | Thumb, Pose
//   Smooth                        | Smooth, Surface Smooth, Enhance Details,
//                                 | Relax, Relax Topology, BMesh Topology Rake,
//                                 | Topology Slide
//   Pinch                         | Pinch, Crease
//   Inflate                       | Inflate, Rotate
//   Flatten                       | Flatten, Fill, Scrape, Multiplane Scrape
//   Mask                          | Mask, Draw Face Sets, Draw Vector
//                                 | Displacement
//
// The remaining SCULPT_OT_* operators (symmetrize, mask_filter, mesh_filter,
// face_sets_create, mask_extract, expand, etc) are exposed as separate calls
// so the host can wire them to pie menus / hotkeys.
// ============================================================================

#include "cad/CADKernel.hpp"
#include "cad/EditableMesh.hpp"
#include "core/Types.hpp"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace mechatron {

// First-class brush algorithms. Each algorithm encapsulates the math for one
// family of Blender brushes. The full 29 brush names map to these via
// BrushPreset in SculptEngine.cpp.
enum class BrushType {
    Draw = 0,
    Grab,
    Smooth,
    Pinch,
    Inflate,
    Flatten,
    Mask,
    Count
};

// Falloff curve (Blender: Smooth, Sphere, Root, InverseSquare, Sharp, Linear,
// Constant, Random). Stored as a per-vertex weight in [0,1] for any brush
// stroke.
enum class BrushFalloff {
    Smooth = 0,    // cos curve
    Sphere,        // 1 - t^2
    Root,          // sqrt(1 - t^2)
    InverseSquare, // 1 / (1 + t^2)
    Sharp,         // 1 - t^4
    Linear,        // 1 - t
    Constant,      // 1
    Random         // noise
};

// Symmetry axes. Mirrored strokes deform the corresponding mirrored vertex
// at the same time as the primary stroke.
enum class SculptSymmetry {
    None = 0,
    X = 1,
    Y = 2,
    Z = 4,
    XZ = X | Z,
    XYZ = X | Y | Z
};

// One-time brush configuration. Set by host before stroke begins.
struct BrushSettings {
    BrushType type = BrushType::Draw;
    float radius = 0.5f;            // World units
    float strength = 0.5f;          // 0..1
    BrushFalloff falloff = BrushFalloff::Smooth;
    float auto_smooth = 0.0f;       // 0..1 fraction of smoothing per stroke
    bool invert = false;            // Ctrl held: invert brush (valley instead of mound)
};

// Per-stroke state owned by the host. begin_stroke() initializes it,
// stroke_to() advances it, end_stroke() commits + pushes undo.
struct SculptSession {
    bool active = false;
    BrushSettings brush;

    // Symmetry bitmask (SculptSymmetry::X | Y | Z).
    int symmetry_mask = 0;
    float symmetry_origin[3] = {0, 0, 0};

    // Stroke sample trail (for grab/snake-hook etc). Edit-space.
    Vec3 last_point{0, 0, 0};
    Vec3 current_point{0, 0, 0};
    bool first_sample = true;

    // Mask cache (mirror of EditableMesh's "sculpt_mask" Float layer for
    // fast lookup). Sized to vertex_count(). 0 = unmasked, 1 = fully masked.
    std::vector<float> mask_cache;

    // Grab anchor: original positions of vertices inside the brush radius at
    // stroke begin. Used by Grab/Snake Hook so the brush pulls a frozen set.
    std::vector<std::pair<uint32_t, Vec3>> grab_anchor;
};

// PBVH-lite spatial hash. Rebuild per stroke begin (or when topology changes).
class SpatialHash {
public:
    void build(const std::vector<Vec3>& vertices, float cell_size);
    void clear();
    // Returns all vertex indices whose world-space distance to `point` is at
    // most `radius`. The vertex positions are snapshotted in build() so the
    // host can mutate the mesh's own vertex array during a stroke without
    // invalidating this cache.
    void query_sphere(const Vec3& point, float radius,
                      std::vector<uint32_t>& out_indices) const;
    size_t cell_count() const { return m_cells.size(); }
private:
    float m_cell_size = 1.0f;
    float m_inv_cell = 1.0f;
    std::vector<Vec3> m_points;
    std::unordered_map<int64_t, std::vector<uint32_t>> m_cells;
    static int64_t cell_key(int x, int y, int z) {
        return (int64_t)x * 73856093LL ^ (int64_t)y * 19349663LL ^ (int64_t)z * 83492791LL;
    }
};

// Ray-mesh intersection result. Edit-space.
struct RayHit {
    bool hit = false;
    Vec3 point{0, 0, 0};
    uint32_t triangle_index = UINT32_MAX;
    float t = std::numeric_limits<float>::max();
};

// Public API. The host (ModelEditor) owns the EditableMesh; SculptEngine is
// stateless beyond the per-session state passed in.
class SculptEngine {
public:
    SculptEngine() = default;
    ~SculptEngine() = default;

    // ----- Stroke lifecycle -----
    // begin_stroke() snapshots the mask cache and grab anchor.
    static void begin_stroke(SculptSession& session,
                             EditableMesh& mesh,
                             const Vec3& start_point);
    // stroke_to() deforms the mesh from last_point to new_point. Returns true
    // if any vertices were modified.
    static bool stroke_to(SculptSession& session,
                          EditableMesh& mesh,
                          const SpatialHash& spatial,
                          const Vec3& new_point);
    // end_stroke() clears the active flag. Host pushes undo at this point.
    static void end_stroke(SculptSession& session);

    // ----- Geometry helpers -----
    static RayHit ray_mesh_intersect(const EditableMesh& mesh,
                                     const Vec3& origin,
                                     const Vec3& direction);

    // ----- Brush math -----
    // Falloff weight in [0,1] for a vertex at distance d from brush center.
    static float falloff_weight(BrushFalloff curve, float radius, float distance);

    // ----- SCULPT_OT_* operators (one-shot, not part of a stroke) -----
    // Symmetrize mesh across the configured axis. Merges near-boundary verts.
    static void symmetrize(EditableMesh& mesh, int axis);

    // Mesh filter: smooth/sharpen/enhance/etc over entire mesh.
    enum class MeshFilterMode {
        Smooth, Sharpen, EnhanceDetails, Inflate, Relax
    };
    static void mesh_filter(EditableMesh& mesh, MeshFilterMode mode,
                            int iterations, float strength);

    // Mask filter: smooth/grow/shrink/invert/fill entire mesh.
    enum class MaskFilterMode {
        Smooth, Grow, Shrink, Increase, Decrease, Invert, Fill, Clear
    };
    static void mask_filter(EditableMesh& mesh, MaskFilterMode mode);

    // Get/set per-vertex mask. Lazily creates the "sculpt_mask" layer.
    static float get_vertex_mask(const EditableMesh& mesh, uint32_t vertex);
    static void set_vertex_mask(EditableMesh& mesh, uint32_t vertex, float value);

    // Pretty brush name (TR + EN).
    static const char* brush_name_tr(BrushType type);
    static const char* brush_name_en(BrushType type);
};

} // namespace mechatron
