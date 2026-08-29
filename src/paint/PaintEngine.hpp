#pragma once

// ============================================================================
// PaintEngine — Stage 5 paint engine for EditableMesh.
//
// Covers Blender's three paint modes:
//   - Vertex Paint : writes a Color4 custom layer named "vertex_color"
//   - Weight Paint : writes a VertexGroup weight for the active group
//   - Texture Paint: writes a Color4 custom layer named "texture_paint"
//                    (UV-driven; uses the same Color4 storage as vertex color
//                    because Mechatron has no separate image editor)
//
// Brush math (falloff, spatial hash, raycast) is reused from SculptEngine.
// The engine itself is stateless beyond the per-stroke PaintSession the host
// pushes; the host pushes one undo snapshot per stroke (begin).
//
// BrushType covers Blender's 16 paint brushes across the three modes:
//
//   Vertex paint  : Draw, Blur, Smear, Average, Set
//   Weight paint  : Add, Subtract, Blur, Average, Smear
//   Texture paint : Draw, Blur, Smear (treated like vertex paint with UVs)
//
// All brushes share the same falloff curves as SculptEngine. Blend modes
// (Mix, Add, Subtract, Multiply, Lighten, Darken, etc) mirror Blender's
// IMB_blend_mode enum.
// ============================================================================

#include "cad/EditableMesh.hpp"
#include "core/Types.hpp"
#include "sculpt/SculptEngine.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace mechatron {

// Which paint mode we're in. Determines which target (color layer, vertex
// group weight) the brush writes to.
enum class PaintMode {
    Vertex  = 0,    // OB_MODE_VERTEX_PAINT
    Weight  = 1,    // OB_MODE_WEIGHT_PAINT
    Texture = 2,    // OB_MODE_TEXTURE_PAINT
};

// First-class paint brush algorithms. Each Blender brush maps onto one of
// these; the mapping is documented in PaintEngine.cpp.
enum class PaintBrushType {
    Draw     = 0,   // Paint brush: stamp color/weight with falloff
    Blur     = 1,   // Average neighbors (color or weight)
    Smear    = 2,   // Drag color/weight along stroke direction
    Average  = 3,   // Average over stroke area
    Set      = 4,   // Hard set (replace, ignore falloff strength)
    Add      = 5,   // Weight only: increase weight by strength
    Subtract = 6,   // Weight only: decrease weight by strength
    Count
};

// Blender IMB_blend_mode subset. Used by Draw / Set brushes to combine the
// stroke color with the existing vertex color.
enum class PaintBlendMode {
    Mix       = 0,
    Add       = 1,
    Subtract  = 2,
    Multiply  = 3,
    Lighten   = 4,
    Darken    = 5,
    Color     = 6,   // HSV hue from brush, value/sat from base
    Count
};

// One-time brush configuration.
struct PaintBrushSettings {
    PaintBrushType type = PaintBrushType::Draw;
    float radius  = 0.5f;          // World units
    float strength = 0.5f;         // 0..1
    BrushFalloff falloff = BrushFalloff::Smooth;
    PaintBlendMode blend = PaintBlendMode::Mix;
    bool invert = false;           // Ctrl held: secondary action (e.g. erase)
    Vec4 color{1.0f, 1.0f, 1.0f, 1.0f};  // Brush color (RGBA)
    float weight = 1.0f;           // Target weight for Weight Paint mode
    bool auto_normalize = true;    // Weight Paint: normalize all groups
    size_t vertex_group_index = 0; // Weight Paint: target vertex group index
};

// Per-stroke state. begin_stroke() initializes it, stroke_to() advances it,
// end_stroke() commits + pushes undo.
struct PaintSession {
    bool active = false;
    PaintMode mode = PaintMode::Vertex;
    PaintBrushSettings brush;

    // Stroke sample trail (edit-space). Used by Smear to drag values along
    // the stroke direction.
    Vec3 last_point{0, 0, 0};
    Vec3 current_point{0, 0, 0};
    bool first_sample = true;

    // Cache of original colors/weights (snapshotted at stroke begin). Used by
    // Blur/Average so per-stroke averaging is stable.
    std::vector<Vec4>  color_cache;
    std::vector<float> weight_cache;

    // Running average of colors sampled during the stroke (Average brush).
    Vec4  avg_color_accum{0, 0, 0, 0};
    int   avg_color_count = 0;
    float avg_weight_accum = 0.0f;
    int   avg_weight_count = 0;
};

// Public API. The host (ModelEditor) owns the EditableMesh; PaintEngine is
// stateless beyond the per-session state passed in. Brush math (falloff,
// spatial queries, raycast) is reused from SculptEngine.
class PaintEngine {
public:
    PaintEngine() = default;
    ~PaintEngine() = default;

    // ----- Stroke lifecycle -----
    static void begin_stroke(PaintSession& session,
                             EditableMesh& mesh,
                             const Vec3& start_point);
    static bool stroke_to(PaintSession& session,
                          EditableMesh& mesh,
                          const SpatialHash& spatial,
                          const Vec3& new_point);
    static void end_stroke(PaintSession& session);

    // ----- Color / weight access (go through EditableMesh's custom layers
    // and vertex groups) -----
    // Lazily creates the "vertex_color" Color4 layer.
    static Vec4 get_vertex_color(const EditableMesh& mesh, uint32_t vertex);
    static void set_vertex_color(EditableMesh& mesh, uint32_t vertex, const Vec4& color);
    static bool has_vertex_color_layer(const EditableMesh& mesh);

    // Weight access uses the active vertex group on session.brush.vertex_group_index.
    static float get_vertex_weight(const EditableMesh& mesh,
                                   size_t vertex_group_index, uint32_t vertex);
    static void  set_vertex_weight(EditableMesh& mesh,
                                   size_t vertex_group_index,
                                   uint32_t vertex, float weight);

    // ----- Blend helpers -----
    // Blend `base` with `brush_color` using `mode` at fraction `t`. Used by
    // Draw and Set brushes.
    static Vec4 blend_colors(PaintBlendMode mode,
                             const Vec4& base, const Vec4& brush_color, float t);

    // ----- Pretty names (TR + EN) -----
    static const char* brush_name_tr(PaintBrushType type);
    static const char* brush_name_en(PaintBrushType type);
    static const char* blend_mode_name(PaintBlendMode mode);

    // ----- Operator shortcuts (PAINT_OT_*) -----
    // Set all vertex colors to the brush color.
    static void set_all_vertex_colors(EditableMesh& mesh, const Vec4& color);
    // Reset all vertex colors to white opaque.
    static void clear_vertex_colors(EditableMesh& mesh);
    // Set all weights in the active group to a given value.
    static void set_all_weights(EditableMesh& mesh, size_t group_index, float weight);
    // Normalize all vertex weights across all groups.
    static void normalize_all_weights(EditableMesh& mesh);
    // Invert vertex colors (1 - color).
    static void invert_vertex_colors(EditableMesh& mesh);
    // Blur all vertex colors (one iteration of neighbor averaging).
    static void blur_all_vertex_colors(EditableMesh& mesh);
    // Levels adjustment (multiply/add to RGB channels of all vertices).
    static void levels_vertex_colors(EditableMesh& mesh, float gain, float bias);
    // Dirty vertex colors — Blender's "Dirty Vertex Colors" operator: compute
    // curvature from vertex normals and write a black/white gradient.
    static void dirty_vertex_colors(EditableMesh& mesh, float strength);
};

} // namespace mechatron
