#pragma once

// ============================================================================
// PaintToolbar — Stage 5 paint-mode UI renderer.
//
// Stateless ImGui panel exposing the 16 Blender paint brushes across the
// three modes (Vertex / Weight / Texture). The toolbar edits the PaintSession
// (brush type, color, weight, radius/strength/falloff, blend mode, vertex
// group) and triggers PAINT_OT_* one-shot operators (set all, clear, invert,
// levels, dirty vertex colors, normalize weights, blur all).
//
// The toolbar does NOT own state — the host (ModelEditor) owns the
// PaintSession + SpatialHash and invokes PaintEngine::begin_stroke /
// stroke_to / end_stroke in response to viewport mouse events. Brush math
// (falloff, spatial hash) is reused from SculptEngine.
//
// Blender reference: editors/sculpt_paint/paint_vertex.cc, paint_weight.cc,
// paint_image.cc, paint_ops.cc.
// ============================================================================

#include "paint/PaintEngine.hpp"
#include "sculpt/SculptEngine.hpp"

#include <cstdint>

namespace mechatron {

class EditableMesh;

class PaintToolbar {
public:
    PaintToolbar() = default;
    ~PaintToolbar() = default;

    // Brush picker — shows the brushes available for the current PaintMode.
    // Updates session.brush.type to the selected preset.
    static void render_brush_picker(PaintSession& session);

    // Brush settings (radius / strength / falloff / color / weight / blend /
    // invert / auto-normalize). Adapts which fields are shown based on
    // session.mode (color picker for Vertex/Texture, weight + group list for
    // Weight).
    static void render_brush_settings(PaintSession& session, EditableMesh& mesh);

    // Vertex group manager — list / add / remove / rename / select active.
    // Only meaningful in Weight Paint mode.
    static void render_vertex_groups(PaintSession& session, EditableMesh& mesh);

    // PAINT_OT_* one-shot operator buttons (set all, clear, invert, levels,
    // dirty vertex colors, normalize weights, blur all).
    static void render_operators(PaintSession& session, EditableMesh& mesh);

    // Convenience: render all four sub-panels in a vertical stack.
    static void render_full_panel(PaintSession& session, EditableMesh& mesh);
};

} // namespace mechatron
