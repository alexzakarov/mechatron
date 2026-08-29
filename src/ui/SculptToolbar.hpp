#pragma once

// ============================================================================
// SculptToolbar — Stage 4 sculpt-mode UI renderer.
//
// Stateless ImGui panel that exposes the 7 first-class brush algorithms
// (Draw/Grab/Smooth/Pinch/Inflate/Flatten/Mask) plus their parameters
// (radius/strength/falloff/auto-smooth/invert), symmetry toggles, and the
// SCULPT_OT_* one-shot operators (symmetrize, mesh filter, mask filter).
//
// The toolbar does NOT own any state — it edits the SculptSession and the
// EditableMesh it is handed. The host (ModelEditor) owns the session and
// triggers begin_stroke/stroke_to/end_stroke in response to viewport mouse
// events. The 29-brush Blender picker is exposed as BrushPreset rows that
// preset the BrushType + parameters (e.g. Clay = Draw with plane projection,
// Scrape = Flatten negative, Crease = Pinch + Draw).
//
// Blender reference: editors/sculpt_paint/sculpt_ops.cc + space_view3d/
// toolbar. The BrushPreset table lives in SculptToolbar.cpp.
// ============================================================================

#include "sculpt/SculptEngine.hpp"

#include <cstdint>
#include <string>

namespace mechatron {

class EditableMesh;

// Brush presets — the 29 Blender brushes map onto the 7 first-class
// algorithms via these presets (see SculptEngine.hpp's brush map table).
struct BrushPreset {
    const char* id;             // Blender operator name e.g. "Draw Sharp"
    const char* id_tr;          // Turkish label e.g. "Keskin Çizim"
    BrushType   algorithm;      // One of 7 first-class algorithms
    float       radius;         // Suggested starting radius
    float       strength;       // Suggested starting strength
    BrushFalloff falloff;
    bool        invert;         // Ctrl-direction by default (e.g. Scrape = Flatten negative)
};

// All 29 Blender sculpt brushes, in the order they appear in Blender's
// brush picker.
const BrushPreset* sculpt_brush_presets();
size_t sculpt_brush_preset_count();

class SculptToolbar {
public:
    SculptToolbar() = default;
    ~SculptToolbar() = default;

    // Render the right-hand brush settings panel. Mutates session.
    static void render_brush_settings(SculptSession& session);

    // Render the symmetry + automasking panel.
    static void render_symmetry_panel(SculptSession& session);

    // Render the brush picker grid (29 brushes, 7 algorithm groups).
    // Updates session.brush.type + parameters to the selected preset.
    static void render_brush_picker(SculptSession& session);

    // Render the SCULPT_OT_* one-shot operator buttons (symmetrize, mesh
    // filter, mask filter, mask clear, mask invert). These wrap SculptEngine
    // operators that act immediately on the mesh.
    static void render_operators(SculptSession& session,
                                 EditableMesh& mesh);

    // Convenience: render all four sub-panels in a vertical stack inside one
    // ImGui child window. Used by ModelEditor for the right-hand sculpt tab.
    static void render_full_panel(SculptSession& session,
                                  EditableMesh& mesh);
};

} // namespace mechatron
