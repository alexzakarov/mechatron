#pragma once

// ============================================================================
// PieMenus — Blender-style pie menus for the ModelEditor.
//
// Three core pies are provided (matching Blender 4.x/5.x behavior):
//   - Shading pie  (Z)       : Wireframe / Solid / Material Preview / Rendered / X-Ray toggle
//   - Mode pie     (Ctrl+Tab): Object / Edit / Sculpt / Texture Paint / Vertex Paint / Weight Paint
//   - View pie     (~)       : View All / View Selected / Camera / Local View / Align Top/Front/Right
//
// A pie opens at the cursor. Direction is chosen by mouse angle at release
// time; each direction can also be activated with its numeric shortcut
// while the pie is open.
//
// PieMenu is a generic container. ModelEditor populates it (via its private
// populate_pie_* methods) and reads back the activated slice index.
// ============================================================================

#include <array>
#include <cstddef>

namespace mechatron {

class ModelEditor;

// A single slice of a pie menu. ModelEditor populates this each frame the
// pie is open.
struct PieSlice {
    const char* label = "";        // Short label shown radially
    const char* tooltip = "";      // Center tooltip (bilingual)
    int shortcut = 0;              // 1..8 numeric shortcut while pie is open
    bool enabled = true;           // false = greyed out
    int user_data = 0;             // Opaque slot for the editor (e.g. enum value)
};

// Generic pie menu. The owner opens it, populates items, ticks render(),
// and reads back the activated index (kNoSelection if cancelled).
class PieMenu {
public:
    static constexpr size_t kMaxSlices = 8;
    static constexpr int kNoSelection = -1;

    void open(const float pos[2]);
    void cancel();
    // Returns the activated slice index this frame, or kNoSelection.
    int render();

    bool is_open() const { return m_open; }
    const float* open_pos() const { return m_open_pos; }

    std::array<PieSlice, kMaxSlices>& items() { return m_items; }
    void set_count(size_t n) { m_count = n > kMaxSlices ? kMaxSlices : n; }
    size_t count() const { return m_count; }

private:
    friend class ModelEditor;
    bool m_open = false;
    float m_open_pos[2] = {0, 0};
    std::array<PieSlice, kMaxSlices> m_items{};
    size_t m_count = 0;
};

// Set of pies owned by ModelEditor. ModelEditor calls render() on the set
// each frame; it returns true if any pie consumed input.
struct PieMenuSet {
    PieMenu shading;   // Z
    PieMenu mode;      // Ctrl+Tab
    PieMenu view;      // ~
    PieMenu snap;      // Shift+S
    PieMenu apply;     // Ctrl+A
    PieMenu edge;      // Ctrl+E
    PieMenu face;      // Ctrl+F
    PieMenu vertex;    // Ctrl+V
    PieMenu merge;     // M
    PieMenu delete_pie;// X (in edit mode, with no drag)
    PieMenu normals;   // Alt+N
    PieMenu specials;  // W

    // Tick all open pies; returns true if any pie was open this frame.
    bool render();
    // True if any pie is currently open (blocks other input).
    bool any_open() const {
        return shading.is_open() || mode.is_open() || view.is_open() || snap.is_open() ||
               apply.is_open() || edge.is_open() || face.is_open() || vertex.is_open() ||
               merge.is_open() || delete_pie.is_open() || normals.is_open() || specials.is_open();
    }
    void cancel_all() {
        shading.cancel(); mode.cancel(); view.cancel(); snap.cancel(); apply.cancel();
        edge.cancel(); face.cancel(); vertex.cancel(); merge.cancel(); delete_pie.cancel();
        normals.cancel(); specials.cancel();
    }
};

} // namespace mechatron
