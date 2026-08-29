// ============================================================================
// PieMenus — Generic pie menu rendering. Activation logic (what each slice
// does) lives in ModelEditor, which reads the selected index back from
// PieMenu::render() and dispatches via its private methods.
// ============================================================================

#include "PieMenus.hpp"

#include "imgui.h"

#include <cmath>

namespace mechatron {

// ----- PieMenu -----

void PieMenu::open(const float pos[2]) {
    m_open = true;
    m_open_pos[0] = pos[0];
    m_open_pos[1] = pos[1];
}

void PieMenu::cancel() {
    m_open = false;
}

// Direction layout (Blender convention — 8 directions max):
//   index 0: top         (12 o'clock)  shortcut 1
//   index 1: top-right   (1-2)         shortcut 2
//   index 2: right       (3)           shortcut 3
//   index 3: bottom-right(4-5)         shortcut 4
//   index 4: bottom      (6)           shortcut 5
//   index 5: bottom-left (7-8)         shortcut 6
//   index 6: left        (9)           shortcut 7
//   index 7: top-left    (10-11)       shortcut 8
static float slice_angle(size_t index, size_t count) {
    // Angle 0 = up (+Y in screen-down space means -PI/2 from +X).
    // We distribute evenly starting from top, going clockwise.
    const float step = (2.0f * 3.14159265358979323846f) / static_cast<float>(count);
    const float base = -3.14159265358979323846f / 2.0f; // pointing up
    return base + step * static_cast<float>(index);
}

int PieMenu::render() {
    if (!m_open) return kNoSelection;

    ImGuiIO& io = ImGui::GetIO();

    // Clamp count.
    const size_t n = m_count > kMaxSlices ? kMaxSlices : m_count;
    if (n == 0) {
        m_open = false;
        return kNoSelection;
    }

    // Pie geometry. Outer/inner radius in screen pixels.
    const float outer_r = 90.0f;
    const float inner_r = 16.0f;
    const ImVec2 center(m_open_pos[0], m_open_pos[1]);

    // Compute mouse angle/distance relative to pie center.
    const float dx = io.MousePos.x - center.x;
    const float dy = io.MousePos.y - center.y;
    const float dist = std::sqrt(dx * dx + dy * dy);
    const float mouse_angle = std::atan2(dy, dx);

    // Determine highlighted slice from angle.
    int hovered = -1;
    if (dist >= inner_r * 0.5f) {
        int best = -1;
        float best_delta = 1e9f;
        for (size_t i = 0; i < n; ++i) {
            const float a = slice_angle(i, n);
            float delta = std::fabs(a - mouse_angle);
            if (delta > 3.14159265358979323846f) delta = 2.0f * 3.14159265358979323846f - delta;
            if (delta < best_delta) {
                best_delta = delta;
                best = static_cast<int>(i);
            }
        }
        if (best >= 0 && dist >= inner_r) hovered = best;
    }

    // Numeric shortcut activation (takes precedence over hover if pressed).
    int shortcut_hit = -1;
    for (size_t i = 0; i < n; ++i) {
        const int sc = m_items[i].shortcut;
        if (sc >= 1 && sc <= 8) {
            const ImGuiKey key = static_cast<ImGuiKey>(static_cast<int>(ImGuiKey_1) + (sc - 1));
            if (ImGui::IsKeyPressed(key)) {
                shortcut_hit = static_cast<int>(i);
                break;
            }
        }
    }

    // ----- Draw -----
    ImDrawList* dl = ImGui::GetForegroundDrawList();

    // Background wedge from center.
    auto slice_color = [](int idx, bool enabled, bool hl) {
        if (!enabled) return Theme::U32(Theme::WithAlpha(Theme::CurrentPalette().surfaceActive, 0.78f));
        if (hl) return Theme::U32(Theme::WithAlpha(Theme::CurrentPalette().primary, 0.90f));
        return Theme::U32(Theme::WithAlpha(Theme::CurrentPalette().surface, 0.86f));
    };

    const ImU32 edge_col = Theme::U32(Theme::CurrentPalette().borderStrong);
    const ImU32 center_col = Theme::U32(Theme::WithAlpha(Theme::CurrentPalette().bg, 0.94f));
    const ImU32 text_col = Theme::U32(Theme::CurrentPalette().text);
    const ImU32 text_disabled_col = Theme::U32(Theme::CurrentPalette().textDisabled);

    for (size_t i = 0; i < n; ++i) {
        const float a0 = slice_angle(i, n);
        const float step = (2.0f * 3.14159265358979323846f) / static_cast<float>(n);
        const float half = step * 0.5f - 0.04f;
        const bool hl = (hovered == static_cast<int>(i));
        const bool enabled = m_items[i].enabled;
        // Draw a wedge from inner to outer radius.
        const int segments = 12;
        // Use PathArcTo on outer + inner to build a wedge ring sector.
        // Outer arc.
        dl->PathArcTo(center, outer_r, a0 - half, a0 + half, segments);
        // Inner arc reversed to form ring sector.
        dl->PathArcTo(center, inner_r, a0 + half, a0 - half, segments);
        dl->PathFillConvex(slice_color(static_cast<int>(i), enabled, hl));
        dl->PathArcTo(center, outer_r, a0 - half, a0 + half, segments);
        dl->PathStroke(edge_col, 0, 1.5f);

        // Label position.
        const float label_r = (outer_r + inner_r) * 0.5f;
        const float la = slice_angle(i, n);
        const ImVec2 lp(center.x + std::cos(la) * label_r, center.y + std::sin(la) * label_r);
        const char* txt = m_items[i].label ? m_items[i].label : "";
        const ImVec2 ts = ImGui::CalcTextSize(txt);
        dl->AddText(ImVec2(lp.x - ts.x * 0.5f, lp.y - ts.y * 0.5f),
                    enabled ? text_col : text_disabled_col, txt);
    }

    // Center disc + tooltip.
    dl->AddCircleFilled(center, inner_r, center_col, 24);
    dl->AddCircle(center, inner_r, edge_col, 24, 1.5f);

    // Tooltip at center: hovered slice's tooltip, or "Select...".
    const char* tip = "Select...";
    if (hovered >= 0 && m_items[hovered].tooltip && *m_items[hovered].tooltip) {
        tip = m_items[hovered].tooltip;
    }
    {
        const ImVec2 ts = ImGui::CalcTextSize(tip);
        ImVec2 tp(center.x - ts.x * 0.5f, center.y - ts.y * 0.5f);
        // Clamp inside pie center.
        dl->AddText(tp, text_col, tip);
    }

    // ----- Activation -----
    int activated = -1;
    if (shortcut_hit >= 0 && m_items[shortcut_hit].enabled) {
        activated = shortcut_hit;
    } else if (hovered >= 0 && m_items[hovered].enabled) {
        // Release to confirm.
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            activated = hovered;
        }
    }

    // Cancel conditions: Escape, right-click, or click far outside.
    if (ImGui::IsKeyPressed(ImGuiKey_Escape) || ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        m_open = false;
        return kNoSelection;
    }

    if (activated >= 0) {
        m_open = false;
        return activated;
    }

    // While open, swallow input that would otherwise reach the viewport.
    return kNoSelection;
}

// ----- PieMenuSet -----

bool PieMenuSet::render() {
    bool any_open_this_frame = false;
    if (shading.is_open())   { shading.render();   any_open_this_frame = true; }
    if (mode.is_open())      { mode.render();      any_open_this_frame = true; }
    if (view.is_open())      { view.render();      any_open_this_frame = true; }
    if (snap.is_open())      { snap.render();      any_open_this_frame = true; }
    if (apply.is_open())     { apply.render();     any_open_this_frame = true; }
    if (edge.is_open())      { edge.render();      any_open_this_frame = true; }
    if (face.is_open())      { face.render();      any_open_this_frame = true; }
    if (vertex.is_open())    { vertex.render();    any_open_this_frame = true; }
    if (merge.is_open())     { merge.render();     any_open_this_frame = true; }
    if (delete_pie.is_open()){ delete_pie.render();any_open_this_frame = true; }
    if (normals.is_open())   { normals.render();   any_open_this_frame = true; }
    if (specials.is_open())  { specials.render();  any_open_this_frame = true; }
    return any_open_this_frame;
}

} // namespace mechatron
