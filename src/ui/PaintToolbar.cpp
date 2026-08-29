// ============================================================================
// PaintToolbar.cpp — Stage 5 paint-mode UI renderer.
// ============================================================================

#include "PaintToolbar.hpp"

#include "cad/EditableMesh.hpp"

#include <imgui.h>

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace mechatron {

namespace {

struct BrushRow {
    const char* id;
    const char* id_tr;
    PaintBrushType type;
    PaintMode mode;
};

// Blender's per-mode brush sets.
constexpr BrushRow kVertexBrushes[] = {
    {"Draw",    "Boyama",    PaintBrushType::Draw,    PaintMode::Vertex},
    {"Blur",    "Bulanık",   PaintBrushType::Blur,    PaintMode::Vertex},
    {"Average", "Ortala",    PaintBrushType::Average, PaintMode::Vertex},
    {"Smear",   "Sürükle",   PaintBrushType::Smear,   PaintMode::Vertex},
    {"Set",     "Ayarla",    PaintBrushType::Set,     PaintMode::Vertex},
};

constexpr BrushRow kWeightBrushes[] = {
    {"Add",      "Ekle",      PaintBrushType::Add,      PaintMode::Weight},
    {"Subtract", "Çıkar",     PaintBrushType::Subtract, PaintMode::Weight},
    {"Draw",     "Boyama",    PaintBrushType::Draw,     PaintMode::Weight},
    {"Blur",     "Bulanık",   PaintBrushType::Blur,     PaintMode::Weight},
    {"Average",  "Ortala",    PaintBrushType::Average,  PaintMode::Weight},
    {"Smear",    "Sürükle",   PaintBrushType::Smear,    PaintMode::Weight},
    {"Set",      "Ayarla",    PaintBrushType::Set,      PaintMode::Weight},
};

constexpr BrushRow kTextureBrushes[] = {
    {"Draw",  "Boyama",  PaintBrushType::Draw,  PaintMode::Texture},
    {"Blur",  "Bulanık", PaintBrushType::Blur,  PaintMode::Texture},
    {"Smear", "Sürükle", PaintBrushType::Smear, PaintMode::Texture},
};

const BrushRow* brushes_for(PaintMode mode, size_t& count) {
    switch (mode) {
        case PaintMode::Vertex:
            count = sizeof(kVertexBrushes) / sizeof(kVertexBrushes[0]);
            return kVertexBrushes;
        case PaintMode::Weight:
            count = sizeof(kWeightBrushes) / sizeof(kWeightBrushes[0]);
            return kWeightBrushes;
        case PaintMode::Texture:
            count = sizeof(kTextureBrushes) / sizeof(kTextureBrushes[0]);
            return kTextureBrushes;
    }
    count = 0;
    return nullptr;
}

const char* mode_label(PaintMode mode) {
    switch (mode) {
        case PaintMode::Vertex:  return "Vertex Paint (Köşe Boyama)";
        case PaintMode::Weight:  return "Weight Paint (Ağırlık Boyama)";
        case PaintMode::Texture: return "Texture Paint (Dokusu Boyama)";
    }
    return "?";
}

const char* falloff_label(BrushFalloff f) {
    switch (f) {
        case BrushFalloff::Smooth:        return "Smooth";
        case BrushFalloff::Sphere:        return "Sphere";
        case BrushFalloff::Root:          return "Root";
        case BrushFalloff::InverseSquare: return "Inverse Square";
        case BrushFalloff::Sharp:         return "Sharp";
        case BrushFalloff::Linear:        return "Linear";
        case BrushFalloff::Constant:      return "Constant";
        case BrushFalloff::Random:        return "Random";
    }
    return "?";
}

} // namespace

// ---------------------------------------------------------------------------
// Brush picker
// ---------------------------------------------------------------------------
void PaintToolbar::render_brush_picker(PaintSession& session) {
    if (!ImGui::CollapsingHeader("Brushes (Fırçalar)##paint_brush_picker",
                                 ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    ImGui::TextDisabled("%s", mode_label(session.mode));
    ImGui::Spacing();

    size_t count = 0;
    const BrushRow* rows = brushes_for(session.mode, count);
    for (size_t i = 0; i < count; ++i) {
        const BrushRow& r = rows[i];
        char label[128];
        snprintf(label, sizeof(label), "%s / %s", r.id, r.id_tr);
        const bool selected = (session.brush.type == r.type);
        if (ImGui::Selectable(label, selected)) {
            session.brush.type = r.type;
        }
    }
}

// ---------------------------------------------------------------------------
// Brush settings
// ---------------------------------------------------------------------------
void PaintToolbar::render_brush_settings(PaintSession& session, EditableMesh& mesh) {
    if (!ImGui::CollapsingHeader("Brush Settings##paint_brush_settings",
                                 ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }

    ImGui::SliderFloat("Radius (Yarıçap)", &session.brush.radius, 0.01f, 5.0f, "%.3f");
    ImGui::SliderFloat("Strength (Güç)",   &session.brush.strength, 0.0f, 1.0f, "%.2f");
    ImGui::Checkbox("Invert (Tersine) [Ctrl]", &session.brush.invert);

    int falloff_idx = static_cast<int>(session.brush.falloff);
    if (ImGui::Combo("Falloff (Düşüş)", &falloff_idx,
                     "Smooth\0Sphere\0Root\0Inverse Square\0Sharp\0Linear\0Constant\0Random\0")) {
        session.brush.falloff = static_cast<BrushFalloff>(falloff_idx);
    }
    ImGui::TextDisabled("Falloff: %s", falloff_label(session.brush.falloff));

    if (session.mode == PaintMode::Vertex || session.mode == PaintMode::Texture) {
        ImGui::Spacing();
        ImGui::TextDisabled("Color (Renk)");
        float col[4] = {session.brush.color.x, session.brush.color.y,
                        session.brush.color.z, session.brush.color.w};
        if (ImGui::ColorEdit4("Brush Color##paint_color", col)) {
            session.brush.color = Vec4{col[0], col[1], col[2], col[3]};
        }

        int blend_idx = static_cast<int>(session.brush.blend);
        if (ImGui::Combo("Blend (Karışım)", &blend_idx,
                         "Mix\0Add\0Subtract\0Multiply\0Lighten\0Darken\0Color\0")) {
            session.brush.blend = static_cast<PaintBlendMode>(blend_idx);
        }
        ImGui::TextDisabled("Blend: %s", PaintEngine::blend_mode_name(session.brush.blend));
    } else {
        ImGui::Spacing();
        ImGui::TextDisabled("Weight (Ağırlık)");
        ImGui::SliderFloat("Target Weight (Hedef Ağırlık)", &session.brush.weight, 0.0f, 1.0f, "%.2f");
        ImGui::Checkbox("Auto Normalize (Otomatik Normalleştir)", &session.brush.auto_normalize);

        // Show active group index + name.
        const size_t group_count = mesh.vertex_group_count_public();
        if (group_count == 0) {
            ImGui::TextDisabled("No vertex groups — create one below.");
        } else {
            int idx = static_cast<int>(session.brush.vertex_group_index);
            if (idx >= static_cast<int>(group_count)) idx = 0;
            if (ImGui::SliderInt("Active Group Index", &idx, 0, static_cast<int>(group_count) - 1)) {
                session.brush.vertex_group_index = static_cast<size_t>(idx);
            }
            std::string name = mesh.vertex_group_name_public(session.brush.vertex_group_index);
            ImGui::TextDisabled("Group: %s", name.c_str());
        }
    }
}

// ---------------------------------------------------------------------------
// Vertex group manager
// ---------------------------------------------------------------------------
void PaintToolbar::render_vertex_groups(PaintSession& session, EditableMesh& mesh) {
    if (session.mode != PaintMode::Weight) return;
    if (!ImGui::CollapsingHeader("Vertex Groups (Köşe Grupları)##paint_groups")) {
        return;
    }

    static char new_name[64] = "Group";
    ImGui::InputText("Name##new_group_name", new_name, sizeof(new_name));
    ImGui::SameLine();
    if (ImGui::Button("+ Add (Ekle)")) {
        size_t idx = mesh.add_vertex_group_public(new_name);
        session.brush.vertex_group_index = idx;
    }

    ImGui::Spacing();
    const size_t count = mesh.vertex_group_count_public();
    for (size_t i = 0; i < count; ++i) {
        std::string name = mesh.vertex_group_name_public(i);
        char label[160];
        snprintf(label, sizeof(label), "[%zu] %s", i, name.c_str());
        const bool selected = (session.brush.vertex_group_index == i);
        if (ImGui::Selectable(label, selected)) {
            session.brush.vertex_group_index = i;
        }
    }
}

// ---------------------------------------------------------------------------
// PAINT_OT_* one-shot operators
// ---------------------------------------------------------------------------
void PaintToolbar::render_operators(PaintSession& session, EditableMesh& mesh) {
    if (!ImGui::CollapsingHeader("Operators (İşlemler)##paint_operators",
                                 ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }

    if (session.mode == PaintMode::Vertex || session.mode == PaintMode::Texture) {
        ImGui::TextDisabled("Vertex Color ops");
        if (ImGui::Button("Set All (Tümünü Boya)")) {
            PaintEngine::set_all_vertex_colors(mesh, session.brush.color);
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear (Temizle)")) {
            PaintEngine::clear_vertex_colors(mesh);
        }
        ImGui::SameLine();
        if (ImGui::Button("Invert (Tersine)")) {
            PaintEngine::invert_vertex_colors(mesh);
        }

        if (ImGui::Button("Blur All (Tümünü Bulanıklaştır)")) {
            PaintEngine::blur_all_vertex_colors(mesh);
        }
        ImGui::SameLine();
        if (ImGui::Button("Dirty (Pis)")) {
            PaintEngine::dirty_vertex_colors(mesh, 1.0f);
        }

        static float gain = 1.0f, bias = 0.0f;
        ImGui::SliderFloat("Gain (Kazanç)", &gain, 0.0f, 4.0f);
        ImGui::SliderFloat("Bias (Öteleme)", &bias, -1.0f, 1.0f);
        if (ImGui::Button("Levels (Seviyeler)")) {
            PaintEngine::levels_vertex_colors(mesh, gain, bias);
        }
    }

    if (session.mode == PaintMode::Weight) {
        ImGui::TextDisabled("Weight ops");
        if (ImGui::Button("Set All Weights")) {
            PaintEngine::set_all_weights(mesh, session.brush.vertex_group_index, session.brush.weight);
        }
        ImGui::SameLine();
        if (ImGui::Button("Normalize All")) {
            PaintEngine::normalize_all_weights(mesh);
        }
    }
}

// ---------------------------------------------------------------------------
// Full panel
// ---------------------------------------------------------------------------
void PaintToolbar::render_full_panel(PaintSession& session, EditableMesh& mesh) {
    render_brush_picker(session);
    ImGui::Spacing();
    render_brush_settings(session, mesh);
    ImGui::Spacing();
    render_vertex_groups(session, mesh);
    ImGui::Spacing();
    render_operators(session, mesh);
}

} // namespace mechatron
