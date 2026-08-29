// ============================================================================
// SculptToolbar.cpp — Stage 4 sculpt-mode UI renderer.
// ============================================================================

#include "SculptToolbar.hpp"

#include "cad/EditableMesh.hpp"

#include <imgui.h>

#include <cmath>

namespace mechatron {

namespace {

// ---------------------------------------------------------------------------
// Brush preset table — 29 Blender brushes mapped onto the 7 algorithms.
// ---------------------------------------------------------------------------
// clang-format off
constexpr BrushPreset kPresets[] = {
    // Draw family
    {"Draw",                    "Çizim",                  BrushType::Draw,    0.50f, 0.50f, BrushFalloff::Smooth, false},
    {"Draw Sharp",              "Keskin Çizim",           BrushType::Draw,    0.40f, 0.60f, BrushFalloff::Sharp,  false},
    {"Clay",                    "Kil",                    BrushType::Draw,    0.65f, 0.40f, BrushFalloff::Smooth, false},
    {"Clay Strips",             "Kil Şeritleri",          BrushType::Draw,    0.70f, 0.55f, BrushFalloff::Sphere, false},
    {"Clay Thumb",              "Kil Başparmak",          BrushType::Draw,    0.60f, 0.45f, BrushFalloff::Root,   false},
    {"Layer",                   "Katman",                 BrushType::Draw,    0.50f, 0.40f, BrushFalloff::Constant, false},

    // Grab family
    {"Grab",                    "Yakala",                 BrushType::Grab,    0.50f, 1.00f, BrushFalloff::Smooth, false},
    {"Grab Silhouette",         "Siluet Yakala",          BrushType::Grab,    0.75f, 0.80f, BrushFalloff::Smooth, false},
    {"Snake Hook",              "Yılan Kanca",            BrushType::Grab,    0.45f, 0.85f, BrushFalloff::Sphere, false},
    {"Thumb",                   "Başparmak",              BrushType::Grab,    0.50f, 0.70f, BrushFalloff::Linear, false},
    {"Pose",                    "Poz",                    BrushType::Grab,    0.80f, 0.60f, BrushFalloff::Constant, false},

    // Smooth family
    {"Smooth",                  "Pürüzsüzleştir",         BrushType::Smooth,  0.50f, 0.50f, BrushFalloff::Smooth, false},
    {"Surface Smooth",          "Yüzey Pürüzsüz",         BrushType::Smooth,  0.45f, 0.40f, BrushFalloff::Sphere, false},
    {"Enhance Details",         "Detayları Vurgula",      BrushType::Smooth,  0.45f, 0.30f, BrushFalloff::Sharp,  true},
    {"Relax",                   "Gevşet",                 BrushType::Smooth,  0.50f, 0.40f, BrushFalloff::Linear, false},
    {"Relax Topology",          "Topoloji Gevşet",        BrushType::Smooth,  0.50f, 0.40f, BrushFalloff::Smooth, false},
    {"BMesh Topology Rake",     "BMesh Topoloji Tırmık",  BrushType::Smooth,  0.40f, 0.35f, BrushFalloff::Root,   false},
    {"Topology Slide",          "Topoloji Kaydır",        BrushType::Smooth,  0.50f, 0.30f, BrushFalloff::Linear, false},

    // Pinch family
    {"Pinch",                   "Sıkıştır",               BrushType::Pinch,   0.50f, 0.50f, BrushFalloff::Smooth, false},
    {"Crease",                  "Kırışık",                BrushType::Pinch,   0.40f, 0.70f, BrushFalloff::Sharp,  false},

    // Inflate family
    {"Inflate",                 "Şişir",                  BrushType::Inflate, 0.50f, 0.50f, BrushFalloff::Smooth, false},
    {"Rotate",                  "Döndür",                 BrushType::Inflate, 0.50f, 0.50f, BrushFalloff::Linear, false},

    // Flatten family
    {"Flatten",                 "Düzleştir",              BrushType::Flatten, 0.50f, 0.50f, BrushFalloff::Smooth, false},
    {"Fill",                    "Doldur",                 BrushType::Flatten, 0.50f, 0.50f, BrushFalloff::Smooth, true},
    {"Scrape",                  "Kazı",                   BrushType::Flatten, 0.50f, 0.50f, BrushFalloff::Smooth, false},
    {"Multiplane Scrape",       "Çok Düzlemli Kazı",      BrushType::Flatten, 0.55f, 0.50f, BrushFalloff::Smooth, false},

    // Mask family
    {"Mask",                    "Maske",                  BrushType::Mask,    0.50f, 1.00f, BrushFalloff::Smooth, false},
    {"Draw Face Sets",          "Yüzey Kümeleri Çiz",     BrushType::Mask,    0.50f, 1.00f, BrushFalloff::Smooth, false},
    {"Draw Vector Displacement","Vektör Yer Değiştirme",  BrushType::Draw,    0.50f, 0.50f, BrushFalloff::Constant, false},
};
// clang-format on

constexpr int kPresetCount = static_cast<int>(sizeof(kPresets) / sizeof(kPresets[0]));

const char* falloff_label(BrushFalloff f) {
    switch (f) {
        case BrushFalloff::Smooth:       return "Smooth (Düzgün)";
        case BrushFalloff::Sphere:       return "Sphere (Küre)";
        case BrushFalloff::Root:         return "Root (Kök)";
        case BrushFalloff::InverseSquare:return "Inverse Square (Ters Kare)";
        case BrushFalloff::Sharp:        return "Sharp (Keskin)";
        case BrushFalloff::Linear:       return "Linear (Doğrusal)";
        case BrushFalloff::Constant:     return "Constant (Sabit)";
        case BrushFalloff::Random:       return "Random (Rastgele)";
    }
    return "?";
}

} // namespace

const BrushPreset* sculpt_brush_presets() { return kPresets; }
size_t sculpt_brush_preset_count() { return static_cast<size_t>(kPresetCount); }

// ---------------------------------------------------------------------------
// Brush picker
// ---------------------------------------------------------------------------
void SculptToolbar::render_brush_picker(SculptSession& session) {
    if (!ImGui::CollapsingHeader("Brushes (Fırçalar)##sculpt_brush_picker",
                                 ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    ImGui::TextDisabled("29 brushes / 7 algorithms");
    ImGui::Spacing();

    // Group presets by algorithm so the picker is browsable.
    for (int algo = 0; algo < static_cast<int>(BrushType::Count); ++algo) {
        const BrushType type = static_cast<BrushType>(algo);
        const char* group_name = SculptEngine::brush_name_en(type);
        if (ImGui::TreeNode(group_name)) {
            for (int i = 0; i < kPresetCount; ++i) {
                const BrushPreset& p = kPresets[i];
                if (p.algorithm != type) continue;
                char label[128];
                snprintf(label, sizeof(label), "%s / %s", p.id, p.id_tr);
                const bool selected = (session.brush.type == p.algorithm);
                if (ImGui::Selectable(label, selected)) {
                    session.brush.type    = p.algorithm;
                    session.brush.radius  = p.radius;
                    session.brush.strength = p.strength;
                    session.brush.falloff = p.falloff;
                    session.brush.invert  = p.invert;
                }
            }
            ImGui::TreePop();
        }
    }
}

// ---------------------------------------------------------------------------
// Brush settings (radius / strength / falloff / auto-smooth / invert)
// ---------------------------------------------------------------------------
void SculptToolbar::render_brush_settings(SculptSession& session) {
    if (!ImGui::CollapsingHeader("Brush Settings##sculpt_brush_settings",
                                 ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    ImGui::SliderFloat("Radius (Yarıçap)", &session.brush.radius, 0.01f, 5.0f, "%.3f");
    ImGui::SliderFloat("Strength (Güç)",  &session.brush.strength, 0.0f, 1.0f, "%.2f");
    ImGui::SliderFloat("Auto-Smooth (Otomatik Pürüzsüzleştirme)",
                       &session.brush.auto_smooth, 0.0f, 1.0f, "%.2f");
    ImGui::Checkbox("Invert (Tersine) [Ctrl]", &session.brush.invert);

    int falloff_idx = static_cast<int>(session.brush.falloff);
    if (ImGui::Combo("Falloff (Düşüş)", &falloff_idx,
                     "Smooth\0Sphere\0Root\0Inverse Square\0Sharp\0Linear\0Constant\0Random\0")) {
        session.brush.falloff = static_cast<BrushFalloff>(falloff_idx);
    }
    ImGui::TextDisabled("Falloff: %s", falloff_label(session.brush.falloff));
}

// ---------------------------------------------------------------------------
// Symmetry + automasking
// ---------------------------------------------------------------------------
void SculptToolbar::render_symmetry_panel(SculptSession& session) {
    if (!ImGui::CollapsingHeader("Symmetry (Simetri)##sculpt_symmetry")) {
        return;
    }
    bool x = (session.symmetry_mask & static_cast<int>(SculptSymmetry::X)) != 0;
    bool y = (session.symmetry_mask & static_cast<int>(SculptSymmetry::Y)) != 0;
    bool z = (session.symmetry_mask & static_cast<int>(SculptSymmetry::Z)) != 0;
    if (ImGui::Checkbox("X", &x)) session.symmetry_mask ^= static_cast<int>(SculptSymmetry::X);
    ImGui::SameLine();
    if (ImGui::Checkbox("Y", &y)) session.symmetry_mask ^= static_cast<int>(SculptSymmetry::Y);
    ImGui::SameLine();
    if (ImGui::Checkbox("Z", &z)) session.symmetry_mask ^= static_cast<int>(SculptSymmetry::Z);

    ImGui::DragFloat3("Symmetry origin (Simetri merkezi)", session.symmetry_origin, 0.01f);
}

// ---------------------------------------------------------------------------
// One-shot SCULPT_OT_* operators
// ---------------------------------------------------------------------------
void SculptToolbar::render_operators(SculptSession& session, EditableMesh& mesh) {
    (void)session;
    if (!ImGui::CollapsingHeader("Operators (İşlemler)##sculpt_operators",
                                 ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }

    if (ImGui::Button("Symmetrize X (X'e Göre Simetrik Yap)")) {
        SculptEngine::symmetrize(mesh, 0);
    }
    ImGui::SameLine();
    if (ImGui::Button("Symmetrize Y")) {
        SculptEngine::symmetrize(mesh, 1);
    }
    ImGui::SameLine();
    if (ImGui::Button("Symmetrize Z")) {
        SculptEngine::symmetrize(mesh, 2);
    }

    ImGui::Spacing();
    ImGui::TextDisabled("Mesh Filter");
    if (ImGui::Button("Smooth Mesh##mesh_filter_smooth")) {
        SculptEngine::mesh_filter(mesh, SculptEngine::MeshFilterMode::Smooth, 5, 0.5f);
    }
    ImGui::SameLine();
    if (ImGui::Button("Sharpen Mesh")) {
        SculptEngine::mesh_filter(mesh, SculptEngine::MeshFilterMode::Sharpen, 3, 0.5f);
    }
    ImGui::SameLine();
    if (ImGui::Button("Relax Mesh")) {
        SculptEngine::mesh_filter(mesh, SculptEngine::MeshFilterMode::Relax, 5, 0.5f);
    }

    ImGui::Spacing();
    ImGui::TextDisabled("Mask Filter");
    if (ImGui::Button("Clear Mask (Maskeyi Temizle)")) {
        SculptEngine::mask_filter(mesh, SculptEngine::MaskFilterMode::Clear);
    }
    ImGui::SameLine();
    if (ImGui::Button("Invert Mask (Maskeyi Tersine Çevir)")) {
        SculptEngine::mask_filter(mesh, SculptEngine::MaskFilterMode::Invert);
    }
    ImGui::SameLine();
    if (ImGui::Button("Fill Mask (Maskeyi Doldur)")) {
        SculptEngine::mask_filter(mesh, SculptEngine::MaskFilterMode::Fill);
    }

    if (ImGui::Button("Smooth Mask##mask_filter_smooth")) {
        SculptEngine::mask_filter(mesh, SculptEngine::MaskFilterMode::Smooth);
    }
    ImGui::SameLine();
    if (ImGui::Button("Grow Mask")) {
        SculptEngine::mask_filter(mesh, SculptEngine::MaskFilterMode::Grow);
    }
    ImGui::SameLine();
    if (ImGui::Button("Shrink Mask")) {
        SculptEngine::mask_filter(mesh, SculptEngine::MaskFilterMode::Shrink);
    }
}

// ---------------------------------------------------------------------------
// Full panel
// ---------------------------------------------------------------------------
void SculptToolbar::render_full_panel(SculptSession& session, EditableMesh& mesh) {
    render_brush_picker(session);
    ImGui::Spacing();
    render_brush_settings(session);
    ImGui::Spacing();
    render_symmetry_panel(session);
    ImGui::Spacing();
    render_operators(session, mesh);
}

} // namespace mechatron
