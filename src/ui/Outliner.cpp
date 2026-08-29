// ============================================================================
// Outliner.cpp — implementation of the scene hierarchy tree.
// See Outliner.hpp for design notes.
// ============================================================================

#include "Outliner.hpp"
#include "UIApplication.hpp"      // for context: SimulationOrchestrator
#include "PropertiesPanel.hpp"
#include "ModelEditor.hpp"
#include "core/SimulationOrchestrator.hpp"
#include "core/Registry.hpp"
#include "core/Subsystem.hpp"
#include "core/Component.hpp"

#include <imgui.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace mechatron {

namespace {

// Persistent UI state keyed by component id. Outliner is the single owner so
// lifetime is straightforward: hide/set stays until the user toggles it back.
// Note: these maps live as function-local statics deliberately — the Outliner
// is a stateless renderer and this avoids cross-session bleed when the
// registry is cleared/repopulated.

std::unordered_set<std::string>& hidden_components() {
    static std::unordered_set<std::string> s;
    return s;
}

std::unordered_set<std::string>& hidden_collections() {
    static std::unordered_set<std::string> s;
    return s;
}

std::unordered_set<std::string>& no_render_components() {
    static std::unordered_set<std::string> s;
    return s;
}

std::unordered_map<std::string, bool>& expand_state() {
    static std::unordered_map<std::string, bool> s;
    return s;
}

bool is_component_hidden(const std::string& id) {
    return hidden_components().count(id) > 0;
}
bool is_collection_hidden(const std::string& id) {
    return hidden_collections().count(id) > 0;
}
bool is_component_no_render(const std::string& id) {
    return no_render_components().count(id) > 0;
}

// Build the set of all component ids that belong to a subsystem.
std::unordered_set<std::string> subsystem_member_ids(const Subsystem& sub) {
    std::unordered_set<std::string> ids;
    for (const Subsystem::Member& m : sub.members()) ids.insert(m.component_id);
    return ids;
}

// Look up a friendly display name for a component. Prefer component_type; fall
// back to id if empty.
std::string component_display_name(const Component& comp) {
    std::string_view t = comp.component_type();
    if (!t.empty()) return std::string(t);
    return comp.id();
}

} // namespace

// ---------------------------------------------------------------------------
// Expand-state helpers
// ---------------------------------------------------------------------------
bool Outliner::is_expanded(const std::string& key) const {
    auto it = expand_state().find(key);
    return it != expand_state().end() ? it->second : true;  // default open
}

void Outliner::set_expanded(const std::string& key, bool open) {
    expand_state()[key] = open;
}

void Outliner::expand_all() {
    for (auto& [k, v] : expand_state()) v = true;
    // Force scene + collection roots open even if never visited.
    expand_state()["scene"] = true;
    expand_state()["collections"] = true;
    expand_state()["loose"] = true;
}

void Outliner::collapse_all() {
    for (auto& [k, v] : expand_state()) v = false;
}

void Outliner::show_active() {
    // Expand the scene root so we can see something.
    expand_state()["scene"] = true;
    expand_state()["collections"] = true;
    expand_state()["loose"] = true;
    // Per-component expansion is irrelevant for our 2-level tree; setting the
    // roots is enough to guarantee the active row is reachable.
}

// ---------------------------------------------------------------------------
// Main render entry
// ---------------------------------------------------------------------------
void Outliner::render(SimulationOrchestrator& orchestrator,
                      PropertiesPanel& properties,
                      ModelEditor* model_editor) {
    m_has_content = false;

    Registry& registry = orchestrator.registry();

    // Build the per-frame view: which components are "loose" (not in any
    // subsystem) vs. contained.
    std::unordered_set<std::string> contained_ids;
    const std::vector<std::string> subsystem_ids = orchestrator.list_subsystem_ids();
    for (const std::string& sid : subsystem_ids) {
        if (Subsystem* sub = orchestrator.get_subsystem(sid)) {
            for (const Subsystem::Member& m : sub->members()) {
                contained_ids.insert(m.component_id);
            }
        }
    }

    // ---- Toolbar: search + collapse/expand ----
    char search_buf[128] = {};
    ImGui::SetNextItemWidth(-1);
    ImGui::InputTextWithHint("##outliner_search", "Search (Ara)...", search_buf, sizeof(search_buf));
    ImGui::Spacing();

    if (ImGui::Button("Expand All")) expand_all();
    ImGui::SameLine();
    if (ImGui::Button("Collapse")) collapse_all();
    ImGui::SameLine();
    if (ImGui::Button("Show Active")) show_active();
    ImGui::Separator();

    // ---- Tree ----
    // Root: Scene
    ImGui::PushID("OutlinerRoot");
    bool scene_open = ImGui::TreeNode("SceneRoot", "Scene (Sahne)");
    if (ImGui::BeginPopupContextItem("SceneCtx")) {
        render_context_menu_scene(orchestrator);
        ImGui::EndPopup();
    }
    if (scene_open) {
        m_has_content = true;

        // Collections
        bool collections_open = is_expanded("collections");
        if (ImGui::TreeNodeEx("Collections",
                              (collections_open ? ImGuiTreeNodeFlags_DefaultOpen : 0),
                              "Collections (%zu)", subsystem_ids.size())) {
            set_expanded("collections", true);
            if (!subsystem_ids.empty()) {
                for (const std::string& sid : subsystem_ids) {
                    Subsystem* sub = orchestrator.get_subsystem(sid);
                    if (!sub) continue;
                    render_collection_row(sid,
                                          sub->display_name().empty() ? sid : sub->display_name(),
                                          orchestrator, properties, model_editor);
                }
            } else {
                ImGui::TextDisabled("(no collections)");
            }
            ImGui::TreePop();
        } else {
            set_expanded("collections", false);
        }

        // Loose components
        std::vector<std::string> loose_ids;
        for (auto& [id, comp] : registry.all_components()) {
            if (contained_ids.count(id) == 0) {
                loose_ids.push_back(id);
            }
        }
        // Sort alphabetically for deterministic order.
        std::sort(loose_ids.begin(), loose_ids.end());

        bool loose_open = is_expanded("loose");
        if (ImGui::TreeNodeEx("Loose",
                              (loose_open ? ImGuiTreeNodeFlags_DefaultOpen : 0),
                              "Loose Components (%zu)", loose_ids.size())) {
            set_expanded("loose", true);
            if (!loose_ids.empty()) {
                for (const std::string& id : loose_ids) {
                    Component* comp = registry.get(id);
                    if (!comp) continue;
                    render_component_row(*comp, properties, model_editor,
                                         orchestrator, std::string(), std::string(), true);
                }
            } else {
                ImGui::TextDisabled("(no loose components)");
            }
            ImGui::TreePop();
        } else {
            set_expanded("loose", false);
        }

        ImGui::TreePop();
    }
    ImGui::PopID();

    // Optional: empty-state hint
    if (registry.size() == 0 && subsystem_ids.empty()) {
        ImGui::Spacing();
        ImGui::TextDisabled("Sahne boş. Üst menüden veya Add Component ile başlayın.");
    }
}

// ---------------------------------------------------------------------------
// Collection row
// ---------------------------------------------------------------------------
void Outliner::render_collection_row(const std::string& subsystem_id,
                                     const std::string& display_name,
                                     SimulationOrchestrator& orchestrator,
                                     PropertiesPanel& properties,
                                     ModelEditor* model_editor) {
    Registry& registry = orchestrator.registry();
    Subsystem* sub = orchestrator.get_subsystem(subsystem_id);
    if (!sub) return;

    ImGui::PushID(("sub:" + subsystem_id).c_str());

    const bool hidden = is_collection_hidden(subsystem_id);
    const char* eye_icon = hidden ? "[x]" : "[v]";
    if (ImGui::SmallButton(eye_icon)) {
        if (hidden) hidden_collections().erase(subsystem_id);
        else        hidden_collections().insert(subsystem_id);
    }
    ImGui::SameLine();

    const std::string node_label = "📁 " + display_name +
                                   "  (" + std::to_string(sub->members().size()) + ")";
    bool open = ImGui::TreeNodeEx("Col",
                                  ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_SpanFullWidth,
                                  "%s", node_label.c_str());

    if (ImGui::BeginPopupContextItem("ColCtx")) {
        render_context_menu_collection(subsystem_id, orchestrator);
        ImGui::EndPopup();
    }

    if (open) {
        if (sub->members().empty()) {
            ImGui::TextDisabled("(empty)");
        } else {
            for (const Subsystem::Member& m : sub->members()) {
                Component* comp = registry.get(m.component_id);
                if (!comp) {
                    ImGui::TextDisabled("⚠ %s (missing)", m.component_id.c_str());
                    continue;
                }
                render_component_row(*comp, properties, model_editor,
                                     orchestrator, subsystem_id, m.role, false);
            }
        }
        ImGui::TreePop();
    }
    ImGui::PopID();
}

// ---------------------------------------------------------------------------
// Component row
// ---------------------------------------------------------------------------
void Outliner::render_component_row(const Component& comp,
                                    PropertiesPanel& properties,
                                    ModelEditor* model_editor,
                                    SimulationOrchestrator& orchestrator,
                                    const std::string& /*parent_collection_id*/,
                                    const std::string& role_label,
                                    bool is_loose) {
    const std::string& id = comp.id();
    const std::string display = component_display_name(comp);
    const bool is_selected = (properties.selected() == id);
    const bool is_hidden = is_component_hidden(id);
    const bool is_no_render = is_component_no_render(id);

    ImGui::PushID(id.c_str());

    // Visibility (eye) toggle.
    const char* eye = is_hidden ? "[x]" : "[v]";
    if (ImGui::SmallButton(eye)) {
        if (is_hidden) hidden_components().erase(id);
        else           hidden_components().insert(id);
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Toggle visibility");
    ImGui::SameLine();

    // Renderable (camera) toggle.
    const char* cam = is_no_render ? "[c-]" : "[c]";
    if (ImGui::SmallButton(cam)) {
        if (is_no_render) no_render_components().erase(id);
        else              no_render_components().insert(id);
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Toggle renderability");
    ImGui::SameLine();

    // Selection row.
    const std::string full_label = (role_label.empty() ? "" : ("[" + role_label + "] ")) +
                                   display + "  ##" + id;
    if (ImGui::Selectable(full_label.c_str(), is_selected,
                          ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick)) {
        properties.set_selected(id);
        if (model_editor) {
            // Tell the editor so it can focus the corresponding asset.
            // (Currently a no-op stub; full asset sync comes in Stage 5.)
        }
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s\nCategory: %s\nPlugin: %s%s",
                          id.c_str(),
                          comp.category().empty() ? "(unknown)" : comp.category().data(),
                          comp.plugin_type().empty() ? "(unknown)" : comp.plugin_type().data(),
                          is_loose ? "\nLoose" : "");
    }
    if (ImGui::BeginPopupContextItem("CompCtx")) {
        render_context_menu_component(id, orchestrator, properties, is_loose);
        ImGui::EndPopup();
    }

    ImGui::PopID();
}

// ---------------------------------------------------------------------------
// Context menus
// ---------------------------------------------------------------------------
void Outliner::render_context_menu_scene(SimulationOrchestrator& orchestrator) {
    if (ImGui::MenuItem("Add Collection (Yeni Koleksiyon)")) {
        int suffix = 0;
        std::string id = "Collection";
        while (orchestrator.get_subsystem(id)) {
            id = "Collection_" + std::to_string(++suffix);
        }
        orchestrator.add_subsystem(id);
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Hide All")) {
        for (auto& [id, _] : orchestrator.registry().all_components()) {
            hidden_components().insert(id);
        }
    }
    if (ImGui::MenuItem("Unhide All")) {
        hidden_components().clear();
        hidden_collections().clear();
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Expand All")) expand_all();
    if (ImGui::MenuItem("Collapse All")) collapse_all();
    ImGui::Separator();
    if (ImGui::MenuItem("Show Active in Tree")) show_active();
}

void Outliner::render_context_menu_collection(const std::string& subsystem_id,
                                              SimulationOrchestrator& orchestrator) {
    Subsystem* sub = orchestrator.get_subsystem(subsystem_id);
    if (!sub) return;

    if (ImGui::MenuItem("Select All Members")) {
        // Note: selection is single. Just pick the first member for now.
        if (!sub->members().empty()) {
            // Can't reach PropertiesPanel from here; the caller will need to
            // forward selection. For now this is a no-op message.
            spdlog::info("Outliner: Select All Members not yet implemented (multi-select pending).");
        }
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Isolate (hide others)")) {
        std::unordered_set<std::string> members = subsystem_member_ids(*sub);
        for (auto& [id, _] : orchestrator.registry().all_components()) {
            if (members.count(id) == 0) hidden_components().insert(id);
        }
        for (const std::string& other_id : orchestrator.list_subsystem_ids()) {
            if (other_id != subsystem_id) hidden_collections().insert(other_id);
        }
    }
    ImGui::Separator();

    // Rename (inline edit of display name).
    static char rename_buf[128] = {};
    if (ImGui::MenuItem("Rename...")) {
        std::string current = sub->display_name().empty() ? subsystem_id : sub->display_name();
        std::snprintf(rename_buf, sizeof(rename_buf), "%s", current.c_str());
        ImGui::OpenPopup("RenameCollection");
    }
    if (ImGui::BeginPopup("RenameCollection")) {
        ImGui::TextDisabled("Yeni isim:");
        ImGui::InputText("##newname", rename_buf, sizeof(rename_buf));
        if (ImGui::Button("OK", ImVec2(80, 0))) {
            sub->set_display_name(rename_buf);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(80, 0))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    if (ImGui::MenuItem("Delete Collection")) {
        orchestrator.remove_subsystem(subsystem_id);
    }
}

void Outliner::render_context_menu_component(const std::string& component_id,
                                             SimulationOrchestrator& orchestrator,
                                             PropertiesPanel& properties,
                                             bool /*is_loose*/) {
    if (ImGui::MenuItem("Select")) {
        properties.set_selected(component_id);
    }
    ImGui::Separator();
    if (is_component_hidden(component_id)) {
        if (ImGui::MenuItem("Reveal")) hidden_components().erase(component_id);
    } else {
        if (ImGui::MenuItem("Hide")) hidden_components().insert(component_id);
    }
    if (is_component_no_render(component_id)) {
        if (ImGui::MenuItem("Enable Render")) no_render_components().erase(component_id);
    } else {
        if (ImGui::MenuItem("Disable Render")) no_render_components().insert(component_id);
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Delete", "X")) {
        orchestrator.registry().remove(component_id);
        if (properties.selected() == component_id) properties.set_selected("");
        spdlog::info("Outliner: deleted component {}", component_id);
    }
}

} // namespace mechatron
