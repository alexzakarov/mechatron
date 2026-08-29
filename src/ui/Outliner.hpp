#pragma once

// ============================================================================
// Outliner — Blender-style scene hierarchy tree.
//
// Renders the project/scene contents as an expandable tree:
//   Scene
//   ├─ Collections (Subsystems)
//   │  ├─ Member components (with role labels)
//   │  └─ ...
//   └─ Loose Components
//      └─ (component_type [id])
//
// Selection is shared with PropertiesPanel so the existing right-side
// inspector and viewport selection stay in sync.
//
// Right-click on any row reveals OUTLINER_OT_* operations:
//   - Select / Rename / Delete
//   - Toggle Visibility / Renderability
//   - Collection ops: Add Collection, Link to Collection, Isolate
//   - Show Hierarchy / Show Active
//   - Hide / Unhide All
//
// Outliner is a stateless renderer: it derives the tree from the
// SimulationOrchestrator every frame and emits no state of its own beyond
// a set of per-row ImGui IDs and an expand-state cache.
// ============================================================================

#include <string>

namespace mechatron {

class SimulationOrchestrator;
class PropertiesPanel;
class ModelEditor;
class Registry;
class Component;

class Outliner {
public:
    Outliner() = default;
    ~Outliner() = default;

    // Render the outliner panel. Caller is responsible for being inside an
    // ImGui::BeginChild() or other suitable container — this function does
    // NOT open its own window.
    //
    // orchestrator  - source of registry + subsystems
    // properties    - selection sink (kept in sync with click selection)
    // model_editor  - optional; when non-null the outliner will also tell the
    //                  editor which asset is selected so the 3D view mirrors
    //                  the outliner.
    void render(SimulationOrchestrator& orchestrator,
                PropertiesPanel& properties,
                ModelEditor* model_editor);

    // Returns true if the most recent render() call drew any rows. Useful for
    // the host UI to decide whether to show "empty scene" hints.
    bool has_content() const { return m_has_content; }

    // Expand/collapse state controls. These persist across frames via this
    // Outliner instance.
    void expand_all();
    void collapse_all();
    void show_active();    // expand parent chain of current selection

private:
    // Per-row expand cache. Keyed by stable id ("scene", "subsys:<id>",
    // "loose", or component id).
    bool is_expanded(const std::string& key) const;
    void set_expanded(const std::string& key, bool open);

    void render_component_row(const Component& comp,
                              PropertiesPanel& properties,
                              ModelEditor* model_editor,
                              SimulationOrchestrator& orchestrator,
                              const std::string& parent_collection_id,
                              const std::string& role_label,
                              bool is_loose);

    void render_collection_row(const std::string& subsystem_id,
                               const std::string& display_name,
                               SimulationOrchestrator& orchestrator,
                               PropertiesPanel& properties,
                               ModelEditor* model_editor);

    void render_context_menu_component(const std::string& component_id,
                                       SimulationOrchestrator& orchestrator,
                                       PropertiesPanel& properties,
                                       bool is_loose);
    void render_context_menu_collection(const std::string& subsystem_id,
                                        SimulationOrchestrator& orchestrator);
    void render_context_menu_scene(SimulationOrchestrator& orchestrator);

    bool m_has_content = false;
};

} // namespace mechatron
