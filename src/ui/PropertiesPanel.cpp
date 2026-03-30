#include "PropertiesPanel.hpp"
#include "SimulationOrchestrator.hpp"
#include <imgui.h>

namespace mechatron {

void PropertiesPanel::render(SimulationOrchestrator& orchestrator) {
    ImGui::Begin("Properties");

    if (m_selected_id.empty()) {
        ImGui::Text("No component selected");
    } else {
        Component* comp = orchestrator.registry().get(m_selected_id);
        if (!comp) {
            ImGui::Text("Component not found: %s", m_selected_id.c_str());
        } else {
            ImGui::Text("ID: %s", comp->id().c_str());
            ImGui::Separator();
            ImGui::Text("Plugin: %s", comp->plugin_type().data());
            ImGui::Text("Type: %s", comp->component_type().data());
            ImGui::Text("Category: %s", comp->category().data());

            ImGui::Separator();
            ImGui::Text("Transform");

            auto& t = comp->transform();
            bool changed = false;
            changed |= ImGui::DragFloat3("Position", &t.position.x, 0.1f);
            changed |= ImGui::DragFloat3("Scale", &t.scale.x, 0.01f, 0.01f, 100.0f);

            ImGui::Separator();
            ImGui::Text("Ports");
            auto ports = comp->get_ports();
            if (ports.empty()) {
                ImGui::TextDisabled("No ports");
            } else {
                for (auto* port : ports) {
                    ImGui::BulletText("%s [%s]", port->name().data(),
                        port->direction() == PortDirection::Input ? "IN" :
                        port->direction() == PortDirection::Output ? "OUT" : "BIDI");
                }
            }
        }
    }

    ImGui::End();
}

} // namespace mechatron
