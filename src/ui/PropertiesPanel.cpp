#include "PropertiesPanel.hpp"
#include "SimulationOrchestrator.hpp"
#include <imgui.h>
#include "Theme.hpp"
#include <algorithm>

namespace mechatron {

void PropertiesPanel::refresh_physical_ports(bool force) {
    const auto now = std::chrono::steady_clock::now();
    if (!force && !m_physical_ports.empty() &&
        now - m_last_port_refresh < std::chrono::seconds(2)) {
        return;
    }

    m_physical_ports = SerialPort::list_ports();
    m_last_port_refresh = now;
}

void PropertiesPanel::render(SimulationOrchestrator& orchestrator) {
    // No Begin/End - we're inside a tab

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

            if (comp->physical_link_supported()) {
                ImGui::Separator();
                ImGui::Text("Physical Link");

                std::string port;
                int baud = 115200;
                comp->physical_link_get_config(port, baud);

                refresh_physical_ports();

                const auto selected_it = std::find_if(
                    m_physical_ports.begin(),
                    m_physical_ports.end(),
                    [&](const SerialPortInfo& info) { return info.port == port; });

                if (port.empty() && !m_physical_ports.empty()) {
                    port = m_physical_ports.front().port;
                    comp->physical_link_set_config(port, baud);
                }

                const char* preview = port.empty() ? "No serial ports" : port.c_str();
                if (selected_it != m_physical_ports.end()) {
                    preview = selected_it->display_name.c_str();
                }

                ImGui::SetNextItemWidth(-70.0f);
                if (ImGui::BeginCombo("Port##PhysicalLink", preview)) {
                    if (m_physical_ports.empty()) {
                        ImGui::TextDisabled("No Arduino/serial device found");
                    }
                    for (const auto& info : m_physical_ports) {
                        const bool selected = info.port == port;
                        std::string label = info.display_name;
                        if (info.likely_arduino) {
                            label += "  [Arduino]";
                        }
                        if (ImGui::Selectable(label.c_str(), selected)) {
                            port = info.port;
                            comp->physical_link_set_config(port, baud);
                        }
                        if (selected) {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                ImGui::SameLine();
                if (ImGui::Button("Refresh##PhysicalLink")) {
                    refresh_physical_ports(true);
                    if (port.empty() && !m_physical_ports.empty()) {
                        port = m_physical_ports.front().port;
                        comp->physical_link_set_config(port, baud);
                    }
                }

                if (ImGui::InputInt("Baud##PhysicalLink", &baud)) {
                    if (baud < 1200) baud = 1200;
                    if (baud > 2000000) baud = 2000000;
                    comp->physical_link_set_config(port, baud);
                }

                if (comp->physical_link_is_connected()) {
                    ImGui::TextColored(Theme::CurrentPalette().success, "Status: Connected");
                    if (ImGui::Button("Disconnect##PhysicalLink")) {
                        comp->physical_link_disconnect();
                    }
                } else {
                    ImGui::TextColored(Theme::CurrentPalette().warning, "Status: Disconnected");
                    if (port.empty()) {
                        ImGui::BeginDisabled();
                    }
                    if (ImGui::Button("Connect##PhysicalLink")) {
                        (void)comp->physical_link_connect();
                    }
                    if (port.empty()) {
                        ImGui::EndDisabled();
                    }
                }
            }
        }
    }
}

} // namespace mechatron
