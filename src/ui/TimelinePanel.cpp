#include "TimelinePanel.hpp"
#include "SimulationOrchestrator.hpp"
#include <imgui.h>
#include <cstdio>

namespace mechatron {

void TimelinePanel::render(SimulationOrchestrator& orchestrator) {
    // No Begin/End - we're inside a tab

    auto& time = orchestrator.time_manager();
    auto state = time.state();

    // Playback controls
    float button_size = 32.0f;

    // Start/Stop
    if (state == SimulationState::Stopped) {
        if (ImGui::Button("Play", ImVec2(button_size * 2, button_size))) {
            orchestrator.start();
        }
    } else {
        if (ImGui::Button("Stop", ImVec2(button_size * 2, button_size))) {
            orchestrator.stop();
        }
    }
    ImGui::SameLine();

    // Pause/Resume
    if (state == SimulationState::Running) {
        if (ImGui::Button("Pause", ImVec2(button_size * 2, button_size))) {
            orchestrator.pause();
        }
    } else if (state == SimulationState::Paused) {
        if (ImGui::Button("Resume", ImVec2(button_size * 2, button_size))) {
            orchestrator.resume();
        }
    }
    ImGui::SameLine();

    // Step
    if (state == SimulationState::Paused || state == SimulationState::Stopped) {
        if (ImGui::Button("Step", ImVec2(button_size * 2, button_size))) {
            if (state == SimulationState::Stopped) orchestrator.start();
            orchestrator.step();
        }
    }

    ImGui::Separator();

    // Simulation info
    double sim_time = time.simulation_time();
    uint64_t tick = time.current_tick();

    ImGui::Text("Time: %.6f s", sim_time);
    ImGui::Text("Tick: %llu", (unsigned long long)tick);

    const char* state_str = "Stopped";
    if (state == SimulationState::Running) state_str = "Running";
    else if (state == SimulationState::Paused) state_str = "Paused";
    else if (state == SimulationState::Stepping) state_str = "Stepping";
    ImGui::Text("State: %s", state_str);

    ImGui::Separator();

    // Realtime factor
    float factor = static_cast<float>(time.realtime_factor());
    if (ImGui::SliderFloat("Speed", &factor, 0.1f, 10.0f, "%.1fx")) {
        time.set_realtime_factor(factor);
    }

    // Physics step info
    ImGui::Text("Physics Step: %.3f ms", time.physics_step_size() * 1000.0);

    // Deterministic toggle
    bool det = time.is_deterministic();
    if (ImGui::Checkbox("Deterministic", &det)) {
        time.set_deterministic(det);
    }

    ImGui::Separator();

    // Timeline bar
    float avail = ImGui::GetContentRegionAvail().x;
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.15f, 0.15f, 0.15f, 1.0f));
    ImGui::BeginChild("TimelineBar", ImVec2(avail, 40), true);

    // Draw tick marks
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    float bar_width = avail - 10;

    // 1 second divisions
    for (int i = 0; i <= 10; ++i) {
        float x = p.x + 5 + (bar_width * i / 10.0f);
        dl->AddLine(ImVec2(x, p.y), ImVec2(x, p.y + 30), IM_COL32(100, 100, 100, 255));
        char label[16];
        std::snprintf(label, sizeof(label), "%ds", i);
        dl->AddText(ImVec2(x + 2, p.y + 15), IM_COL32(150, 150, 150, 255), label);
    }

    // Current time marker
    float progress = std::fmod(sim_time, 10.0) / 10.0f;
    float marker_x = p.x + 5 + bar_width * progress;
    dl->AddLine(ImVec2(marker_x, p.y), ImVec2(marker_x, p.y + 30), IM_COL32(0, 200, 100, 255), 2.0f);

    ImGui::EndChild();
    ImGui::PopStyleColor();
}

} // namespace mechatron
