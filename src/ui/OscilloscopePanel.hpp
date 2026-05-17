#pragma once

#include <string>
#include <imgui.h>
#include <vector>

namespace mechatron {

class SimulationOrchestrator;

class OscilloscopePanel {
public:
    enum class TriggerEdge { Rising = 0, Falling = 1 };

    explicit OscilloscopePanel(const std::string& component_id);

    void render(SimulationOrchestrator& orchestrator);

    const std::string& component_id() const { return m_component_id; }

private:
    std::string m_component_id;

    // Display state
    bool m_paused = false;
    bool m_hold_acquisition = false; // when true, stops recording into the component buffers
    double m_hold_time = 0.0;
    double m_time_scale = 1.0;
    float m_volts_per_div = 1.0f;      // V/div (vertical)
    float m_v_offset = 0.0f;           // global vertical offset (V)

    static constexpr int kChannelCount = 6;
    bool m_channel_enabled[kChannelCount] = {true, true, true, true, true, true};

    // Trigger / cursors / measurements (PWM inspection)
    bool m_trigger_enabled = false;
    int m_trigger_channel = 0;
    TriggerEdge m_trigger_edge = TriggerEdge::Rising;
    float m_trigger_level = 2.5f;
    bool m_trigger_holdoff = true; // aligns window start to trigger when possible

    bool m_show_cursors = false;
    double m_cursor_t0 = 0.0;
    double m_cursor_t1 = 0.0;

    int m_measure_channel = 0;

    // Scratch buffers to avoid per-frame allocations
    std::vector<ImVec2> m_polyline;
};

} // namespace mechatron
