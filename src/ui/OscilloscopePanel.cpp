#include "OscilloscopePanel.hpp"
#include "core/SimulationOrchestrator.hpp"
#include "core/Registry.hpp"
#include "plugins/instruments/InstrumentPlugin.hpp"
#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace mechatron {

OscilloscopePanel::OscilloscopePanel(const std::string& component_id)
    : m_component_id(component_id) {
    m_polyline.reserve(8192);
}

static float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

template <class SampleContainer>
static double find_trigger_time(const SampleContainer& data,
                                double t_start, double t_end,
                                float level,
                                OscilloscopePanel::TriggerEdge edge) {
    if (data.size() < 2) return std::numeric_limits<double>::quiet_NaN();
    auto it = std::lower_bound(
        data.begin(), data.end(), t_start,
        [](const OscilloscopeSample& s, double t) { return s.time < t; });
    if (it == data.end()) return std::numeric_limits<double>::quiet_NaN();
    if (it != data.begin()) {
        --it;
    }

    OscilloscopeSample prev = *it;
    for (++it; it != data.end() && it->time <= t_end; ++it) {
        const OscilloscopeSample cur = *it;
        const bool prev_above = prev.voltage >= level;
        const bool cur_above = cur.voltage >= level;
        bool crossing = false;
        if (edge == OscilloscopePanel::TriggerEdge::Rising) {
            crossing = (!prev_above && cur_above);
        } else {
            crossing = (prev_above && !cur_above);
        }
        if (crossing) {
            const float dv = cur.voltage - prev.voltage;
            if (std::abs(dv) < 1e-9f) return cur.time;
            const float alpha = (level - prev.voltage) / dv;
            const double t = prev.time + static_cast<double>(alpha) * (cur.time - prev.time);
            if (t >= t_start && t <= t_end) return t;
            return cur.time;
        }
        prev = cur;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

struct ScopeMeasurements {
    bool ok = false;
    float vmin = 0.0f;
    float vmax = 0.0f;
    float vavg = 0.0f;
    float vrms = 0.0f;
    double freq_hz = 0.0;
    double duty = 0.0; // 0..1
};

template <class SampleContainer>
static ScopeMeasurements measure_window(const SampleContainer& data,
                                        double t_start, double t_end,
                                        float threshold) {
    ScopeMeasurements m{};
    if (data.size() < 2) return m;

    auto it0 = std::lower_bound(
        data.begin(), data.end(), t_start,
        [](const OscilloscopeSample& s, double t) { return s.time < t; });
    if (it0 == data.end()) return m;

    // accumulate + edges
    double sum = 0.0;
    double sumsq = 0.0;
    int count = 0;
    float vmin = std::numeric_limits<float>::infinity();
    float vmax = -std::numeric_limits<float>::infinity();

    bool prev_high = it0->voltage >= threshold;
    double prev_t = it0->time;
    double high_time = 0.0;

    std::vector<double> rising;
    rising.reserve(16);

    for (auto it = it0; it != data.end() && it->time <= t_end; ++it) {
        const float v = it->voltage;
        vmin = std::min(vmin, v);
        vmax = std::max(vmax, v);
        sum += v;
        sumsq += static_cast<double>(v) * static_cast<double>(v);
        ++count;

        const bool high = v >= threshold;
        const double t = it->time;
        const double dt = t - prev_t;
        if (dt > 0.0 && prev_high) {
            high_time += dt;
        }

        if (!prev_high && high) {
            rising.push_back(t);
        }

        prev_high = high;
        prev_t = t;
    }

    if (count <= 0) return m;
    m.ok = true;
    m.vmin = vmin;
    m.vmax = vmax;
    m.vavg = static_cast<float>(sum / static_cast<double>(count));
    m.vrms = static_cast<float>(std::sqrt(sumsq / static_cast<double>(count)));

    const double window = std::max(1e-9, t_end - t_start);
    m.duty = clampf(static_cast<float>(high_time / window), 0.0f, 1.0f);

    if (rising.size() >= 2) {
        // estimate freq from average period between consecutive rising edges
        double period_sum = 0.0;
        int periods = 0;
        for (size_t i = 1; i < rising.size(); ++i) {
            const double p = rising[i] - rising[i - 1];
            if (p > 0.0) {
                period_sum += p;
                ++periods;
            }
        }
        if (periods > 0) {
            const double period = period_sum / static_cast<double>(periods);
            if (period > 0.0) m.freq_hz = 1.0 / period;
        }
    }
    return m;
}

void OscilloscopePanel::render(SimulationOrchestrator& orchestrator) {
    auto* scope = orchestrator.registry().get_as<OscilloscopeComponent>(m_component_id);
    if (!scope) {
        ImGui::TextDisabled("Oscilloscope '%s' not found", m_component_id.c_str());
        return;
    }

    // Control bar
    if (ImGui::Button(m_paused ? "Run" : "Pause", ImVec2(60, 0))) {
        m_paused = !m_paused;
        if (m_paused) {
            // freeze time window at the moment of pausing
            double current_time = 0.0;
            for (int ch = 0; ch < kChannelCount; ++ch) {
                const auto& data = scope->channel_data(ch);
                if (!data.empty()) current_time = std::max(current_time, data.back().time);
            }
            m_hold_time = current_time;
        }
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("Hold Acq", &m_hold_acquisition)) {
        // stop/start recording into component buffers
        // (scope is still part of ngspice network; this only affects data capture)
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear", ImVec2(60, 0))) {
        scope->clear_data();
    }

    ImGui::SameLine();
    ImGui::SetNextItemWidth(120);
    ImGui::SliderFloat("V/div", &m_volts_per_div, 0.01f, 50.0f, "%.3f");

    ImGui::SameLine();
    ImGui::SetNextItemWidth(120);
    float ts = static_cast<float>(m_time_scale);
    if (ImGui::SliderFloat("T Scale", &ts, 0.01f, 10.0f, "%.2f s")) {
        m_time_scale = static_cast<double>(ts);
    }

    ImGui::SameLine();
    ImGui::SetNextItemWidth(120);
    ImGui::SliderFloat("V Offset", &m_v_offset, -50.0f, 50.0f, "%.2f V");

    // Sync scale to component (kept for persistence/serialization; panel is the source of truth)
    const float full_scale = std::max(1e-6f, m_volts_per_div * 8.0f);
    scope->set_voltage_scale(full_scale);
    scope->set_time_scale(m_time_scale);

    // Channel enable checkboxes
    ImGui::Spacing();
    static const char* ch_names[kChannelCount] = {"CH1", "CH2", "CH3", "CH4", "CH5", "CH6"};
    static const ImVec4 ch_colors[kChannelCount] = {
        {1.0f, 1.0f, 0.0f, 1.0f},   // Yellow
        {0.0f, 1.0f, 1.0f, 1.0f},   // Cyan
        {1.0f, 0.0f, 1.0f, 1.0f},   // Magenta
        {0.0f, 1.0f, 0.0f, 1.0f},   // Green
        {1.0f, 0.0f, 0.0f, 1.0f},   // Red
        {0.4f, 0.4f, 1.0f, 1.0f},   // Blue
    };

    for (int ch = 0; ch < kChannelCount; ++ch) {
        ImGui::SameLine(60.0f * ch);
        ImGui::PushStyleColor(ImGuiCol_Text, ch_colors[ch]);
        ImGui::Checkbox(ch_names[ch], &m_channel_enabled[ch]);
        ImGui::PopStyleColor();
        const bool enabled = m_channel_enabled[ch] && !m_hold_acquisition;
        scope->set_channel_enabled(ch, enabled);
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Trigger + measurements bar
    if (ImGui::Checkbox("Trigger", &m_trigger_enabled)) {
        // noop
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(70);
    ImGui::SliderInt("CH##trig", &m_trigger_channel, 0, kChannelCount - 1);
    ImGui::SameLine();
    int edge = (m_trigger_edge == TriggerEdge::Rising) ? 0 : 1;
    ImGui::SetNextItemWidth(90);
    if (ImGui::Combo("Edge", &edge, "Rising\0Falling\0")) {
        m_trigger_edge = (edge == 0) ? TriggerEdge::Rising : TriggerEdge::Falling;
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120);
    ImGui::SliderFloat("Level", &m_trigger_level, -50.0f, 50.0f, "%.2f V");
    ImGui::SameLine();
    ImGui::Checkbox("Align", &m_trigger_holdoff);

    ImGui::SameLine();
    ImGui::Checkbox("Cursors", &m_show_cursors);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(70);
    ImGui::SliderInt("Meas##ch", &m_measure_channel, 0, kChannelCount - 1);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Waveform display area
    ImVec2 avail = ImGui::GetContentRegionAvail();
    float wave_height = avail.y - 4.0f;
    float wave_width = avail.x;

    if (wave_width < 50.0f || wave_height < 50.0f) return;

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 cursor = ImGui::GetCursorScreenPos();

    // Background
    draw_list->AddRectFilled(cursor, ImVec2(cursor.x + wave_width, cursor.y + wave_height),
                             IM_COL32(10, 10, 30, 255));

    // Grid: 10 columns x 8 rows (standard oscilloscope layout)
    ImU32 grid_color = IM_COL32(40, 40, 60, 255);
    ImU32 grid_color_center = IM_COL32(60, 60, 80, 255);

    for (int i = 0; i <= 10; ++i) {
        float x = cursor.x + (wave_width * i) / 10.0f;
        ImU32 col = (i == 5) ? grid_color_center : grid_color;
        draw_list->AddLine(ImVec2(x, cursor.y), ImVec2(x, cursor.y + wave_height), col);
    }
    for (int i = 0; i <= 8; ++i) {
        float y = cursor.y + (wave_height * i) / 8.0f;
        ImU32 col = (i == 4) ? grid_color_center : grid_color;
        draw_list->AddLine(ImVec2(cursor.x, y), ImVec2(cursor.x + wave_width, y), col);
    }

    // Border
    draw_list->AddRect(cursor, ImVec2(cursor.x + wave_width, cursor.y + wave_height),
                       IM_COL32(80, 80, 100, 255));

    // Trigger level indicator (horizontal dashed line)
    if (m_trigger_enabled) {
        const float v_full = std::max(1e-6f, m_volts_per_div * 8.0f);
        const float v_center = 0.5f * v_full + m_v_offset;
        const float ny = clampf(0.5f - (m_trigger_level - v_center) / v_full, 0.0f, 1.0f);
        const float y = cursor.y + ny * wave_height;
        const ImU32 trig_col = IM_COL32(255, 255, 255, 90);
        const float dash = 6.0f;
        for (float x = cursor.x; x < cursor.x + wave_width; x += dash * 2.0f) {
            draw_list->AddLine(ImVec2(x, y), ImVec2(std::min(x + dash, cursor.x + wave_width), y), trig_col);
        }
    }

    // Reserve space for the waveform area (invisible button for interaction)
    ImGui::Dummy(ImVec2(wave_width, wave_height));

    // Get current time from scope's last sample
    double current_time = 0.0;
    for (int ch = 0; ch < kChannelCount; ++ch) {
        auto& data = scope->channel_data(ch);
        if (!data.empty()) {
            current_time = std::max(current_time, data.back().time);
        }
    }
    if (m_paused) current_time = m_hold_time;

    double t_end = current_time;
    double t_start = t_end - m_time_scale;

    // Trigger align: attempt to shift window so that first trigger is near 20% of the screen.
    if (m_trigger_enabled && m_trigger_holdoff) {
        const int tch = std::clamp(m_trigger_channel, 0, kChannelCount - 1);
        const auto& tdata = scope->channel_data(tch);
        const double search_start = std::max(0.0, t_start);
        const double trig_t = find_trigger_time(tdata, search_start, t_end, m_trigger_level, m_trigger_edge);
        if (std::isfinite(trig_t)) {
            const double desired_x = 0.2; // 20% from left
            t_start = trig_t - desired_x * m_time_scale;
            t_end = t_start + m_time_scale;
        }
    }

    // Measurements (for PWM-like signals): use threshold as trigger level
    ScopeMeasurements meas{};
    {
        const int mch = std::clamp(m_measure_channel, 0, kChannelCount - 1);
        const auto& mdata = scope->channel_data(mch);
        meas = measure_window(mdata, t_start, t_end, m_trigger_level);
    }

    // Draw waveforms
    for (int ch = 0; ch < kChannelCount; ++ch) {
        if (!m_channel_enabled[ch]) continue;

        const auto& data = scope->channel_data(ch);
        if (data.empty()) continue;

        // Find samples in visible range
        auto it_start = std::lower_bound(data.begin(), data.end(), t_start,
            [](const OscilloscopeSample& s, double t) { return s.time < t; });
        auto it_end = data.end();

        if (it_start == it_end) continue;

        // Build polyline points (decimate to pixel width)
        ImU32 color = ImGui::GetColorU32(ch_colors[ch]);
        m_polyline.clear();

        const float v_full = std::max(1e-6f, m_volts_per_div * 8.0f);
        const float v_center = 0.5f * v_full + m_v_offset;
        const int max_points = static_cast<int>(wave_width); // ~1 sample per pixel
        const int total = static_cast<int>(std::distance(it_start, it_end));
        const int step = std::max(1, total / std::max(1, max_points));
        int idx = 0;

        for (auto it = it_start; it != it_end; ++it, ++idx) {
            if ((idx % step) != 0) continue;
            const float nx = clampf(static_cast<float>((it->time - t_start) / m_time_scale), 0.0f, 1.0f);
            const float vy = it->voltage;
            // Map voltage to screen: center at mid-line, show +/- 4 divisions
            const float ny = clampf(0.5f - (vy - v_center) / v_full, 0.0f, 1.0f);
            m_polyline.push_back(ImVec2(cursor.x + nx * wave_width,
                                        cursor.y + ny * wave_height));
        }

        if (m_polyline.size() >= 2) {
            draw_list->AddPolyline(m_polyline.data(), static_cast<int>(m_polyline.size()),
                                   color, false, 1.5f);
        }
    }

    // Cursors interaction
    if (m_show_cursors) {
        // Initialize cursors if unset
        if (m_cursor_t0 == 0.0 && m_cursor_t1 == 0.0) {
            m_cursor_t0 = t_start + 0.3 * m_time_scale;
            m_cursor_t1 = t_start + 0.7 * m_time_scale;
        }

        const double t0 = std::clamp(m_cursor_t0, t_start, t_end);
        const double t1 = std::clamp(m_cursor_t1, t_start, t_end);
        const float x0 = cursor.x + static_cast<float>((t0 - t_start) / m_time_scale) * wave_width;
        const float x1 = cursor.x + static_cast<float>((t1 - t_start) / m_time_scale) * wave_width;
        const ImU32 cur_col = IM_COL32(220, 220, 220, 160);
        draw_list->AddLine(ImVec2(x0, cursor.y), ImVec2(x0, cursor.y + wave_height), cur_col, 1.0f);
        draw_list->AddLine(ImVec2(x1, cursor.y), ImVec2(x1, cursor.y + wave_height), cur_col, 1.0f);

        // Simple drag handle: invisible buttons near top
        ImGui::SetCursorScreenPos(ImVec2(x0 - 6, cursor.y));
        ImGui::InvisibleButton("cur0", ImVec2(12, wave_height));
        if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            const float mx = ImGui::GetIO().MousePos.x;
            const float nx = clampf((mx - cursor.x) / wave_width, 0.0f, 1.0f);
            m_cursor_t0 = t_start + nx * m_time_scale;
        }
        ImGui::SetCursorScreenPos(ImVec2(x1 - 6, cursor.y));
        ImGui::InvisibleButton("cur1", ImVec2(12, wave_height));
        if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            const float mx = ImGui::GetIO().MousePos.x;
            const float nx = clampf((mx - cursor.x) / wave_width, 0.0f, 1.0f);
            m_cursor_t1 = t_start + nx * m_time_scale;
        }

        const double dt = std::abs(m_cursor_t1 - m_cursor_t0);
        char buf[128];
        std::snprintf(buf, sizeof(buf), "dT=%.6fs", dt);
        draw_list->AddText(ImVec2(cursor.x + 4, cursor.y + wave_height - 18),
                           IM_COL32(220, 220, 220, 200), buf);
    }

    // Time/Voltage labels
    draw_list->AddText(ImVec2(cursor.x + 4, cursor.y + 2),
                       IM_COL32(200, 200, 200, 200),
                       (std::to_string(m_volts_per_div) + "V/div").c_str());
    draw_list->AddText(ImVec2(cursor.x + wave_width - 60, cursor.y + 2),
                       IM_COL32(200, 200, 200, 200),
                       (std::to_string(static_cast<float>(m_time_scale)) + "s").c_str());

    if (meas.ok) {
        char mbuf[256];
        std::snprintf(mbuf, sizeof(mbuf),
                      "Vmin=%.3f  Vmax=%.3f  Vavg=%.3f  Vrms=%.3f  f=%.2fHz  duty=%.1f%%",
                      meas.vmin, meas.vmax, meas.vavg, meas.vrms,
                      meas.freq_hz, meas.duty * 100.0);
        draw_list->AddText(ImVec2(cursor.x + 4, cursor.y + 18),
                           IM_COL32(200, 200, 200, 220), mbuf);
    }
}

} // namespace mechatron
