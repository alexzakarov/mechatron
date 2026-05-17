#pragma once

#include "core/PluginHost.hpp"
#include "core/Component.hpp"
#include "core/Port.hpp"
#include <vector>
#include <deque>
#include <array>
#include <string_view>

namespace mechatron {

struct OscilloscopeSample {
    double time = 0.0;
    float voltage = 0.0f;
};

class OscilloscopeComponent : public Component {
public:
    static constexpr int kChannelCount = 6;
    static constexpr int kMaxSamples = 4096;

    OscilloscopeComponent();

    std::string_view plugin_type() const override { return "instrument"; }
    std::string_view component_type() const override { return "oscilloscope"; }
    std::string_view category() const override { return "instrument"; }

    void update(double dt) override;

    void serialize(nlohmann::json& out) const override;
    void deserialize(const nlohmann::json& in) override;

    std::vector<Port*> get_ports() override;

    // Data access
    const std::deque<OscilloscopeSample>& channel_data(int ch) const;
    bool channel_enabled(int ch) const;
    void set_channel_enabled(int ch, bool enabled);
    double time_scale() const;
    void set_time_scale(double ts);
    float voltage_scale() const;
    void set_voltage_scale(float vs);
    void clear_data();

private:
    // Ports
    std::array<Port, kChannelCount> m_ports;

    // Per-channel ring buffers
    std::array<std::deque<OscilloscopeSample>, kChannelCount> m_buffers;

    // Per-channel enabled state
    std::array<bool, kChannelCount> m_channel_enabled{};

    // Scale settings
    double m_time_scale = 1.0;   // seconds visible in window
    float m_voltage_scale = 5.0f; // volts full-scale

    double m_elapsed = 0.0;
};

class InstrumentPlugin : public IMechatronPlugin {
public:
    std::string_view name() const override { return "instrument"; }
    std::string_view version() const override { return "1.0.0"; }

    std::vector<ComponentDescriptor> components() const override;
    std::unique_ptr<Component> create(std::string_view type) override;

    void on_register(PluginHost& host) override;
    void on_unregister() override;
};

} // namespace mechatron
