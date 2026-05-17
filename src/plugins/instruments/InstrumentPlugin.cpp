#include "InstrumentPlugin.hpp"
#include <spdlog/spdlog.h>

namespace mechatron {

// ============================================================================
// OscilloscopeComponent
// ============================================================================

static bool voltage_like_value(const Port& port, float& voltage) {
    if (const float* val = port.get_value<float>()) {
        voltage = *val;
        return true;
    }
    if (const double* d = port.get_value<double>()) {
        voltage = static_cast<float>(*d);
        return true;
    }
    if (const bool* b = port.get_value<bool>()) {
        voltage = *b ? 5.0f : 0.0f;
        return true;
    }
    return false;
}

OscilloscopeComponent::OscilloscopeComponent()
    : m_ports{{
        Port("CH1", PortDomain::Analog, PortDirection::Input),
        Port("CH2", PortDomain::Analog, PortDirection::Input),
        Port("CH3", PortDomain::Analog, PortDirection::Input),
        Port("CH4", PortDomain::Analog, PortDirection::Input),
        Port("CH5", PortDomain::Analog, PortDirection::Input),
        Port("CH6", PortDomain::Analog, PortDirection::Input)
    }}
{
    for (auto& port : m_ports) {
        assign_port_owner(&port);
    }
    m_channel_enabled.fill(true);
}

void OscilloscopeComponent::update(double dt) {
    m_elapsed += dt;

    for (int ch = 0; ch < kChannelCount; ++ch) {
        if (!m_channel_enabled[ch]) continue;

        // Treat input as "voltage-like" regardless of whether the upstream pin
        // is analog (float) or digital (bool). If net propagation has not yet
        // copied the value into the channel, read the connected peer directly:
        // an oscilloscope probe observes the node it is attached to.
        float voltage = 0.0f;
        bool found = false;
        for (auto* conn : m_ports[ch].connections()) {
            if (!conn) continue;
            const Port* peer = (conn->source == &m_ports[ch]) ? conn->target : conn->source;
            if (peer && voltage_like_value(*peer, voltage)) {
                found = true;
                break;
            }
        }
        if (!found) {
            found = voltage_like_value(m_ports[ch], voltage);
        }
        if (found) {
            m_ports[ch].set_value(voltage);
        }

        auto& buf = m_buffers[ch];
        buf.push_back({m_elapsed, voltage});
        while (static_cast<int>(buf.size()) > kMaxSamples) buf.pop_front();
    }
}

void OscilloscopeComponent::serialize(nlohmann::json& out) const {
    out["time_scale"] = m_time_scale;
    out["voltage_scale"] = m_voltage_scale;
    auto ch_states = nlohmann::json::array();
    for (int i = 0; i < kChannelCount; ++i) {
        ch_states.push_back(m_channel_enabled[i]);
    }
    out["channel_enabled"] = ch_states;
}

void OscilloscopeComponent::deserialize(const nlohmann::json& in) {
    if (in.contains("time_scale")) m_time_scale = in["time_scale"].get<double>();
    if (in.contains("voltage_scale")) m_voltage_scale = in["voltage_scale"].get<float>();
    if (in.contains("channel_enabled") && in["channel_enabled"].is_array()) {
        auto arr = in["channel_enabled"];
        for (int i = 0; i < kChannelCount && i < static_cast<int>(arr.size()); ++i) {
            m_channel_enabled[i] = arr[i].get<bool>();
        }
    }
}

std::vector<Port*> OscilloscopeComponent::get_ports() {
    std::vector<Port*> ports;
    for (auto& p : m_ports) {
        ports.push_back(&p);
    }
    return ports;
}

const std::deque<OscilloscopeSample>& OscilloscopeComponent::channel_data(int ch) const {
    return m_buffers[ch];
}

bool OscilloscopeComponent::channel_enabled(int ch) const {
    return m_channel_enabled[ch];
}

void OscilloscopeComponent::set_channel_enabled(int ch, bool enabled) {
    m_channel_enabled[ch] = enabled;
}

double OscilloscopeComponent::time_scale() const {
    return m_time_scale;
}

void OscilloscopeComponent::set_time_scale(double ts) {
    m_time_scale = ts;
}

float OscilloscopeComponent::voltage_scale() const {
    return m_voltage_scale;
}

void OscilloscopeComponent::set_voltage_scale(float vs) {
    m_voltage_scale = vs;
}

void OscilloscopeComponent::clear_data() {
    for (auto& buf : m_buffers) {
        buf.clear();
    }
}

// ============================================================================
// InstrumentPlugin
// ============================================================================

std::vector<ComponentDescriptor> InstrumentPlugin::components() const {
    return {
        {"oscilloscope", "Oscilloscope", "instrument", "6-channel oscilloscope with waveform display"}
    };
}

std::unique_ptr<Component> InstrumentPlugin::create(std::string_view type) {
    if (type == "oscilloscope") {
        return std::make_unique<OscilloscopeComponent>();
    }
    return nullptr;
}

void InstrumentPlugin::on_register(PluginHost& host) {
    spdlog::info("InstrumentPlugin registered");
}

void InstrumentPlugin::on_unregister() {
}

} // namespace mechatron
