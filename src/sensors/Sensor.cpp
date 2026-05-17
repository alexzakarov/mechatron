#include "Sensor.hpp"
#include <spdlog/spdlog.h>
#include <cmath>
#include <random>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mechatron {

// Proximity Sensor
ProximitySensor::ProximitySensor() {
    m_min_value = 0.0f;
    m_max_value = 5.0f;

    // Create sensor ports
    m_v_plus_port = std::make_unique<Port>("V+", PortDomain::Electrical, PortDirection::Input);
    m_v_plus_port->set_value(0.0f);

    m_output_port = std::make_unique<Port>("OUT", PortDomain::Analog, PortDirection::Output);
    m_output_port->set_value(0.0f);

    m_gnd_port = std::make_unique<Port>("GND", PortDomain::Electrical, PortDirection::Input);
    m_gnd_port->set_value(0.0f);
}

void ProximitySensor::update(double dt) {
    // Calculate distance to target
    Vec3 diff = {
        m_target_position.x - transform().position.x,
        m_target_position.y - transform().position.y,
        m_target_position.z - transform().position.z
    };

    float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);

    // Clamp to max range
    m_distance = std::min(dist, m_max_range);

    // Add small noise if configured
    if (m_noise_level > 0.0f) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::normal_distribution<float> noise(0.0f, m_noise_level);
        m_distance += noise(gen);
        m_distance = std::max(0.0f, m_distance);
    }

    // Convert distance to output voltage
    // Closer = higher voltage, farther = lower voltage
    float ratio = 1.0f - (m_distance / m_max_range);
    float signal_voltage = ratio * m_max_value;

    // Write to output port
    if (m_output_port) {
        m_output_port->set_value(signal_voltage);
    }
}

void ProximitySensor::serialize(nlohmann::json& out) const {
    out["max_range"] = m_max_range;
    out["fov"] = m_fov;
    out["distance"] = m_distance;
}

void ProximitySensor::deserialize(const nlohmann::json& in) {
    if (in.contains("max_range")) m_max_range = in["max_range"];
    if (in.contains("fov")) m_fov = in["fov"];
}

// Limit Switch
LimitSwitch::LimitSwitch() {
    m_min_value = 0.0f;
    m_max_value = 5.0f;

    // Create 2-terminal switch contact ports
    m_terminal_a_port = std::make_unique<Port>("A", PortDomain::Electrical, PortDirection::Bidirectional);
    m_terminal_a_port->set_value(0.0f);

    m_terminal_b_port = std::make_unique<Port>("B", PortDomain::Electrical, PortDirection::Bidirectional);
    m_terminal_b_port->set_value(0.0f);
}

void LimitSwitch::update(double dt) {
    // Check distance to trigger position
    Vec3 diff = {
        m_trigger_position.x - transform().position.x,
        m_trigger_position.y - transform().position.y,
        m_trigger_position.z - transform().position.z
    };

    float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
    m_triggered = dist < m_threshold;

    // Update switch contact state
    // When triggered: switch is CLOSED (terminals connected)
    // When not triggered: switch is OPEN
    if (m_triggered) {
        // Closed switch: connect terminals (copy voltage from A to B)
        if (m_terminal_a_port && m_terminal_b_port) {
            if (const float* val_a = m_terminal_a_port->get_value<float>()) {
                m_terminal_b_port->set_value(*val_a);
            }
        }
    } else {
        // Open switch: terminals disconnected, set both to 0
        if (m_terminal_a_port) m_terminal_a_port->set_value(0.0f);
        if (m_terminal_b_port) m_terminal_b_port->set_value(0.0f);
    }
}

void LimitSwitch::serialize(nlohmann::json& out) const {
    out["triggered"] = m_triggered;
    out["threshold"] = m_threshold;
}

void LimitSwitch::deserialize(const nlohmann::json& in) {
    if (in.contains("threshold")) m_threshold = in["threshold"];
}

// Rotary Encoder
RotaryEncoder::RotaryEncoder() {
    m_min_value = 0.0f;
    m_max_value = 5.0f;
}

void RotaryEncoder::update(double dt) {
    // Use the angular velocity set by coupled motor (or simulated)
    // Update angle: theta = theta + omega * dt
    float delta_angle = m_angular_velocity * static_cast<float>(dt) * (180.0f / M_PI); // Convert rad/s to deg
    m_angle += delta_angle;

    // Keep angle in range [0, 360)
    while (m_angle >= 360.0f) {
        m_angle -= 360.0f;
        m_pulse_count++;
    }
    while (m_angle < 0.0f) {
        m_angle += 360.0f;
        m_pulse_count--;
    }

    // Update pulses based on resolution
    // Each pulse corresponds to (360 / resolution) degrees
    float pulses_per_revolution = m_resolution;
    float degrees_per_pulse = 360.0f / pulses_per_revolution;
    int total_pulses = static_cast<int>(m_angle / degrees_per_pulse);
}

void RotaryEncoder::serialize(nlohmann::json& out) const {
    out["angle"] = m_angle;
    out["pulse_count"] = m_pulse_count;
    out["resolution"] = m_resolution;
}

void RotaryEncoder::deserialize(const nlohmann::json& in) {
    if (in.contains("angle")) m_angle = in["angle"];
    if (in.contains("pulse_count")) m_pulse_count = in["pulse_count"];
    if (in.contains("resolution")) m_resolution = in["resolution"];
}

// Potentiometer
Potentiometer::Potentiometer() {
    m_min_value = 0.0f;
    m_max_value = 5.0f;

    // Create 3-terminal potentiometer ports
    m_v_plus_port = std::make_unique<Port>("V+", PortDomain::Electrical, PortDirection::Input);
    m_v_plus_port->set_value(0.0f);

    m_wiper_port = std::make_unique<Port>("WIPER", PortDomain::Analog, PortDirection::Output);
    m_wiper_port->set_value(0.0f);

    m_gnd_port = std::make_unique<Port>("GND", PortDomain::Electrical, PortDirection::Input);
    m_gnd_port->set_value(0.0f);
}

void Potentiometer::update(double dt) {
    // Simulate potentiometer reading from transform rotation
    // Using rotation around Z axis (yaw)

    // For demo, just use position.x as simulated angle input
    // In real system, this would read actual mechanical position

    float normalized_pos = transform().position.x / 10.0f; // Normalize input
    normalized_pos = std::max(0.0f, std::min(1.0f, normalized_pos));

    m_angle = m_min_angle + normalized_pos * (m_max_angle - m_min_angle);

    // Check if V+ and GND are connected for voltage divider
    float v_plus = default_max_voltage();  // Default supply voltage
    float v_gnd = 0.0f;

    if (m_v_plus_port && !m_v_plus_port->connections().empty()) {
        if (const float* val = m_v_plus_port->get_value<float>()) v_plus = *val;
    }
    if (m_gnd_port && !m_gnd_port->connections().empty()) {
        if (const float* val = m_gnd_port->get_value<float>()) v_gnd = *val;
    }

    // Voltage divider: V_wiper = (V+ - GND) * (angle / max_angle) + GND
    float voltage_ratio = m_angle / m_max_angle;
    m_voltage = (v_plus - v_gnd) * voltage_ratio + v_gnd;

    // Write wiper voltage to output port
    if (m_wiper_port) {
        m_wiper_port->set_value(m_voltage);
    }
}

void Potentiometer::serialize(nlohmann::json& out) const {
    out["voltage"] = m_voltage;
    out["angle"] = m_angle;
    out["min_angle"] = m_min_angle;
    out["max_angle"] = m_max_angle;
}

void Potentiometer::deserialize(const nlohmann::json& in) {
    if (in.contains("min_angle")) m_min_angle = in["min_angle"];
    if (in.contains("max_angle")) m_max_angle = in["max_angle"];
}

} // namespace mechatron
