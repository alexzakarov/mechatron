#pragma once

#include "core/Types.hpp"
#include "core/Component.hpp"
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mechatron {

// Base sensor class
class Sensor : public Component {
public:
    virtual ~Sensor() = default;

    // Read current sensor value
    virtual float read() const = 0;

    // Sensor-specific parameters
    void set_min_value(float min) { m_min_value = min; }
    void set_max_value(float max) { m_max_value = max; }
    void set_noise_level(float noise) { m_noise_level = noise; }

    float min_value() const { return m_min_value; }
    float max_value() const { return m_max_value; }

protected:
    float m_min_value = 0.0f;
    float m_max_value = 5.0f;   // Default 0-5V range
    float m_noise_level = 0.0f; // Add noise to readings
};

// Proximity Sensor - measures distance to objects
class ProximitySensor : public Sensor {
public:
    ProximitySensor();
    ~ProximitySensor() override = default;

    std::string_view plugin_type() const override { return "mech_machine_elements"; }
    std::string_view component_type() const override { return "proximity_sensor"; }
    std::string_view category() const override { return "sensor"; }

    void update(double dt) override;
    void serialize(nlohmann::json& out) const override;
    void deserialize(const nlohmann::json& in) override;

    float read() const override {
        // Convert distance to voltage (0-5V)
        // Closer = higher voltage, farther = lower voltage
        float ratio = 1.0f - (m_distance / m_max_range);
        return ratio * m_max_value;
    }

    // Get actual distance in meters
    float get_distance() const { return m_distance; }

    // Configuration
    void set_max_range(float range) { m_max_range = range; }
    void set_fov(float fov_degrees) { m_fov = fov_degrees; }
    void set_target_position(Vec3 pos) { m_target_position = pos; }

    // Port access for circuit integration
    std::vector<Port*> get_ports() override {
        std::vector<Port*> result;
        if (m_v_plus_port) result.push_back(m_v_plus_port.get());
        if (m_output_port) result.push_back(m_output_port.get());
        if (m_gnd_port) result.push_back(m_gnd_port.get());
        return result;
    }

private:
    float m_distance = 0.0f;
    float m_max_range = 5.0f;     // Maximum detection range (meters)
    float m_fov = 30.0f;          // Field of view (degrees)
    Vec3 m_target_position{0, 0, 0};    // Position to measure distance to

    // Ports for circuit integration
    std::unique_ptr<Port> m_v_plus_port;   // V+ input
    std::unique_ptr<Port> m_output_port;   // Output signal (distance → voltage)
    std::unique_ptr<Port> m_gnd_port;      // GND input
};

// Limit Switch - binary contact sensor
class LimitSwitch : public Sensor {
public:
    LimitSwitch();
    ~LimitSwitch() override = default;

    std::string_view plugin_type() const override { return "mech_machine_elements"; }
    std::string_view component_type() const override { return "limit_switch"; }
    std::string_view category() const override { return "sensor"; }

    void update(double dt) override;
    void serialize(nlohmann::json& out) const override;
    void deserialize(const nlohmann::json& in) override;

    float read() const override { return m_triggered ? 5.0f : 0.0f; }
    bool is_triggered() const { return m_triggered; }

    void set_trigger_position(Vec3 pos) { m_trigger_position = pos; }
    void set_trigger_threshold(float threshold) { m_threshold = threshold; }

    // Port access for circuit integration (2-terminal switch contact)
    std::vector<Port*> get_ports() override {
        std::vector<Port*> result;
        if (m_terminal_a_port) result.push_back(m_terminal_a_port.get());
        if (m_terminal_b_port) result.push_back(m_terminal_b_port.get());
        return result;
    }

private:
    bool m_triggered = false;
    Vec3 m_trigger_position{0, 0, 0};
    float m_threshold = 0.1f;    // Distance threshold for triggering

    // Ports for circuit integration (2-terminal switch contact)
    std::unique_ptr<Port> m_terminal_a_port;  // Switch terminal A
    std::unique_ptr<Port> m_terminal_b_port;  // Switch terminal B
};

// Rotary Encoder - measures angular position
class RotaryEncoder : public Sensor {
public:
    RotaryEncoder();
    ~RotaryEncoder() override = default;

    std::string_view plugin_type() const override { return "mech_machine_elements"; }
    std::string_view component_type() const override { return "rotary_encoder"; }
    std::string_view category() const override { return "sensor"; }

    void update(double dt) override;
    void serialize(nlohmann::json& out) const override;
    void deserialize(const nlohmann::json& in) override;

    float read() const override { return m_angle; }

    void set_resolution(int ppr) { m_resolution = ppr; } // Pulses per revolution
    int get_pulse_count() const { return m_pulse_count; }
    void reset_count() { m_pulse_count = 0; m_angle = 0.0f; }

    // For coupling with DC motor
    void set_angular_velocity(float rad_s) { m_angular_velocity = rad_s; }
    float get_angular_velocity() const { return m_angular_velocity; }

    // Get angle in degrees
    float get_angle_degrees() const { return m_angle; }
    // Get angle in radians
    float get_angle_radians() const { return m_angle * (M_PI / 180.0f); }

private:
    float m_angle = 0.0f;
    int m_pulse_count = 0;
    int m_resolution = 360;     // Default 360 PPR (1 degree per pulse)
    float m_angular_velocity = 0.0f; // rad/s - for coupling with motor
};

// Potentiometer - angular position sensor
class Potentiometer : public Sensor {
public:
    Potentiometer();
    ~Potentiometer() override = default;

    std::string_view plugin_type() const override { return "mech_machine_elements"; }
    std::string_view component_type() const override { return "potentiometer"; }
    std::string_view category() const override { return "sensor"; }

    void update(double dt) override;
    void serialize(nlohmann::json& out) const override;
    void deserialize(const nlohmann::json& in) override;

    float read() const override { return m_voltage; }

    // Get angle in degrees (0-270 for typical potentiometer)
    float get_angle() const { return m_angle; }

    void set_angle_range(float min_deg, float max_deg) {
        m_min_angle = min_deg;
        m_max_angle = max_deg;
    }

    // Port access for circuit integration
    std::vector<Port*> get_ports() override {
        std::vector<Port*> result;
        if (m_v_plus_port) result.push_back(m_v_plus_port.get());
        if (m_wiper_port) result.push_back(m_wiper_port.get());
        if (m_gnd_port) result.push_back(m_gnd_port.get());
        return result;
    }

private:
    float m_voltage = 0.0f;
    float m_angle = 0.0f;
    float m_min_angle = 0.0f;
    float m_max_angle = 270.0f;

    // Ports for circuit integration (3-terminal potentiometer)
    std::unique_ptr<Port> m_v_plus_port;  // V+ input
    std::unique_ptr<Port> m_wiper_port;   // Wiper output (divided voltage)
    std::unique_ptr<Port> m_gnd_port;     // GND input
};

} // namespace mechatron
