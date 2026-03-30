#pragma once

#include "core/Types.hpp"
#include "core/Component.hpp"
#include "physics/PhysicsWorld.hpp"
#include <string>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mechatron {

// Base actuator class
class Actuator : public Component {
public:
    virtual ~Actuator() = default;

    // Apply actuator force/torque to physics body
    virtual void apply_to_physics(PhysicsBody* body) = 0;

    // Control input
    void set_input(float value) { m_input = value; }
    float get_input() const { return m_input; }

    // Enable/disable
    void set_enabled(bool enabled) { m_enabled = enabled; }
    bool is_enabled() const { return m_enabled; }

protected:
    float m_input = 0.0f;      // Control input (0-1 or voltage)
    bool m_enabled = true;
    PhysicsBody* m_attached_body = nullptr;
};

// Solenoid Actuator - electromechanical linear actuator
class SolenoidActuator : public Actuator {
public:
    SolenoidActuator();
    ~SolenoidActuator() override = default;

    std::string_view plugin_type() const override { return "mech_machine_elements"; }
    std::string_view component_type() const override { return "solenoid_actuator"; }
    std::string_view category() const override { return "actuator"; }

    void update(double dt) override;
    void serialize(nlohmann::json& out) const override;
    void deserialize(const nlohmann::json& in) override;

    void apply_to_physics(PhysicsBody* body) override;

    // Electrical parameters
    void set_resistance(float ohms) { m_resistance = ohms; }
    void set_inductance(float henries) { m_inductance = henries; }
    void set_turns(int n) { m_turns = n; }

    // Mechanical parameters
    void set_stroke(float mm) { m_stroke = mm / 1000.0f; } // Convert mm to m
    void set_plunger_mass(float grams) { m_plunger_mass = grams / 1000.0f; }
    void set_spring_constant(float k) { m_spring_k = k; }

    // Get current plunger position (0 = retracted, 1 = extended)
    float get_position() const { return m_plunger_position; }

    // Attach to physics body
    void attach_body(PhysicsBody* body) { m_attached_body = body; }

private:
    // Electrical
    float m_resistance = 12.0f;    // Ohms
    float m_inductance = 0.5f;     // Henry
    int m_turns = 500;
    float m_current = 0.0f;

    // Mechanical
    float m_stroke = 0.01f;        // 10mm stroke (meters)
    float m_plunger_mass = 0.005f; // 5g (kg)
    float m_spring_k = 50.0f;      // Spring constant (N/m)
    float m_plunger_position = 0.0f; // 0-1 normalized
    float m_plunger_velocity = 0.0f;

    // Calculate electromagnetic force
    float calculate_force(float voltage) const;
};

// Forward declaration
class RotaryEncoder;

// DC Motor - rotational electric motor
class DCMotor : public Actuator {
public:
    DCMotor();
    ~DCMotor() override = default;

    std::string_view plugin_type() const override { return "mech_machine_elements"; }
    std::string_view component_type() const override { return "dc_motor"; }
    std::string_view category() const override { return "actuator"; }

    void update(double dt) override;
    void serialize(nlohmann::json& out) const override;
    void deserialize(const nlohmann::json& in) override;

    void apply_to_physics(PhysicsBody* body) override;

    // Motor parameters
    void set_voltage_rating(float volts) { m_voltage_rating = volts; }
    void set_no_load_speed(float rpm) { m_no_load_rpm = rpm; }
    void set_stall_torque(float nm) { m_stall_torque = nm; }

    // Get current state
    float get_angular_velocity() const { return m_angular_velocity; } // rad/s
    float get_rpm() const { return m_angular_velocity * 60.0f / (2.0f * M_PI); }
    float get_torque() const { return m_output_torque; }

    void attach_body(PhysicsBody* body) { m_attached_body = body; }

    // Encoder coupling
    void attach_encoder(RotaryEncoder* encoder) { m_encoder = encoder; }
    RotaryEncoder* get_encoder() const { return m_encoder; }

private:
    // Motor parameters
    float m_voltage_rating = 12.0f;
    float m_no_load_rpm = 3000.0f;
    float m_stall_torque = 0.5f;

    // Derived constants
    float m_kv = 0.0f;   // Velocity constant (rad/s/V)
    float m_kt = 0.0f;   // Torque constant (Nm/A)

    // State
    float m_angular_velocity = 0.0f; // rad/s
    float m_angle = 0.0f;
    float m_output_torque = 0.0f;

    // Encoder for position feedback
    RotaryEncoder* m_encoder = nullptr;

    // Calculate motor torque
    float calculate_torque(float voltage, float angular_vel) const;
};

// Servo Motor - position-controlled motor
class ServoMotor : public Actuator {
public:
    ServoMotor();
    ~ServoMotor() override = default;

    std::string_view plugin_type() const override { return "mech_machine_elements"; }
    std::string_view component_type() const override { return "servo_motor"; }
    std::string_view category() const override { return "actuator"; }

    void update(double dt) override;
    void serialize(nlohmann::json& out) const override;
    void deserialize(const nlohmann::json& in) override;

    void apply_to_physics(PhysicsBody* body) override;

    // Servo parameters
    void set_angle_range(float min_deg, float max_deg) {
        m_min_angle = min_deg;
        m_max_angle = max_deg;
    }

    // Set target angle (0-180 degrees for typical servo)
    void set_target_angle(float degrees) {
        m_target_angle = (std::max)(m_min_angle, (std::min)(m_max_angle, degrees));
    }

    float get_current_angle() const { return m_current_angle; }

    void attach_body(PhysicsBody* body) { m_attached_body = body; }

private:
    float m_min_angle = 0.0f;
    float m_max_angle = 180.0f;
    float m_target_angle = 90.0f;
    float m_current_angle = 90.0f;
    float m_max_speed = 300.0f; // degrees/second
    float m_max_torque = 2.0f;  // Nm
    float m_output_torque = 0.0f;
};

} // namespace mechatron
