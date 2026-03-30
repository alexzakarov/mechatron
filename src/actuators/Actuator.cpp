#include "Actuator.hpp"
#include "sensors/Sensor.hpp"
#include <spdlog/spdlog.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mechatron {

// Solenoid Actuator
SolenoidActuator::SolenoidActuator() {
    // Calculate motor constant from parameters
    // KV = no_load_rpm / voltage_rating * (2*pi/60)
    // KT = stall_torque / (voltage_rating / resistance)
}

float SolenoidActuator::calculate_force(float voltage) const {
    if (!m_enabled || voltage <= 0.0f) return 0.0f;

    // Solenoid force model: F = (N*I)^2 * mu0 * A / (2 * g^2)
    // Simplified: F proportional to current^2

    // Current: I = V / R (steady state, neglecting inductance for simplicity)
    float current = voltage / m_resistance;

    // Force proportional to (turns * current)^2
    float force_constant = 1e-5f; // Magnetic constant (simplified)
    float force = force_constant * m_turns * m_turns * current * current;

    // Force decreases with plunger position (air gap increases)
    // Model: F = F0 / (1 + position * factor)
    float gap_factor = 1.0f + m_plunger_position * 10.0f;
    force = force / gap_factor;

    return force;
}

void SolenoidActuator::update(double dt) {
    if (!m_enabled) return;

    // Calculate plunger dynamics
    float voltage = m_input * 5.0f; // Assume 0-5V control
    float force = calculate_force(voltage);

    // Spring force: F = -k * x
    float spring_force = -m_spring_k * m_plunger_position * m_stroke;

    // Damping (friction)
    float damping = -0.5f * m_plunger_velocity;

    // Total force
    float total_force = force + spring_force + damping;

    // Acceleration: a = F / m
    float acceleration = total_force / m_plunger_mass;

    // Integrate: v = v + a * dt
    m_plunger_velocity += acceleration * dt;

    // Integrate: x = x + v * dt
    m_plunger_position += m_plunger_velocity * dt / m_stroke;

    // Clamp position [0, 1]
    m_plunger_position = std::max(0.0f, std::min(1.0f, m_plunger_position));

    // Stop velocity if at limits
    if (m_plunger_position <= 0.0f || m_plunger_position >= 1.0f) {
        if ((m_plunger_position <= 0.0f && m_plunger_velocity < 0.0f) ||
            (m_plunger_position >= 1.0f && m_plunger_velocity > 0.0f)) {
            m_plunger_velocity = 0.0f;
        }
    }
}

void SolenoidActuator::apply_to_physics(PhysicsBody* body) {
    if (!body || !m_enabled) return;

    // Apply linear force along local Y axis (upward push)
    float force_magnitude = calculate_force(m_input * 5.0f);
    body->force_accumulator.y += force_magnitude;
}

void SolenoidActuator::serialize(nlohmann::json& out) const {
    out["position"] = m_plunger_position;
    out["current"] = m_current;
    out["input"] = m_input;
}

void SolenoidActuator::deserialize(const nlohmann::json& in) {
    if (in.contains("resistance")) m_resistance = in["resistance"];
    if (in.contains("inductance")) m_inductance = in["inductance"];
    if (in.contains("turns")) m_turns = in["turns"];
}

// DC Motor
DCMotor::DCMotor() {
    // Calculate KV and KT from rating parameters
    m_kv = m_no_load_rpm / m_voltage_rating * (2.0f * M_PI / 60.0f); // rad/s/V
    m_kt = m_stall_torque / (m_voltage_rating / 1.0f); // Simplified (assume 1 ohm)
}

float DCMotor::calculate_torque(float voltage, float angular_vel) const {
    if (!m_enabled) return 0.0f;

    // Back EMF: Vemf = KV * omega
    float back_emf = angular_vel / m_kv;

    // Voltage across resistance: V - Vemf
    float effective_voltage = voltage - back_emf;

    // Current: I = V / R (simplified, assume R = 1 ohm)
    float current = effective_voltage;

    // Torque: T = KT * I
    float torque = m_kt * current;

    return torque;
}

void DCMotor::update(double dt) {
    if (!m_enabled) return;

    float voltage = m_input * m_voltage_rating;
    float torque = calculate_torque(voltage, m_angular_velocity);

    // Rotational dynamics with proper inertia and damping
    // omega = omega + (T / J) * dt
    float inertia = 0.001f; // kg*m^2 (balanced for control)
    float angular_accel = torque / inertia;

    m_angular_velocity += angular_accel * dt;

    // Apply damping (viscous friction + load)
    float damping_factor = 0.95f; // Higher damping for stability
    m_angular_velocity *= damping_factor;

    // Clamp to realistic max speed
    float max_omega = m_no_load_rpm * (2.0f * M_PI / 60.0f); // rad/s
    m_angular_velocity = std::max(-max_omega, std::min(max_omega, m_angular_velocity));

    // Update angle
    m_angle += m_angular_velocity * dt;

    m_output_torque = torque;

    // Update attached encoder with current angular velocity
    if (m_encoder) {
        m_encoder->set_angular_velocity(m_angular_velocity);
    }
}

void DCMotor::apply_to_physics(PhysicsBody* body) {
    if (!body || !m_enabled) return;

    // Apply torque around Y axis (vertical rotation)
    m_output_torque = calculate_torque(m_input * m_voltage_rating, m_angular_velocity);
    body->torque_accumulator.y += m_output_torque;
}

void DCMotor::serialize(nlohmann::json& out) const {
    out["angular_velocity"] = m_angular_velocity;
    out["rpm"] = get_rpm();
    out["torque"] = m_output_torque;
}

void DCMotor::deserialize(const nlohmann::json& in) {
    if (in.contains("voltage_rating")) m_voltage_rating = in["voltage_rating"];
    if (in.contains("no_load_rpm")) m_no_load_rpm = in["no_load_rpm"];
    if (in.contains("stall_torque")) m_stall_torque = in["stall_torque"];

    // Recalculate constants
    m_kv = m_no_load_rpm / m_voltage_rating * (2.0f * M_PI / 60.0f);
    m_kt = m_stall_torque / m_voltage_rating;
}

// Servo Motor
ServoMotor::ServoMotor() = default;

void ServoMotor::update(double dt) {
    if (!m_enabled) return;

    // Calculate angle difference
    float error = m_target_angle - m_current_angle;

    // Simple P controller (proportional)
    float speed = error * 10.0f; // Gain

    // Clamp speed
    speed = std::max(-m_max_speed, std::min(m_max_speed, speed));

    // Update angle
    m_current_angle += speed * dt;

    // Clamp to range
    m_current_angle = std::max(m_min_angle, std::min(m_max_angle, m_current_angle));
}

void ServoMotor::apply_to_physics(PhysicsBody* body) {
    if (!body || !m_enabled) return;

    // Servo applies torque to reach target position
    float error = m_target_angle - m_current_angle;
    float torque = error * m_max_torque / 90.0f;

    m_output_torque = torque;
    body->torque_accumulator.y += torque;
}

void ServoMotor::serialize(nlohmann::json& out) const {
    out["target_angle"] = m_target_angle;
    out["current_angle"] = m_current_angle;
}

void ServoMotor::deserialize(const nlohmann::json& in) {
    if (in.contains("min_angle")) m_min_angle = in["min_angle"];
    if (in.contains("max_angle")) m_max_angle = in["max_angle"];
    if (in.contains("target_angle")) m_target_angle = in["target_angle"];
}

} // namespace mechatron
