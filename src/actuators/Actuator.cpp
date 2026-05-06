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
    create_actuator_ports();  // V+, GND ports
    // Calculate motor constants from parameters
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

    // Check if circuit is complete: BOTH V+ and GND must be connected
    bool v_plus_connected = (m_power_port && !m_power_port->connections().empty());
    bool gnd_connected = (m_gnd_port && !m_gnd_port->connections().empty());

    if (!v_plus_connected || !gnd_connected) {
        // Open circuit - no current can flow, solenoid de-energizes
        m_plunger_velocity *= 0.9f;

        // Spring returns to retracted position
        float spring_force = -m_spring_k * m_plunger_position * m_stroke;
        float damping = -0.5f * m_plunger_velocity;
        float acceleration = (spring_force + damping) / m_plunger_mass;
        m_plunger_velocity += acceleration * dt;
        m_plunger_position += m_plunger_velocity * dt / m_stroke;
        m_plunger_position = std::max(0.0f, std::min(1.0f, m_plunger_position));
        if (m_plunger_position <= 0.0f) m_plunger_velocity = 0.0f;

        if (m_power_port) m_power_port->set_value(0.0f);
        return;
    }

    // Read voltage difference: V+ - GND
    float v_plus = 0.0f, v_gnd = 0.0f;
    if (const float* val = m_power_port->get_value<float>()) v_plus = *val;
    if (const float* val = m_gnd_port->get_value<float>()) v_gnd = *val;

    float voltage = v_plus - v_gnd;

    if (voltage == 0.0f) {
        m_plunger_velocity *= 0.9f;
        return;
    }

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
    create_actuator_ports(false);  // V+, GND only (no signal pin)

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

    // Check if circuit is complete: BOTH V+ and GND must be connected
    // for current to flow. An unconnected pin means an open circuit.
    bool v_plus_connected = (m_power_port && !m_power_port->connections().empty());
    bool gnd_connected = (m_gnd_port && !m_gnd_port->connections().empty());

    if (!v_plus_connected || !gnd_connected) {
        // Open circuit - no current can flow, motor coasts to stop
        m_angular_velocity *= 0.95f;
        m_output_torque = 0.0f;

        // Update port values with 0 (no current draw)
        if (m_power_port) m_power_port->set_value(0.0f);
        return;
    }

    // Read voltage difference: V+ - GND (actual potential across motor)
    float v_plus = 0.0f, v_gnd = 0.0f;
    if (const float* val = m_power_port->get_value<float>()) v_plus = *val;
    if (const float* val = m_gnd_port->get_value<float>()) v_gnd = *val;

    float voltage = v_plus - v_gnd;

    // If no voltage difference, motor coasts
    if (voltage == 0.0f) {
        m_angular_velocity *= 0.95f;
        m_output_torque = 0.0f;
        return;
    }

    float torque = calculate_torque(voltage, m_angular_velocity);

    // Rotational dynamics with proper inertia and damping
    float inertia = 0.001f; // kg*m^2
    float angular_accel = torque / inertia;

    m_angular_velocity += angular_accel * dt;

    // Apply damping
    float damping_factor = 0.95f;
    m_angular_velocity *= damping_factor;

    // Clamp to realistic max speed
    float max_omega = m_no_load_rpm * (2.0f * M_PI / 60.0f); // rad/s
    m_angular_velocity = std::max(-max_omega, std::min(max_omega, m_angular_velocity));

    // Update angle
    m_angle += m_angular_velocity * dt;

    m_output_torque = torque;

    // Update port values for circuit feedback
    if (m_power_port) {
        m_power_port->set_value(v_plus);
    }

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
ServoMotor::ServoMotor() {
    create_actuator_ports(true);  // V+, GND, SIG (signal for position control)
}

void ServoMotor::update(double dt) {
    if (!m_enabled) return;

    // Check if circuit is complete: BOTH V+ and GND must be connected
    bool v_plus_connected = (m_power_port && !m_power_port->connections().empty());
    bool gnd_connected = (m_gnd_port && !m_gnd_port->connections().empty());

    if (!v_plus_connected || !gnd_connected) {
        // Open circuit - no power, servo holds position
        m_output_torque = 0.0f;
        if (m_power_port) m_power_port->set_value(0.0f);
        return;
    }

    // Read voltage difference: V+ - GND
    float v_plus = 0.0f, v_gnd = 0.0f;
    if (const float* val = m_power_port->get_value<float>()) v_plus = *val;
    if (const float* val = m_gnd_port->get_value<float>()) v_gnd = *val;
    float voltage = v_plus - v_gnd;

    if (voltage == 0.0f) {
        m_output_torque = 0.0f;
        return;
    }

    // Read target angle from SIG port (if connected)
    // SIG port maps angle: 0V = min_angle, 5V = max_angle
    if (m_signal_port && !m_signal_port->connections().empty()) {
        if (const float* sig_val = m_signal_port->get_value<float>()) {
            // Map 0-5V signal to min-max angle range
            float sig_normalized = std::max(0.0f, std::min(1.0f, *sig_val / 5.0f));
            m_target_angle = m_min_angle + sig_normalized * (m_max_angle - m_min_angle);
        }
    }

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
