/**
 * @file motor_pid_demo.cpp
 * @brief H-Bridge → DC Motor → Rotary Encoder → PID closed-loop control demo
 *
 * This demonstrates the complete mechatronic control loop:
 * 1. PID controller computes control output from setpoint and feedback
 * 2. H-Bridge converts control signal to motor voltage (with direction)
 * 3. DC Motor converts voltage to rotational motion
 * 4. Rotary Encoder measures motor position
 * 5. Encoder reading fed back to PID controller
 */

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <cmath>
#include <iostream>
#include <iomanip>

#include "actuators/Actuator.hpp"
#include "sensors/Sensor.hpp"
#include "electronics/CircuitSimulator.hpp"
#include "control/ControlAlgorithms.hpp"

using namespace mechatron;

// Simple demo application class
class MotorPIDDemo {
public:
    MotorPIDDemo() {
        // Create components
        m_pid = std::make_unique<PIDController>();
        m_hbridge = std::make_unique<HBridge>(12.0f); // 12V supply
        m_motor = std::make_unique<DCMotor>();
        m_encoder = std::make_unique<RotaryEncoder>();

        // Configure PID controller
        PIDController::Params pid_params;
        pid_params.kp = 0.15f;  // Low Kp for smooth approach
        pid_params.ki = 0.01f; // Very low Ki to prevent windup
        pid_params.kd = 0.05f;  // Higher Kd for damping
        pid_params.output_min = -1.0f;  // -100% (reverse)
        pid_params.output_max = 1.0f;   // +100% (forward)
        pid_params.integral_min = -100.0f; // Anti-windup clamping
        pid_params.integral_max = 100.0f;
        pid_params.dt = 0.001f;
        m_pid->set_params(pid_params);

        // Configure motor (faster motor for better control)
        m_motor->set_voltage_rating(12.0f);
        m_motor->set_no_load_speed(3000.0f);  // Faster motor (RPM)
        m_motor->set_stall_torque(0.5f);

        // Configure encoder
        m_encoder->set_resolution(360); // 360 PPR

        // Couple encoder to motor
        m_motor->attach_encoder(m_encoder.get());

        spdlog::info("Motor PID Demo initialized");
        spdlog::info("Target: {} degrees, PID: Kp={:.2f}, Ki={:.2f}, Kd={:.2f}",
                     m_target_angle, pid_params.kp, pid_params.ki, pid_params.kd);
    }

    void run(double duration_sec = 5.0) {
        const double dt = 0.001; // 1ms timestep
        const size_t steps = static_cast<size_t>(duration_sec / dt);

        spdlog::info("Running simulation for {:.1f} seconds...", duration_sec);
        spdlog::info("{:>8} {:>10} {:>10} {:>10} {:>10} {:>10}",
                     "Time(s)", "Setpoint", "Angle", "Error", "Control", "RPM");

        // Print header
        std::cout << std::fixed << std::setprecision(3);

        for (size_t i = 0; i < steps; i++) {
            double time = i * dt;

            // 1. Read feedback from encoder (use cumulative position for PID)
            // Total angle = full revolutions (pulse_count * 360) + current partial rotation
            float current_angle = m_encoder->get_pulse_count() * 360.0f + m_encoder->get_angle_degrees();

            // 2. PID controller computes control output
            float control_output = m_pid->compute(m_target_angle, current_angle);

            // 3. Convert control output to H-Bridge signals
            // control_output: -1.0 (full reverse) to +1.0 (full forward)
            float abs_control = std::abs(control_output);

            if (control_output > 0.01f) {
                // Forward
                m_hbridge->set_in1(true);
                m_hbridge->set_in2(false);
            } else if (control_output < -0.01f) {
                // Reverse
                m_hbridge->set_in1(false);
                m_hbridge->set_in2(true);
            } else {
                // Stop/brake
                m_hbridge->set_in1(false);
                m_hbridge->set_in2(false);
            }

            m_hbridge->set_enable(true);
            m_hbridge->set_pwm_duty(abs_control);

            // 4. Update H-Bridge (computes output voltage)
            m_hbridge->update(dt);

            // 5. Get H-Bridge output and set motor input
            // Pass signed voltage ratio to motor (positive = forward, negative = reverse)
            float motor_voltage_ratio = m_hbridge->get_output_voltage() / 12.0f;
            m_motor->set_input(motor_voltage_ratio);

            // 6. Update motor (computes angular velocity)
            m_motor->update(dt);

            // 7. Update encoder (reads angular velocity from motor)
            m_encoder->update(dt);

            // Print status every 100ms
            if (i % 100 == 0) {
                float error = m_target_angle - current_angle;
                spdlog::info("{:>8.3f} {:>10.2f} {:>10.2f} {:>10.2f} {:>10.3f} {:>10.1f}",
                             time, m_target_angle, current_angle, error,
                             control_output, m_motor->get_rpm());
            }

            // Check if settled (within 1 degree for 0.5 seconds)
            if (std::abs(m_target_angle - current_angle) < 1.0f) {
                m_settled_count++;
                if (m_settled_count > 500) { // 0.5 seconds
                    spdlog::info("Target reached! Settled at {:.2f} degrees", current_angle);
                    break;
                }
            } else {
                m_settled_count = 0;
            }
        }

        print_final_status();
    }

    void test_step_response() {
        spdlog::info("\n=== Step Response Test ===");

        // Test 1: Step to 90 degrees (forward)
        m_target_angle = 90.0f;
        m_pid->reset();
        run(3.0);

        // Test 2: Step to 180 degrees (forward)
        spdlog::info("\n=== New Target: 180 degrees ===");
        m_target_angle = 180.0f;
        m_pid->reset();
        run(3.0);

        // Test 3: Step to 270 degrees (continuing forward)
        // This demonstrates continuous rotation without "wraparound" issues
        spdlog::info("\n=== New Target: 270 degrees ===");
        m_target_angle = 270.0f;
        m_pid->reset();
        run(3.0);

        // Test 4: Step back to 180 degrees (reverse motion)
        // Now the motor must spin REVERSE to go from 270° back to 180°
        spdlog::info("\n=== New Target: 180 degrees (REVERSE) ===");
        m_target_angle = 180.0f;
        m_pid->reset();
        run(3.0);
    }

    void test_disturbance_rejection() {
        spdlog::info("\n=== Disturbance Rejection Test ===");
        spdlog::info("Target: 90 degrees, applying external load at t=1s");

        m_target_angle = 90.0f;
        m_pid->reset();

        const double dt = 0.001;
        bool disturbance_applied = false;

        for (int i = 0; i < 5000; i++) { // 5 seconds
            double time = i * dt;

            // Apply disturbance at t=1s (external load on motor)
            if (time >= 1.0 && time < 2.0 && !disturbance_applied) {
                spdlog::info("Applying external load torque!");
                // Simulate external load by reducing motor effective torque
                disturbance_applied = true;
            }

            // Use cumulative pulse count for continuous tracking
            float current_angle = m_encoder->get_pulse_count() * 360.0f + m_encoder->get_angle_degrees();
            float control_output = m_pid->compute(m_target_angle, current_angle);

            // H-Bridge control
            float abs_control = std::abs(control_output);
            if (control_output > 0.01f) {
                m_hbridge->set_in1(true);
                m_hbridge->set_in2(false);
            } else if (control_output < -0.01f) {
                m_hbridge->set_in1(false);
                m_hbridge->set_in2(true);
            } else {
                m_hbridge->set_in1(false);
                m_hbridge->set_in2(false);
            }
            m_hbridge->set_enable(true);
            m_hbridge->set_pwm_duty(abs_control);

            m_hbridge->update(dt);

            // Pass signed voltage ratio to motor (positive = forward, negative = reverse)
            float motor_input = m_hbridge->get_output_voltage() / 12.0f;

            // Apply disturbance by reducing effective input during 1-2s
            if (time >= 1.0 && time < 2.0) {
                motor_input *= 0.3f; // External load reduces motor effectiveness
            }

            m_motor->set_input(motor_input);
            m_motor->update(dt);
            m_encoder->update(dt);

            if (i % 100 == 0) {
                spdlog::info("t={:.2f}s | Angle={:.2f}° | Control={:.3f} | RPM={:.1f} {}",
                             time, current_angle, control_output, m_motor->get_rpm(),
                             (time >= 1.0 && time < 2.0) ? "[LOAD]" : "");
            }
        }

        spdlog::info("Disturbance rejection test complete");
    }

private:
    void print_final_status() {
        spdlog::info("\n=== Final Status ===");
        spdlog::info("Motor RPM: {:.1f}", m_motor->get_rpm());
        spdlog::info("Motor Torque: {:.4f} Nm", m_motor->get_torque());
        spdlog::info("Encoder Angle: {:.2f} degrees", m_encoder->get_angle_degrees());
        spdlog::info("Encoder Pulse Count: {}", m_encoder->get_pulse_count());
        spdlog::info("H-Bridge Direction: {}", m_hbridge->get_direction());
        spdlog::info("H-Bridge Output Voltage: {:.2f} V", m_hbridge->get_output_voltage());
        spdlog::info("PID Integral: {:.4f}", m_pid->get_integral());
    }

    std::unique_ptr<PIDController> m_pid;
    std::unique_ptr<HBridge> m_hbridge;
    std::unique_ptr<DCMotor> m_motor;
    std::unique_ptr<RotaryEncoder> m_encoder;

    float m_target_angle = 90.0f;
    size_t m_settled_count = 0;
};

int main() {
    // Setup logging
    auto console = spdlog::stdout_color_mt("console");
    spdlog::set_default_logger(console);
    spdlog::set_level(spdlog::level::info);

    spdlog::info("========================================");
    spdlog::info("  H-Bridge → DC Motor → Encoder → PID");
    spdlog::info("  Closed-Loop Control Demo");
    spdlog::info("========================================\n");

    try {
        MotorPIDDemo demo;

        // Run step response tests
        demo.test_step_response();

        // Run disturbance rejection test
        demo.test_disturbance_rejection();

        spdlog::info("\n========================================");
        spdlog::info("  Demo completed successfully!");
        spdlog::info("========================================");

        return 0;

    } catch (const std::exception& e) {
        spdlog::error("Error: {}", e.what());
        return 1;
    }
}
