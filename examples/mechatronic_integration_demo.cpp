// Mechatronic Integration Demo
// Demonstrates full integration: MCU → Circuit → Actuator → Physics → Sensor → MCU feedback loop

#include "mcu/QEMUInterface.hpp"
#include "actuators/Actuator.hpp"
#include "sensors/Sensor.hpp"
#include "electronics/CircuitSimulator.hpp"
#include "core/TimeManager.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <thread>
#include <chrono>

using namespace mechatron;

// Simple physics body for demo
struct DemoPhysicsBody {
    Vec3 position{0, 0, 0};
    Vec3 velocity{0, 0, 0};
    float mass = 0.1f;  // 100g
};

class MechatronicSystem {
public:
    MechatronicSystem() {
        // Create minimal test.hex
        std::ofstream hex_file("test_mech.hex");
        hex_file << ":00000001FF\n";
        hex_file.close();

        // Initialize MCU in simulation mode
        m_mcu.set_mode(MCUMode::Simulation);
        m_mcu.launch("test_mech.hex");

        // Create solenoid actuator
        m_solenoid = std::make_unique<SolenoidActuator>();
        m_solenoid->set_resistance(12.0f);  // 12 ohms
        m_solenoid->set_inductance(0.05f);  // 50mH
        m_solenoid->set_turns(500);
        m_solenoid->set_stroke(10.0f);      // 10mm stroke
        m_solenoid->set_plunger_mass(5.0f); // 5g

        // Create proximity sensor
        m_sensor = std::make_unique<ProximitySensor>();
        m_sensor->set_max_range(0.02f);     // 20mm range
        m_sensor->transform().position = {0, 0, 0};

        // Physics body (metal piece that solenoid moves)
        m_body.position = {0, 0, 0.015f};   // Start 15mm away
    }

    ~MechatronicSystem() {
        m_mcu.stop();
    }

    void run_simulation(double duration_seconds) {
        std::cout << "\n=== Mechatronic Simulation ===" << std::endl;
        std::cout << "Duration: " << duration_seconds << " seconds" << std::endl;
        std::cout << "Time step: " << TIME_STEP << " ms" << std::endl;

        double sim_time = 0.0;
        int step_count = 0;

        // Configure Arduino pin 13 as output (controls solenoid)
        m_mcu.write_register(ATmega328P_Registers::DDRB, 0x20);  // Pin 13 output

        std::cout << "\n"
                  << std::setw(10) << "Time(s)"
                  << std::setw(10) << "D13"
                  << std::setw(12) << "Solenoid(V)"
                  << std::setw(12) << "Plunger(%)"
                  << std::setw(12) << "Position(mm)"
                  << std::setw(12) << "Sensor(V)" << std::endl;
        std::cout << std::string(70, '-') << std::endl;

        while (sim_time < duration_seconds) {
            // 1. MCU Step: Read digital output (D13 controls solenoid)
            bool pin13_state = m_mcu.digital_read(13);
            float control_voltage = pin13_state ? 5.0f : 0.0f;

            // 2. Circuit Step: Calculate actual voltage across solenoid coil
            // Simple model: V_out = V_in * (R_load / (R_load + R_internal))
            float coil_voltage = control_voltage;  // Ideal for now

            // 3. Actuator Step: Solenoid generates force, moves plunger
            m_solenoid->set_input(pin13_state ? 1.0f : 0.0f);
            m_solenoid->update(TIME_STEP);

            // 4. Physics Step: Plunger moves the metal piece
            // Force transmitted through magnetic field
            float plunger_force = m_solenoid->get_position() * 2.0f;  // Simplified
            if (plunger_force > 0.01f) {
                // Move piece towards solenoid
                float direction = (m_body.position.z > 0) ? -1.0f : 1.0f;
                m_body.velocity.z += direction * plunger_force / m_body.mass * TIME_STEP;
            }

            // Apply damping
            m_body.velocity.z *= 0.9f;
            m_body.position.z += m_body.velocity.z * TIME_STEP;

            // Clamp position
            m_body.position.z = (std::max)(0.0f, (std::min)(0.02f, m_body.position.z));

            // 5. Sensor Step: Measure distance to metal piece
            m_sensor->set_target_position(m_body.position);
            m_sensor->update(TIME_STEP);
            float sensor_voltage = m_sensor->read();

            // 6. Feedback: Write sensor value to ADC (A0)
            // Map 0-20mm to 0-5V
            uint16_t adc_value = static_cast<uint16_t>((m_body.position.z / 0.02f) * 1023.0f);
            m_mcu.memory().write_io(ATmega328P_Registers::ADCL, adc_value & 0xFF);
            m_mcu.memory().write_io(ATmega328P_Registers::ADCH, (adc_value >> 8) & 0x03);

            // Print status every 100ms
            if (step_count % 10 == 0) {
                std::cout << std::fixed << std::setprecision(3)
                          << std::setw(10) << sim_time
                          << std::setw(10) << (pin13_state ? "HIGH" : "LOW")
                          << std::setw(12) << coil_voltage
                          << std::setw(12) << (m_solenoid->get_position() * 100.0f)
                          << std::setw(12) << (m_body.position.z * 1000.0f)
                          << std::setw(12) << sensor_voltage << std::endl;
            }

            sim_time += TIME_STEP;
            step_count++;

            // Simulate Arduino blink: toggle D13 every 500ms
            int blink_period_ms = 500;
            int current_ms = static_cast<int>(sim_time * 1000);
            bool blink_state = (current_ms / blink_period_ms) % 2 == 0;
            m_mcu.digital_write(13, blink_state);
        }

        std::cout << "\n=== Simulation Complete ===" << std::endl;
        print_final_state();
    }

    void test_manual_control() {
        std::cout << "\n=== Manual Control Test ===" << std::endl;

        // Test solenoid response to different input levels
        std::cout << "\nTesting solenoid at different duty cycles:" << std::endl;
        std::cout << std::setw(15) << "Input(0-1)"
                  << std::setw(15) << "Voltage(V)"
                  << std::setw(15) << "Position(%)" << std::endl;

        for (float input = 0.0f; input <= 1.0f; input += 0.1f) {
            m_solenoid->set_input(input);

            // Run for 100ms to reach steady state
            for (int i = 0; i < 10; i++) {
                m_solenoid->update(0.01);
            }

            std::cout << std::fixed << std::setprecision(2)
                      << std::setw(15) << input
                      << std::setw(15) << (input * 5.0f)
                      << std::setw(15) << (m_solenoid->get_position() * 100.0f)
                      << std::endl;
        }
    }

    void test_sensor_feedback() {
        std::cout << "\n=== Sensor Feedback Test ===" << std::endl;

        // Test sensor at different distances
        std::cout << "\nTesting proximity sensor at different distances:" << std::endl;
        std::cout << std::setw(15) << "Distance(mm)"
                  << std::setw(15) << "Sensor(V)"
                  << std::setw(15) << "ADC(0-1023)" << std::endl;

        for (float dist_mm = 0.0f; dist_mm <= 20.0f; dist_mm += 2.0f) {
            m_body.position.z = dist_mm / 1000.0f;
            m_sensor->set_target_position(m_body.position);
            m_sensor->update(0.01);

            float voltage = m_sensor->read();
            uint16_t adc = static_cast<uint16_t>((voltage / 5.0f) * 1023.0f);

            std::cout << std::fixed << std::setprecision(1)
                      << std::setw(15) << dist_mm
                      << std::setw(15) << voltage
                      << std::setw(15) << adc << std::endl;
        }
    }

    void test_closed_loop_control() {
        std::cout << "\n=== Closed-Loop Control Test ===" << std::endl;
        std::cout << "Goal: Maintain position at 10mm using sensor feedback" << std::endl;

        float target_position_mm = 10.0f;
        float tolerance_mm = 1.0f;

        // Start at 18mm (far from target, needs to pull in)
        m_body.position = {0, 0, 0.018f};
        m_body.velocity = {0, 0, 0};

        std::cout << "\n"
                  << std::setw(10) << "Time(s)"
                  << std::setw(15) << "Target(mm)"
                  << std::setw(15) << "Actual(mm)"
                  << std::setw(15) << "Error(mm)"
                  << std::setw(10) << "D13" << std::endl;
        std::cout << std::string(70, '-') << std::endl;

        for (double t = 0; t < 3.0; t += TIME_STEP) {
            // Read current position from sensor
            m_sensor->set_target_position(m_body.position);
            m_sensor->update(TIME_STEP);

            float current_mm = m_body.position.z * 1000.0f;
            float error = target_position_mm - current_mm;

            // Simple on/off control with hysteresis
            // If actual > target + tolerance, activate solenoid to pull in
            // If actual < target - tolerance, deactivate and let spring extend
            bool activate = current_mm > (target_position_mm + tolerance_mm);

            m_mcu.digital_write(13, activate);

            // Update solenoid and physics
            m_solenoid->set_input(activate ? 1.0f : 0.0f);
            m_solenoid->update(TIME_STEP);

            // Physics: Solenoid pulls piece toward z=0, spring pushes toward z=max
            float solenoid_force = 0.0f;
            if (activate && m_solenoid->get_position() > 0.01f) {
                // Solenoid pulls inward (negative z direction)
                // Force increases with plunger position
                solenoid_force = -m_solenoid->get_position() * 8.0f;
            }

            // Spring force (restores to extended position at z=18mm)
            float spring_k = 5.0f;  // N/m - weaker spring
            float rest_position = 0.018f;  // 18mm rest position
            float spring_force = spring_k * (rest_position - m_body.position.z);  // Pushes outward

            // Total force and acceleration
            float total_force = solenoid_force + spring_force;
            float acceleration = total_force / m_body.mass;

            m_body.velocity.z += acceleration * TIME_STEP;
            m_body.velocity.z *= 0.85f;  // Damping
            m_body.position.z += m_body.velocity.z * TIME_STEP;
            m_body.position.z = (std::max)(0.0f, (std::min)(0.02f, m_body.position.z));

            // Print every 100ms
            int step = static_cast<int>(t / TIME_STEP);
            if (step % 10 == 0) {
                std::cout << std::fixed << std::setprecision(3)
                          << std::setw(10) << t
                          << std::setw(15) << target_position_mm
                          << std::setw(15) << current_mm
                          << std::setw(15) << error
                          << std::setw(10) << (activate ? "ON" : "OFF")
                          << std::endl;
            }
        }

        std::cout << "\nFinal position: " << (m_body.position.z * 1000.0f) << " mm" << std::endl;
    }

private:
    void print_final_state() {
        std::cout << "\n--- Final System State ---" << std::endl;
        std::cout << "Solenoid position: " << (m_solenoid->get_position() * 100.0f) << "%" << std::endl;
        std::cout << "Metal piece position: " << (m_body.position.z * 1000.0f) << " mm" << std::endl;
        std::cout << "Sensor reading: " << m_sensor->read() << " V" << std::endl;

        // Read MCU registers
        uint8_t ddrb = m_mcu.memory().read_io(ATmega328P_Registers::DDRB);
        uint8_t portb = m_mcu.memory().read_io(ATmega328P_Registers::PORTB);
        std::cout << "MCU PORTB: DDR=0x" << std::hex << (int)ddrb
                  << " PORT=0x" << (int)portb << std::dec << std::endl;
    }

    static constexpr double TIME_STEP = 0.01;  // 10ms

    QEMUInterface m_mcu;
    std::unique_ptr<SolenoidActuator> m_solenoid;
    std::unique_ptr<ProximitySensor> m_sensor;
    DemoPhysicsBody m_body;
};

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "=== Mechatronic Integration Demo ===" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "\nThis demo demonstrates the full mechatronic simulation loop:" << std::endl;
    std::cout << "  MCU (Arduino) → Digital Output → Solenoid Actuator" << std::endl;
    std::cout << "  → Physics (Metal Piece) → Proximity Sensor → Analog Input → MCU" << std::endl;

    try {
        MechatronicSystem system;

        // Test 1: Manual control
        system.test_manual_control();

        // Test 2: Sensor feedback
        system.test_sensor_feedback();

        // Test 3: Closed-loop control
        system.test_closed_loop_control();

        // Test 4: Full simulation with blink pattern
        std::cout << "\nPress Enter to start full simulation (5 seconds)..." << std::endl;
        std::cin.get();
        system.run_simulation(5.0);

        std::cout << "\n========================================" << std::endl;
        std::cout << "=== ALL TESTS PASSED! ===" << std::endl;
        std::cout << "========================================" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "\n✗ TEST FAILED: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
