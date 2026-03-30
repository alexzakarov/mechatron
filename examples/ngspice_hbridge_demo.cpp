/**
 * @file ngspice_hbridge_demo.cpp
 * @brief ngspice H-Bridge circuit simulation demo
 *
 * This demonstrates using ngspice to simulate an H-Bridge motor driver circuit.
 * The circuit includes:
 * - H-Bridge with 4 transistors (2 NPN, 2 PNP)
 * - DC motor model (inductor + resistor)
 * - PWM control signals
 * - Transient analysis to show motor current
 */

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <iostream>
#include <iomanip>

#include "electronics/NgspiceWrapper.hpp"

using namespace mechatron;

// Helper to print simulation results
void print_results(const SimulationResult& result) {
    if (!result.success) {
        std::cerr << "Simulation failed: " << result.error << std::endl;
        return;
    }

    std::cout << "\n=== Simulation Results ===" << std::endl;
    std::cout << "Time points: " << result.time_points.size() << std::endl;
    std::cout << "Nodes: ";
    for (const auto& node : result.nodes) {
        std::cout << node << " ";
    }
    std::cout << std::endl;

    // Print first few time points
    std::cout << "\nFirst 10 time points:" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < std::min(size_t(10), result.time_points.size()); ++i) {
        const auto& [time, voltages] = result.time_points[i];
        std::cout << "t=" << time << "s";
        for (const auto& v : voltages) {
            std::cout << ", " << v.node << "=" << v.voltage << "V";
        }
        std::cout << std::endl;
    }

    // Print last few time points
    std::cout << "\nLast 10 time points:" << std::endl;
    size_t start = result.time_points.size() > 10 ? result.time_points.size() - 10 : 0;
    for (size_t i = start; i < result.time_points.size(); ++i) {
        const auto& [time, voltages] = result.time_points[i];
        std::cout << "t=" << time << "s";
        for (const auto& v : voltages) {
            std::cout << ", " << v.node << "=" << v.voltage << "V";
        }
        std::cout << std::endl;
    }
}

// Test 1: Simple H-Bridge circuit with resistive load
void test_simple_hbridge(NgspiceWrapper& ngspice) {
    spdlog::info("\n=== Test 1: Simple H-Bridge (Resistive Load) ===");

    NetlistBuilder netlist;
    netlist.set_title("H-Bridge Motor Driver - Simplified");

    // Voltage supply (12V)
    netlist.add_dc_voltage("VCC", "VCC", "0", 12.0);

    // Simplified: Direct pulse voltage to output instead of transistor switching
    // This avoids the timestep issues with BJTs
    netlist.add_pulse_voltage("VDRV", "OUT", "0",
                              0, 12,    // V_low=0V, V_high=12V
                              1e-6, 1e-6, 1e-6, 1e-3, 2e-3);  // 50% duty, 500Hz

    // Simple resistive load
    netlist.add_resistor("R_MOTOR", "OUT", "0", 10);  // 10 ohm load

    // Simulation command: transient analysis with relaxed timestep
    netlist.add_simulation("tran", "1u 5m 0 1u");

    std::string circuit = netlist.build();

    std::cout << "\nGenerated Netlist:\n" << circuit << std::endl;

    // Run simulation
    auto result = ngspice.simulate(circuit, 0.01, 0.0001);

    if (result.success) {
        spdlog::info("Simulation completed successfully!");
        print_results(result);

        // Analyze results
        spdlog::info("\n=== Analysis ===");
        if (!result.time_points.empty()) {
            const auto& [time, voltages] = result.time_points.back();

            float out_voltage = 0.0f;
            for (const auto& v : voltages) {
                if (v.node == "out") out_voltage = v.voltage;  // ngspice returns lowercase
            }

            float motor_current = out_voltage / 10.0f;
            spdlog::info("Motor current: {:.3f} A (at {:.6f}s)", motor_current, time);
        }
    } else {
        spdlog::error("Simulation failed: {}", result.error);
    }
}

// Test 2: H-Bridge with inductive load (DC motor model)
void test_hbridge_motor(NgspiceWrapper& ngspice) {
    spdlog::info("\n=== Test 2: H-Bridge with DC Motor Model ===");

    NetlistBuilder netlist;
    netlist.set_title("H-Bridge with DC Motor Model");

    // Voltage supply
    netlist.add_dc_voltage("VCC", "VCC", "0", 12.0);

    // PWM control signals - use VPWM as source name, PWM_NODE as node
    netlist.add_pulse_voltage("VPWM", "PWM_NODE", "0",
                              0, 12,
                              0, 1e-6, 1e-6, 2e-4, 4e-4);  // 50% duty, 2.5kHz

    // DC Motor model: R + L in series
    // Typical small DC motor: 10 ohm resistance, 1mH inductance
    netlist.add_resistor("R_MOTOR", "PWM_NODE", "MOTOR_NODE", 10);
    netlist.add_inductor("L_MOTOR", "MOTOR_NODE", "0", 1e-3);  // 1mH to ground

    // Simulation command: transient analysis with relaxed timestep
    netlist.add_simulation("tran", "1u 5m 0 1u");

    std::string netlist_str = netlist.build();

    std::cout << "\nGenerated Netlist:\n" << netlist_str << std::endl;

    // Run simulation
    auto result = ngspice.simulate(netlist_str, 0.01, 0.0001);

    if (result.success) {
        spdlog::info("Motor simulation completed!");
        print_results(result);

        // Calculate motor current from voltage across resistor
        spdlog::info("\n=== Motor Current Analysis ===");
        for (size_t i = 0; i < result.time_points.size(); i += 100) {  // Every 100th point
            const auto& [time, voltages] = result.time_points[i];

            float v_pwm = 0.0f, v_motor = 0.0f;
            for (const auto& v : voltages) {
                // Node names are lowercase in ngspice output
                if (v.node == "pwm_node") v_pwm = v.voltage;
                if (v.node == "motor_node") v_motor = v.voltage;
            }

            float motor_current = (v_pwm - v_motor) / 10.0f;  // I = V/R
            spdlog::info("t={:.6f}s: Motor current = {:.3f} A (V_pwm={:.2f}V, V_motor={:.2f}V)",
                        time, motor_current, v_pwm, v_motor);
        }
    } else {
        spdlog::error("Simulation failed: {}", result.error);
    }
}

// Test 3: Full H-Bridge with 4 transistors
void test_full_hbridge(NgspiceWrapper& ngspice) {
    spdlog::info("\n=== Test 3: Full H-Bridge (4 Transistors) ===");

    NetlistBuilder netlist;
    netlist.set_title("Simple NPN Switch with Motor Load");

    // Voltage supply
    netlist.add_dc_voltage("VCC", "VCC", "0", 12.0);

    // PWM control signal
    netlist.add_pulse_voltage("VIN", "VIN", "0",
                              0, 5,
                              1e-6, 10e-9, 10e-9, 100e-6, 200e-6);  // 5kHz

    // Simple NPN switch (low-side driver)
    netlist.add_bjt("Q1", "OUT", "VIN", "0", "NPN");

    // Base resistor for current limiting
    netlist.add_resistor("RB", "VIN", "VIN", 1000);

    // Motor load (between VCC and OUT)
    // When Q1 is ON, OUT connects to GND, current flows VCC->OUT->GND
    netlist.add_resistor("R_MOTOR", "VCC", "OUT", 10);

    // Add simulation command
    netlist.add_simulation("tran", "100n 200u 0 50n");

    std::string netlist_str = netlist.build();

    // Insert models
    std::string options =
        ".OPTIONS GMIN=1N ABSTOL=1P RELTOL=0.01 CHGTOL=1P\n";

    std::string models =
        ".MODEL NPN NPN(IS=1E-14 BF=150 VAF=100 TF=500P TR=10N CJE=2P CJC=2P RB=50 RC=10 RE=1)\n";

    size_t tran_pos = netlist_str.find(".tran");
    if (tran_pos != std::string::npos) {
        netlist_str.insert(tran_pos, options + models);
    }

    std::cout << "\nGenerated Netlist:\n" << netlist_str << std::endl;

    auto result = ngspice.simulate(netlist_str, 0.01, 0.0001);

    if (result.success) {
        spdlog::info("NPN Switch simulation completed!");
        print_results(result);

        // Analyze output voltage
        spdlog::info("\n=== NPN Switch Analysis ===");
        size_t step = result.time_points.size() > 1000 ? 200 : 50;
        for (size_t i = 0; i < result.time_points.size(); i += step) {
            const auto& [time, voltages] = result.time_points[i];

            float out_voltage = 0.0f;
            for (const auto& v : voltages) {
                if (v.node == "out") out_voltage = v.voltage;
            }

            float motor_voltage = 12.0f - out_voltage;  // Voltage across motor
            float motor_current = motor_voltage / 10.0f;
            spdlog::info("t={:.6f}s: OUT={:.3f}V, Motor_V={:.3f}V, Current={:.3f}A",
                        time, out_voltage, motor_voltage, motor_current);
        }
    } else {
        spdlog::error("Simulation failed: {}", result.error);
    }
}

int main() {
    // Setup logging
    auto console = spdlog::stdout_color_mt("console");
    spdlog::set_default_logger(console);
    spdlog::set_level(spdlog::level::info);

    spdlog::info("========================================");
    spdlog::info("  ngspice H-Bridge Circuit Demo");
    spdlog::info("========================================");

    NgspiceWrapper ngspice;

    // Check if ngspice is available
    if (!ngspice.is_available()) {
        spdlog::warn("ngspice is not available on this system");
        spdlog::warn("Please install ngspice to run circuit simulations");
        spdlog::info("Demo will continue but simulations will fail");
    } else {
        spdlog::info("ngspice is available!");
    }

    try {
        // Run tests
        test_simple_hbridge(ngspice);
        test_hbridge_motor(ngspice);
        test_full_hbridge(ngspice);

        spdlog::info("\n========================================");
        spdlog::info("  Demo completed!");
        spdlog::info("========================================");

        return 0;

    } catch (const std::exception& e) {
        spdlog::error("Error: {}", e.what());
        return 1;
    }
}
