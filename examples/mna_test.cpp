// Comprehensive MNA Circuit Simulation Verification Tests
// Tests: Ohm's Law, Kirchhoff's Laws, parallel/series resistors,
//        voltage dividers, current dividers, diode behavior, capacitors

#include "electronics/CircuitSimulator.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <cassert>
#include <string>

using namespace mechatron;

static int tests_passed = 0;
static int tests_failed = 0;

void check(const std::string& name, double actual, double expected, double tolerance = 0.05) {
    double error = std::abs(actual - expected);
    double rel_error = (expected != 0.0) ? error / std::abs(expected) : error;
    if (rel_error <= tolerance) {
        std::cout << "  PASS: " << name << " (got " << std::fixed << std::setprecision(4) << actual
                  << ", expected " << expected << ")" << std::endl;
        tests_passed++;
    } else {
        std::cout << "  FAIL: " << name << " (got " << std::fixed << std::setprecision(4) << actual
                  << ", expected " << expected << ", error=" << (rel_error*100) << "%)" << std::endl;
        tests_failed++;
    }
}

// ============================================================================
// Test 1: Simple Ohm's Law - 5V source + 1kΩ resistor + ground
// Expected: I = 5V / 1000Ω = 5mA
// ============================================================================
void test_simple_ohms_law() {
    std::cout << "\n=== Test 1: Simple Ohm's Law (5V + 1kΩ) ===" << std::endl;

    CircuitSimulator sim;
    auto* src = sim.add_component<DCVoltageSource>("V1", 5.0f);
    auto* R1 = sim.add_component<Resistor>("R1", 1000.0f);
    auto* gnd = sim.add_component<Ground>("GND");

    // V1.V+ -> R1.pin1
    sim.connect("w1", "V1", "V+", "R1", "1");
    // R1.pin2 -> GND
    sim.connect("w2", "R1", "2", "GND", "GND");
    // V1.GND -> GND
    sim.connect("w3", "V1", "GND", "GND", "GND");

    sim.step(0.001);

    float current = R1->get_pins()[0]->current;
    float voltage_drop = R1->get_pins()[0]->voltage - R1->get_pins()[1]->voltage;

    check("Voltage across R1", voltage_drop, 5.0);
    check("Current through R1 (mA)", current * 1000.0, 5.0);
}

// ============================================================================
// Test 2: Voltage Divider - 10V source + 3kΩ + 7kΩ in series
// Expected: V_mid = 10V * 7kΩ / (3kΩ + 7kΩ) = 7V  (voltage across R2)
//           I = 10V / 10kΩ = 1mA
// ============================================================================
void test_voltage_divider() {
    std::cout << "\n=== Test 2: Voltage Divider (10V, 3kΩ + 7kΩ) ===" << std::endl;

    CircuitSimulator sim;
    auto* src = sim.add_component<DCVoltageSource>("V1", 10.0f);
    auto* R1 = sim.add_component<Resistor>("R1", 3000.0f);
    auto* R2 = sim.add_component<Resistor>("R2", 7000.0f);
    auto* gnd = sim.add_component<Ground>("GND");

    // V1.V+ -> R1.pin1
    sim.connect("w1", "V1", "V+", "R1", "1");
    // R1.pin2 -> R2.pin1 (midpoint)
    sim.connect("w2", "R1", "2", "R2", "1");
    // R2.pin2 -> GND
    sim.connect("w3", "R2", "2", "GND", "GND");
    // V1.GND -> GND
    sim.connect("w4", "V1", "GND", "GND", "GND");

    sim.step(0.001);

    float v_mid = R1->get_pins()[1]->voltage;  // R1.pin2 = R2.pin1 = midpoint
    float current = R1->get_pins()[0]->current;

    check("Midpoint voltage (V)", v_mid, 7.0);
    check("Series current (mA)", current * 1000.0, 1.0);
    check("Voltage across R1", R1->get_pins()[0]->voltage - R1->get_pins()[1]->voltage, 3.0);
    check("Voltage across R2", R2->get_pins()[0]->voltage - R2->get_pins()[1]->voltage, 7.0);
}

// ============================================================================
// Test 3: Parallel Resistors - 5V source + (1kΩ || 2kΩ) + ground
// Expected: I_1kΩ = 5mA, I_2kΩ = 2.5mA, I_total = 7.5mA
//           R_eq = 1/(1/1000 + 1/2000) = 666.67Ω
// ============================================================================
void test_parallel_resistors() {
    std::cout << "\n=== Test 3: Parallel Resistors (5V, 1kΩ || 2kΩ) ===" << std::endl;

    CircuitSimulator sim;
    auto* src = sim.add_component<DCVoltageSource>("V1", 5.0f);
    auto* R1 = sim.add_component<Resistor>("R1", 1000.0f);  // Low resistance
    auto* R2 = sim.add_component<Resistor>("R2", 2000.0f);  // Higher resistance
    auto* gnd = sim.add_component<Ground>("GND");

    // V1.V+ -> R1.pin1
    sim.connect("w1", "V1", "V+", "R1", "1");
    // V1.V+ -> R2.pin1
    sim.connect("w2", "V1", "V+", "R2", "1");
    // R1.pin2 -> GND
    sim.connect("w3", "R1", "2", "GND", "GND");
    // R2.pin2 -> GND
    sim.connect("w4", "R2", "2", "GND", "GND");
    // V1.GND -> GND
    sim.connect("w5", "V1", "GND", "GND", "GND");

    sim.step(0.001);

    float I_R1 = R1->get_pins()[0]->current;
    float I_R2 = R2->get_pins()[0]->current;
    float I_total = I_R1 + I_R2;

    check("Current through 1kΩ (mA)", I_R1 * 1000.0, 5.0);
    check("Current through 2kΩ (mA)", I_R2 * 1000.0, 2.5);
    check("Total current (mA)", I_total * 1000.0, 7.5);

    // Verify current follows path of least resistance:
    // R1 has lower resistance, so more current should flow through it
    check("R1 current > R2 current", (I_R1 > I_R2) ? 1.0 : 0.0, 1.0);
}

// ============================================================================
// Test 4: Three Parallel Paths - 12V + (1kΩ || 2kΩ || 3kΩ)
// Expected: I_1k = 12mA, I_2k = 6mA, I_3k = 4mA, I_total = 22mA
// ============================================================================
void test_three_parallel_paths() {
    std::cout << "\n=== Test 4: Three Parallel Paths (12V, 1kΩ || 2kΩ || 3kΩ) ===" << std::endl;

    CircuitSimulator sim;
    auto* src = sim.add_component<DCVoltageSource>("V1", 12.0f);
    auto* R1 = sim.add_component<Resistor>("R1", 1000.0f);
    auto* R2 = sim.add_component<Resistor>("R2", 2000.0f);
    auto* R3 = sim.add_component<Resistor>("R3", 3000.0f);
    auto* gnd = sim.add_component<Ground>("GND");

    // All resistor pin1 connected to V+
    sim.connect("w1", "V1", "V+", "R1", "1");
    sim.connect("w2", "V1", "V+", "R2", "1");
    sim.connect("w3", "V1", "V+", "R3", "1");
    // All resistor pin2 connected to GND
    sim.connect("w4", "R1", "2", "GND", "GND");
    sim.connect("w5", "R2", "2", "GND", "GND");
    sim.connect("w6", "R3", "2", "GND", "GND");
    sim.connect("w7", "V1", "GND", "GND", "GND");

    sim.step(0.001);

    float I1 = R1->get_pins()[0]->current * 1000.0f;
    float I2 = R2->get_pins()[0]->current * 1000.0f;
    float I3 = R3->get_pins()[0]->current * 1000.0f;
    float I_total = I1 + I2 + I3;

    check("I_1kΩ (mA)", I1, 12.0);
    check("I_2kΩ (mA)", I2, 6.0);
    check("I_3kΩ (mA)", I3, 4.0);
    check("I_total (mA)", I_total, 22.0);

    // Verify: lowest resistance = highest current
    check("I_1k > I_2k > I_3k", (I1 > I2 && I2 > I3) ? 1.0 : 0.0, 1.0);
}

// ============================================================================
// Test 5: Kirchhoff's Voltage Law (KVL)
// 12V source + R1(2kΩ) + R2(3kΩ) + R3(1kΩ) in series
// Expected: V_R1 + V_R2 + V_R3 = 12V, I = 12/6000 = 2mA
// V_R1 = 4V, V_R2 = 6V, V_R3 = 2V
// ============================================================================
void test_kirchhoff_voltage_law() {
    std::cout << "\n=== Test 5: Kirchhoff's Voltage Law (12V, 2k+3k+1k series) ===" << std::endl;

    CircuitSimulator sim;
    auto* src = sim.add_component<DCVoltageSource>("V1", 12.0f);
    auto* R1 = sim.add_component<Resistor>("R1", 2000.0f);
    auto* R2 = sim.add_component<Resistor>("R2", 3000.0f);
    auto* R3 = sim.add_component<Resistor>("R3", 1000.0f);
    auto* gnd = sim.add_component<Ground>("GND");

    // Series connection: V+ -> R1 -> R2 -> R3 -> GND
    sim.connect("w1", "V1", "V+", "R1", "1");
    sim.connect("w2", "R1", "2", "R2", "1");
    sim.connect("w3", "R2", "2", "R3", "1");
    sim.connect("w4", "R3", "2", "GND", "GND");
    sim.connect("w5", "V1", "GND", "GND", "GND");

    sim.step(0.001);

    float V_R1 = R1->get_pins()[0]->voltage - R1->get_pins()[1]->voltage;
    float V_R2 = R2->get_pins()[0]->voltage - R2->get_pins()[1]->voltage;
    float V_R3 = R3->get_pins()[0]->voltage - R3->get_pins()[1]->voltage;
    float V_sum = V_R1 + V_R2 + V_R3;
    float I = R1->get_pins()[0]->current * 1000.0f;

    check("V_R1 = 4V", V_R1, 4.0);
    check("V_R2 = 6V", V_R2, 6.0);
    check("V_R3 = 2V", V_R3, 2.0);
    check("KVL: V_R1+V_R2+V_R3 = 12V", V_sum, 12.0);
    check("Series current (mA)", I, 2.0);
}

// ============================================================================
// Test 6: Kirchhoff's Current Law (KCL) at a node
// 12V -> R1(4kΩ) -> Node A -> R2(6kΩ) and R3(6kΩ) -> GND
// Expected: I_R1 = 1.5mA, I_R2 = 0.75mA, I_R3 = 0.75mA
//           I_R1 = I_R2 + I_R3 (KCL at Node A)
// ============================================================================
void test_kirchhoff_current_law() {
    std::cout << "\n=== Test 6: Kirchhoff's Current Law (KCL at node) ===" << std::endl;

    CircuitSimulator sim;
    auto* src = sim.add_component<DCVoltageSource>("V1", 12.0f);
    auto* R1 = sim.add_component<Resistor>("R1", 4000.0f);
    auto* R2 = sim.add_component<Resistor>("R2", 6000.0f);
    auto* R3 = sim.add_component<Resistor>("R3", 6000.0f);
    auto* gnd = sim.add_component<Ground>("GND");

    // V+ -> R1 -> Node A (R1.pin2 = R2.pin1 = R3.pin1)
    sim.connect("w1", "V1", "V+", "R1", "1");
    sim.connect("w2", "R1", "2", "R2", "1");
    sim.connect("w3", "R1", "2", "R3", "1");
    sim.connect("w4", "R2", "2", "GND", "GND");
    sim.connect("w5", "R3", "2", "GND", "GND");
    sim.connect("w6", "V1", "GND", "GND", "GND");

    sim.step(0.001);

    float I_in = R1->get_pins()[1]->current;   // Current INTO node from R1 (negative because it's pin2)
    float I_R2 = R2->get_pins()[0]->current;   // Current out of node through R2
    float I_R3 = R3->get_pins()[0]->current;   // Current out of node through R3

    // Node voltage: voltage divider R1 vs (R2||R3)
    // R2||R3 = 3kΩ, so Node A = 12V * 3k/(4k+3k) = 12 * 3/7 ≈ 5.143V
    float node_voltage = R2->get_pins()[0]->voltage;
    float expected_node_v = 12.0f * 3000.0f / (4000.0f + 3000.0f);

    check("Node A voltage (V)", node_voltage, expected_node_v);
    check("Current through R2 (mA)", I_R2 * 1000.0, 0.857, 0.1);
    check("Current through R3 (mA)", I_R3 * 1000.0, 0.857, 0.1);

    // KCL verification: |I_in| should equal |I_R2 + I_R3|
    float I_out_total = I_R2 + I_R3;
    check("KCL: |I_in| ≈ |I_out_total|", std::abs(I_in), std::abs(I_out_total), 0.15);
}

// ============================================================================
// Test 7: Current Division - verify lower resistance gets more current
// 10V -> R1(100Ω) || R2(900Ω) -> GND
// I_R1 = 100mA, I_R2 = 11.11mA
// I_R1/I_R2 = R2/R1 = 900/100 = 9
// ============================================================================
void test_current_division() {
    std::cout << "\n=== Test 7: Current Division Ratio ===" << std::endl;

    CircuitSimulator sim;
    auto* src = sim.add_component<DCVoltageSource>("V1", 10.0f);
    auto* R1 = sim.add_component<Resistor>("R1", 100.0f);
    auto* R2 = sim.add_component<Resistor>("R2", 900.0f);
    auto* gnd = sim.add_component<Ground>("GND");

    sim.connect("w1", "V1", "V+", "R1", "1");
    sim.connect("w2", "V1", "V+", "R2", "1");
    sim.connect("w3", "R1", "2", "GND", "GND");
    sim.connect("w4", "R2", "2", "GND", "GND");
    sim.connect("w5", "V1", "GND", "GND", "GND");

    sim.step(0.001);

    float I_R1 = R1->get_pins()[0]->current;
    float I_R2 = R2->get_pins()[0]->current;
    float ratio = I_R1 / I_R2;  // Should be ~9

    check("I_R1 = 100mA", I_R1 * 1000.0, 100.0);
    check("I_R2 = 11.11mA", I_R2 * 1000.0, 11.11, 0.1);
    check("I_R1/I_R2 ratio = 9", ratio, 9.0, 0.1);
}

// ============================================================================
// Test 8: LED Forward Voltage Drop
// 5V -> LED(Vf=2V) -> R(150Ω) -> GND
// Expected: I ≈ (5V - 2V) / 150Ω ≈ 20mA
// ============================================================================
void test_led_circuit() {
    std::cout << "\n=== Test 8: LED Forward Voltage Drop ===" << std::endl;

    CircuitSimulator sim;
    auto* src = sim.add_component<DCVoltageSource>("V1", 5.0f);
    auto* led = sim.add_component<LED>("LED1", 2.0f);
    auto* R1 = sim.add_component<Resistor>("R1", 150.0f);
    auto* gnd = sim.add_component<Ground>("GND");

    sim.connect("w1", "V1", "V+", "LED1", "anode");
    sim.connect("w2", "LED1", "cathode", "R1", "1");
    sim.connect("w3", "R1", "2", "GND", "GND");
    sim.connect("w4", "V1", "GND", "GND", "GND");

    sim.step(0.001);

    float I = led->get_pins()[0]->current;
    float expected_I = (5.0f - 2.0f) / 150.0f;

    check("LED current (mA)", I * 1000.0, expected_I * 1000.0, 0.15);
    check("LED is lit", led->is_lit() ? 1.0 : 0.0, 1.0);
}

// ============================================================================
// Test 9: Capacitor Steady-State (DC) - should act as open circuit
// 5V -> R(1kΩ) -> C(1µF) -> GND
// After settling: no current through C, V_C = 5V
// ============================================================================
void test_capacitor_dc_steady() {
    std::cout << "\n=== Test 9: Capacitor DC Steady-State ===" << std::endl;

    CircuitSimulator sim;
    auto* src = sim.add_component<DCVoltageSource>("V1", 5.0f);
    auto* R1 = sim.add_component<Resistor>("R1", 1000.0f);
    auto* C1 = sim.add_component<Capacitor>("C1", 1e-6f);
    auto* gnd = sim.add_component<Ground>("GND");

    sim.connect("w1", "V1", "V+", "R1", "1");
    sim.connect("w2", "R1", "2", "C1", "1");
    sim.connect("w3", "C1", "2", "GND", "GND");
    sim.connect("w4", "V1", "GND", "GND", "GND");

    // Run many steps to reach steady state
    for (int i = 0; i < 100; i++) {
        sim.step(0.001);
    }

    float V_C = C1->get_pins()[0]->voltage - C1->get_pins()[1]->voltage;

    // At steady state, capacitor voltage should approach 5V
    check("Capacitor voltage approaches 5V", V_C, 5.0, 0.15);
}

// ============================================================================
// Test 10: Wheatstone Bridge
//        R1(1k)    R2(1k)
//  5V --+---/\/\/---+--- GND
//       |           |
//       +---/\/\/---+
//        R3(1k)    R4(1.5k)
//
// V_mid_top = 5V * 1k/(1k+1k) = 2.5V
// V_mid_bot = 5V * 1.5k/(1k+1.5k) = 3V
// V_diff = 3V - 2.5V = 0.5V
// ============================================================================
void test_wheatstone_bridge() {
    std::cout << "\n=== Test 10: Wheatstone Bridge ===" << std::endl;

    CircuitSimulator sim;
    auto* src = sim.add_component<DCVoltageSource>("V1", 5.0f);
    auto* R1 = sim.add_component<Resistor>("R1", 1000.0f);
    auto* R2 = sim.add_component<Resistor>("R2", 1000.0f);
    auto* R3 = sim.add_component<Resistor>("R3", 1000.0f);
    auto* R4 = sim.add_component<Resistor>("R4", 1500.0f);
    auto* gnd = sim.add_component<Ground>("GND");

    // Top path: V+ -> R1 -> R2 -> GND
    sim.connect("w1", "V1", "V+", "R1", "1");
    sim.connect("w2", "R1", "2", "R2", "1");
    sim.connect("w3", "R2", "2", "GND", "GND");

    // Bottom path: V+ -> R3 -> R4 -> GND
    sim.connect("w4", "V1", "V+", "R3", "1");
    sim.connect("w5", "R3", "2", "R4", "1");
    sim.connect("w6", "R4", "2", "GND", "GND");

    sim.connect("w7", "V1", "GND", "GND", "GND");

    sim.step(0.001);

    float V_top = R1->get_pins()[1]->voltage;  // Midpoint of top path
    float V_bot = R3->get_pins()[1]->voltage;  // Midpoint of bottom path
    float V_diff = V_bot - V_top;

    check("Top midpoint = 2.5V", V_top, 2.5);
    check("Bot midpoint = 3.0V", V_bot, 3.0);
    check("Bridge voltage diff = 0.5V", V_diff, 0.5);
}

// ============================================================================
// Test 11: Current Through Voltage Source
// 5V -> R(500Ω) -> GND
// Check that voltage source branch current is reported correctly
// I = 10mA
// ============================================================================
void test_voltage_source_current() {
    std::cout << "\n=== Test 11: Voltage Source Current ===" << std::endl;

    CircuitSimulator sim;
    auto* src = sim.add_component<DCVoltageSource>("V1", 5.0f);
    auto* R1 = sim.add_component<Resistor>("R1", 500.0f);
    auto* gnd = sim.add_component<Ground>("GND");

    sim.connect("w1", "V1", "V+", "R1", "1");
    sim.connect("w2", "R1", "2", "GND", "GND");
    sim.connect("w3", "V1", "GND", "GND", "GND");

    sim.step(0.001);

    float I = R1->get_pins()[0]->current;
    check("Source current = 10mA", I * 1000.0, 10.0);
}

// ============================================================================
// Test 12: Multiple Voltage Sources
// 5V -> R(1k) -> 3V (second source) -> GND
// Net voltage across R = 5V - 3V = 2V, I = 2mA
// ============================================================================
void test_multiple_sources() {
    std::cout << "\n=== Test 12: Multiple Voltage Sources ===" << std::endl;

    CircuitSimulator sim;
    auto* V1 = sim.add_component<DCVoltageSource>("V1", 5.0f);
    auto* V2 = sim.add_component<DCVoltageSource>("V2", 3.0f);
    auto* R1 = sim.add_component<Resistor>("R1", 1000.0f);
    auto* gnd = sim.add_component<Ground>("GND");

    // V1.V+ -> R1.pin1
    sim.connect("w1", "V1", "V+", "R1", "1");
    // R1.pin2 -> V2.V+
    sim.connect("w2", "R1", "2", "V2", "V+");
    // V2.GND -> GND
    sim.connect("w3", "V2", "GND", "GND", "GND");
    // V1.GND -> GND
    sim.connect("w4", "V1", "GND", "GND", "GND");

    sim.step(0.001);

    float I = R1->get_pins()[0]->current;
    check("Current (V1-V2)/R (mA)", I * 1000.0, 2.0);
}

// ============================================================================
// Test 13: ESC 3-Phase Output with Commutation
// Demonstrates 3-phase inverter output with 6-step trapezoidal commutation
// Uses real MOSFETs with gate drive, shows voltage changes in real-time
// ============================================================================
void test_esc_half_bridge() {
    std::cout << "\n=== Test 13: ESC 3-Phase Output ===" << std::endl;

    CircuitSimulator sim;

    // Power: 12V battery
    auto* bat = sim.add_component<DCVoltageSource>("BAT", 12.0f);
    auto* gnd = sim.add_component<Ground>("GND");
    sim.connect("w_bat_gnd", "BAT", "GND", "GND", "GND");

    // 3 Half-bridges: 6 MOSFETs (AH/AL, BH/BL, CH/CL)
    auto* AH = sim.add_component<MOSFETTransistor>("AH",
        MOSFETTransistor::NChannel, 2.0f, 100.0f);
    auto* AL = sim.add_component<MOSFETTransistor>("AL",
        MOSFETTransistor::NChannel, 2.0f, 100.0f);
    auto* BH = sim.add_component<MOSFETTransistor>("BH",
        MOSFETTransistor::NChannel, 2.0f, 100.0f);
    auto* BL = sim.add_component<MOSFETTransistor>("BL",
        MOSFETTransistor::NChannel, 2.0f, 100.0f);
    auto* CH = sim.add_component<MOSFETTransistor>("CH",
        MOSFETTransistor::NChannel, 2.0f, 100.0f);
    auto* CL = sim.add_component<MOSFETTransistor>("CL",
        MOSFETTransistor::NChannel, 2.0f, 100.0f);

    // Gate drive sources (control MOSFET on/off)
    auto* g_ah = sim.add_component<DCVoltageSource>("G_AH", 12.0f);
    auto* g_al = sim.add_component<DCVoltageSource>("G_AL", 0.0f);
    auto* g_bh = sim.add_component<DCVoltageSource>("G_BH", 0.0f);
    auto* g_bl = sim.add_component<DCVoltageSource>("G_BL", 0.0f);
    auto* g_ch = sim.add_component<DCVoltageSource>("G_CH", 0.0f);
    auto* g_cl = sim.add_component<DCVoltageSource>("G_CL", 0.0f);

    // Gate resistors (0.1Ω for strong drive)
    auto* Rg_ah = sim.add_component<Resistor>("RgAH", 0.1f);
    auto* Rg_al = sim.add_component<Resistor>("RgAL", 0.1f);
    auto* Rg_bh = sim.add_component<Resistor>("RgBH", 0.1f);
    auto* Rg_bl = sim.add_component<Resistor>("RgBL", 0.1f);
    auto* Rg_ch = sim.add_component<Resistor>("RgCH", 0.1f);
    auto* Rg_cl = sim.add_component<Resistor>("RgCL", 0.1f);

    // Helper: connect high-side gate with bootstrap
    auto connect_gate_hi = [&](const char* vg, const char* rg, const char* mosfet, const char* src) {
        sim.connect(std::string("w_") + vg + "_gs", vg, "V+", rg, "1");
        sim.connect(std::string("w_") + vg + "_gm", rg, "2", mosfet, "gate");
        sim.connect(std::string("w_") + vg + "_gg", vg, "GND", src, "source");
    };
    // Helper: connect low-side gate (GND referenced)
    auto connect_gate_lo = [&](const char* vg, const char* rg, const char* mosfet) {
        sim.connect(std::string("w_") + vg + "_gs", vg, "V+", rg, "1");
        sim.connect(std::string("w_") + vg + "_gm", rg, "2", mosfet, "gate");
        sim.connect(std::string("w_") + vg + "_gg", vg, "GND", "GND", "GND");
    };

    connect_gate_hi("G_AH", "RgAH", "AH", "AH");
    connect_gate_lo("G_AL", "RgAL", "AL");
    connect_gate_hi("G_BH", "RgBH", "BH", "BH");
    connect_gate_lo("G_BL", "RgBL", "BL");
    connect_gate_hi("G_CH", "RgCH", "CH", "CH");
    connect_gate_lo("G_CL", "RgCL", "CL");

    // Power connections
    sim.connect("w_ah_drain", "BAT", "V+", "AH", "drain");
    sim.connect("w_bh_drain", "BAT", "V+", "BH", "drain");
    sim.connect("w_ch_drain", "BAT", "V+", "CH", "drain");

    // Phase nodes
    sim.connect("w_phase_a", "AH", "source", "AL", "drain");
    sim.connect("w_phase_b", "BH", "source", "BL", "drain");
    sim.connect("w_phase_c", "CH", "source", "CL", "drain");

    // Low-side to GND
    sim.connect("w_al_gnd", "AL", "source", "GND", "GND");
    sim.connect("w_bl_gnd", "BL", "source", "GND", "GND");
    sim.connect("w_cl_gnd", "CL", "source", "GND", "GND");

    // Add load resistors (motor winding simulation)
    // Each phase has 1Ω to star point, star point floats
    auto* R_A = sim.add_component<Resistor>("R_A", 1.0f);
    auto* R_B = sim.add_component<Resistor>("R_B", 1.0f);
    auto* R_C = sim.add_component<Resistor>("R_C", 1.0f);

    // Connect phase nodes to load resistors
    sim.connect("w_ra1", "AH", "source", "R_A", "1");
    sim.connect("w_rb1", "BH", "source", "R_B", "1");
    sim.connect("w_rc1", "CH", "source", "R_C", "1");

    // Star connection: all resistor pin2s connected together
    sim.connect("w_star_ab", "R_A", "2", "R_B", "2");
    sim.connect("w_star_bc", "R_B", "2", "R_C", "2");

    // Phase outputs
    auto* v_phase_a = AH->get_pins()[2];  // source pin
    auto* v_phase_b = BH->get_pins()[2];
    auto* v_phase_c = CH->get_pins()[2];

    // Verification: each phase should reach 12V when high-side ON, 0V when low-side ON
    float max_phase_a = 0, min_phase_a = 12;
    float max_phase_b = 0, min_phase_b = 12;
    float max_phase_c = 0, min_phase_c = 12;

    std::cout << "\n  Step | AH  AL  BH  BL  CH  CL | Phase A | Phase B | Phase C" << std::endl;
    std::cout << "  -----|------------------------|---------|---------|---------" << std::endl;

    // 6-step commutation sequence
    int commutation_steps[6][6] = {
        // AH  AL  BH  BL  CH  CL
        {  1,  0,  0,  1,  0,  0 },  // Step 1: A high, B low  (current VCC→A→B→GND)
        {  1,  0,  0,  0,  0,  1 },  // Step 2: A high, C low  (VCC→A→C→GND)
        {  0,  0,  1,  0,  0,  1 },  // Step 3: B high, C low  (VCC→B→C→GND)
        {  0,  1,  1,  0,  0,  0 },  // Step 4: B high, A low  (VCC→B→A→GND)
        {  0,  1,  0,  0,  1,  0 },  // Step 5: C high, A low  (VCC→C→A→GND)
        {  0,  0,  1,  1,  0,  0 }   // Step 6: C high, B low  (VCC→C→B→GND)
    };

    for (int step = 0; step < 6; step++) {
        // Set gate voltages
        g_ah->set_parameter("voltage", commutation_steps[step][0] ? 12.0 : 0.0);
        g_al->set_parameter("voltage", commutation_steps[step][1] ? 12.0 : 0.0);
        g_bh->set_parameter("voltage", commutation_steps[step][2] ? 12.0 : 0.0);
        g_bl->set_parameter("voltage", commutation_steps[step][3] ? 12.0 : 0.0);
        g_ch->set_parameter("voltage", commutation_steps[step][4] ? 12.0 : 0.0);
        g_cl->set_parameter("voltage", commutation_steps[step][5] ? 12.0 : 0.0);

        sim.step(0.001);

        // Track max/min phase voltages
        max_phase_a = (std::max)(max_phase_a, v_phase_a->voltage);
        min_phase_a = (std::min)(min_phase_a, v_phase_a->voltage);
        max_phase_b = (std::max)(max_phase_b, v_phase_b->voltage);
        min_phase_b = (std::min)(min_phase_b, v_phase_b->voltage);
        max_phase_c = (std::max)(max_phase_c, v_phase_c->voltage);
        min_phase_c = (std::min)(min_phase_c, v_phase_c->voltage);

        // Output phase voltages
        std::cout << "  " << (step+1) << "    | ";
        for (int i = 0; i < 6; i++) {
            std::cout << (commutation_steps[step][i] ? "ON " : "OFF") << " ";
        }
        std::cout << "| " << std::fixed << std::setprecision(1) << v_phase_a->voltage << "V    "
                  << v_phase_b->voltage << "V    " << v_phase_c->voltage << "V" << std::endl;
    }

    std::cout << "\n  Verification:" << std::endl;
    check("Phase A reaches 12V", max_phase_a, 12.0, 0.1);
    check("Phase A drops to 0V", min_phase_a, 0.0, 0.5);
    check("Phase B reaches 12V", max_phase_b, 12.0, 0.1);
    check("Phase B drops to 0V", min_phase_b, 0.0, 0.5);
    check("Phase C reaches 12V", max_phase_c, 12.0, 0.1);
    check("Phase C drops to 0V", min_phase_c, 0.0, 0.5);
}

// ============================================================================
// Test 14: ESC 3-Phase Commutation (6-step trapezoidal)
// 6 N-Channel MOSFETs in 3 half-bridges
// 12V supply, motor phases connected via load resistors (1Ω winding)
// Commutation step 1: AH+BL ON → current 12V→A→R_A→R_B→B→GND
// ============================================================================
void test_esc_three_phase_commutation() {
    std::cout << "\n=== Test 14: ESC 3-Phase Commutation ===" << std::endl;

    CircuitSimulator sim;

    // Power
    auto* bat = sim.add_component<DCVoltageSource>("BAT", 12.0f);
    auto* gnd = sim.add_component<Ground>("GND");
    sim.connect("w_bat_gnd", "BAT", "GND", "GND", "GND");

    // Motor winding resistors (simplified: 1Ω per phase winding)
    auto* R_A = sim.add_component<Resistor>("R_A", 1.0f);
    auto* R_B = sim.add_component<Resistor>("R_B", 1.0f);
    auto* R_C = sim.add_component<Resistor>("R_C", 1.0f);

    // 6 MOSFETs: AH, AL, BH, BL, CH, CL
    // Kp reduced to realistic value (IRF3205-ish: Kp ≈ 10A/V² for power MOSFET)
    auto* AH = sim.add_component<MOSFETTransistor>("AH",
        MOSFETTransistor::NChannel, 2.0f, 10.0f);
    auto* AL = sim.add_component<MOSFETTransistor>("AL",
        MOSFETTransistor::NChannel, 2.0f, 10.0f);
    auto* BH = sim.add_component<MOSFETTransistor>("BH",
        MOSFETTransistor::NChannel, 2.0f, 10.0f);
    auto* BL = sim.add_component<MOSFETTransistor>("BL",
        MOSFETTransistor::NChannel, 2.0f, 10.0f);
    auto* CH = sim.add_component<MOSFETTransistor>("CH",
        MOSFETTransistor::NChannel, 2.0f, 10.0f);
    auto* CL = sim.add_component<MOSFETTransistor>("CL",
        MOSFETTransistor::NChannel, 2.0f, 10.0f);

    // Gate drive voltage sources for each MOSFET
    // We'll control these to simulate commutation
    auto* g_ah = sim.add_component<DCVoltageSource>("G_AH", 12.0f);
    auto* g_al = sim.add_component<DCVoltageSource>("G_AL", 0.0f);
    auto* g_bh = sim.add_component<DCVoltageSource>("G_BH", 0.0f);
    auto* g_bl = sim.add_component<DCVoltageSource>("G_BL", 12.0f);
    auto* g_ch = sim.add_component<DCVoltageSource>("G_CH", 0.0f);
    auto* g_cl = sim.add_component<DCVoltageSource>("G_CL", 0.0f);

    // Gate resistors (0.1Ω each for strong gate drive)
    auto* Rg_ah = sim.add_component<Resistor>("RgAH", 0.1f);
    auto* Rg_al = sim.add_component<Resistor>("RgAL", 0.1f);
    auto* Rg_bh = sim.add_component<Resistor>("RgBH", 0.1f);
    auto* Rg_bl = sim.add_component<Resistor>("RgBL", 0.1f);
    auto* Rg_ch = sim.add_component<Resistor>("RgCH", 0.1f);
    auto* Rg_cl = sim.add_component<Resistor>("RgCL", 0.1f);

    // Helper lambda for low-side gate connections (GND referenced)
    auto connect_gate_lo = [&](const char* prefix, const char* gate_src,
                               const char* rg, const char* mosfet) {
        sim.connect(std::string(prefix) + "_gs", gate_src, "V+", rg, "1");
        sim.connect(std::string(prefix) + "_gm", rg, "2", mosfet, "gate");
        sim.connect(std::string(prefix) + "_gg", gate_src, "GND", "GND", "GND");
    };

    // Helper lambda for high-side gate connections (bootstrap: referenced to source)
    auto connect_gate_hi = [&](const char* prefix, const char* gate_src,
                               const char* rg, const char* mosfet) {
        sim.connect(std::string(prefix) + "_gs", gate_src, "V+", rg, "1");
        sim.connect(std::string(prefix) + "_gm", rg, "2", mosfet, "gate");
        sim.connect(std::string(prefix) + "_gg", gate_src, "GND", mosfet, "source");
    };

    // High-side: bootstrap (gate GND → MOSFET source = phase node)
    connect_gate_hi("ah", "G_AH", "RgAH", "AH");
    connect_gate_hi("bh", "G_BH", "RgBH", "BH");
    connect_gate_hi("ch", "G_CH", "RgCH", "CH");
    // Low-side: GND referenced
    connect_gate_lo("al", "G_AL", "RgAL", "AL");
    connect_gate_lo("bl", "G_BL", "RgBL", "BL");
    connect_gate_lo("cl", "G_CL", "RgCL", "CL");

    // Power connections: BAT V+ → all high-side drains
    sim.connect("w_ah_drain", "BAT", "V+", "AH", "drain");
    sim.connect("w_bh_drain", "BAT", "V+", "BH", "drain");
    sim.connect("w_ch_drain", "BAT", "V+", "CH", "drain");

    // Phase outputs: high-side source = low-side drain = phase node
    sim.connect("w_phase_a", "AH", "source", "AL", "drain");
    sim.connect("w_phase_b", "BH", "source", "BL", "drain");
    sim.connect("w_phase_c", "CH", "source", "CL", "drain");

    // Low-side sources → GND
    sim.connect("w_al_gnd", "AL", "source", "GND", "GND");
    sim.connect("w_bl_gnd", "BL", "source", "GND", "GND");
    sim.connect("w_cl_gnd", "CL", "source", "GND", "GND");

    // Motor windings: Phase A → R_A → star point, etc.
    // Star connection: R_A pin2 = R_B pin2 = R_C pin2 (star point)
    sim.connect("w_ra1", "AH", "source", "R_A", "1");
    sim.connect("w_rb1", "BH", "source", "R_B", "1");
    sim.connect("w_rc1", "CH", "source", "R_C", "1");
    sim.connect("w_star_ab", "R_A", "2", "R_B", "2");
    sim.connect("w_star_bc", "R_B", "2", "R_C", "2");

    // === Commutation Step 1: AH + BL ON (current: VCC→A→B→GND) ===
    // AH=ON, BL=ON, all others OFF
    sim.step(0.001);

    float v_phase_a = AH->get_pins()[2]->voltage;
    float v_phase_b = BH->get_pins()[2]->voltage;
    float v_phase_c = CH->get_pins()[2]->voltage;

    printf("Step 1: Phase A=%.2fV (from AH source pin)\n", v_phase_a);
    fflush(stdout);
    printf("  AH pins: gate=%.2fV, drain=%.2fV, source=%.2fV\n",
           AH->get_pins()[0]->voltage, AH->get_pins()[1]->voltage, AH->get_pins()[2]->voltage);
    fflush(stdout);

    // Phase A should be near 12V (high-side ON)
    check("Step1: Phase A near 12V", v_phase_a, 12.0, 0.1);
    // Phase B should be near 0V (low-side ON)
    check("Step1: Phase B near 0V", v_phase_b, 0.0, 0.5);

    // Current should flow: VCC→AH→PhaseA→R_A→star→R_B→PhaseB→BL→GND
    float i_a = R_A->get_pins()[0]->current;
    float i_b = R_B->get_pins()[0]->current;
    float expected_I = 12.0f / 2.0f; // 12V / (R_A + R_B) = 6A
    check("Step1: Phase A current (A)", i_a, expected_I, 0.1);
    check("Step1: Phase B current = Phase A", i_b, i_a, 0.05);

    // Phase C should be floating (no MOSFET on), near star point voltage
    float v_star = R_A->get_pins()[1]->voltage;
    check("Step1: Phase C floating at star point", v_phase_c, v_star, 0.1);

    // === Commutation Step 2: AH + CL ON (current: VCC→A→C→GND) ===
    g_ah->set_parameter("voltage", 12.0);  // AH ON
    g_al->set_parameter("voltage", 0.0);
    g_bh->set_parameter("voltage", 0.0);
    g_bl->set_parameter("voltage", 0.0);   // BL OFF
    g_ch->set_parameter("voltage", 0.0);
    g_cl->set_parameter("voltage", 12.0);  // CL ON

    sim.step(0.001);

    v_phase_a = AH->get_pins()[2]->voltage;
    v_phase_c = CH->get_pins()[2]->voltage;
    float i_c = R_C->get_pins()[0]->current;
    expected_I = 12.0f / 2.0f; // same total resistance

    check("Step2: Phase A near 12V", v_phase_a, 12.0, 0.1);
    check("Step2: Phase C near 0V", v_phase_c, 0.0, 0.5);
    check("Step2: Phase C current (A)", i_c, expected_I, 0.1);

    // === Commutation Step 3: BH + CL ON (current: VCC→B→C→GND) ===
    g_ah->set_parameter("voltage", 0.0);   // AH OFF
    g_bh->set_parameter("voltage", 12.0);  // BH ON
    g_cl->set_parameter("voltage", 12.0);  // CL ON
    g_al->set_parameter("voltage", 0.0);
    g_bl->set_parameter("voltage", 0.0);
    g_ch->set_parameter("voltage", 0.0);

    sim.step(0.001);

    v_phase_b = BH->get_pins()[2]->voltage;
    v_phase_c = CH->get_pins()[2]->voltage;
    i_b = R_B->get_pins()[0]->current;
    i_c = R_C->get_pins()[0]->current;

    check("Step3: Phase B near 12V", v_phase_b, 12.0, 0.1);
    check("Step3: Phase C near 0V", v_phase_c, 0.0, 0.5);
    check("Step3: Phase B current (A)", i_b, expected_I, 0.1);
    check("Step3: Phase C current = Phase B", i_c, i_b, 0.05);

    // === KVL verification: V_phase_a - V_phase_b should equal voltage across windings ===
    // After step 1: V_A ≈ 12V, V_B ≈ 0V, V_AB = 12V
    // Voltage across R_A + R_B = I * (R_A + R_B) = 6A * 2Ω = 12V ✓
    g_ah->set_parameter("voltage", 12.0);
    g_bl->set_parameter("voltage", 12.0);
    g_al->set_parameter("voltage", 0.0);
    g_bh->set_parameter("voltage", 0.0);
    g_ch->set_parameter("voltage", 0.0);
    g_cl->set_parameter("voltage", 0.0);

    sim.step(0.001);

    float v_ab = AH->get_pins()[2]->voltage - BH->get_pins()[2]->voltage;
    i_a = R_A->get_pins()[0]->current;
    float v_winding_total = i_a * 2.0f; // I * (R_A + R_B)
    check("KVL: V_AB = I * (R_A + R_B)", v_ab, v_winding_total, 0.05);
}

// ============================================================================
// Test 15: ESC with Motor Load (inductive windings)
// Same 3-phase inverter but with inductors for motor windings
// Verifies current builds up over time (L di/dt = V)
// ============================================================================
void test_esc_motor_load() {
    std::cout << "\n=== Test 15: ESC with Inductive Motor Load ===" << std::endl;

    CircuitSimulator sim;

    auto* bat = sim.add_component<DCVoltageSource>("BAT", 12.0f);
    auto* gnd = sim.add_component<Ground>("GND");
    sim.connect("w_bat_gnd", "BAT", "GND", "GND", "GND");

    // Motor winding: R (0.5Ω) + L (0.5mH) per phase
    auto* R_A = sim.add_component<Resistor>("R_A", 0.5f);
    auto* L_A = sim.add_component<Inductor>("L_A", 0.5e-3f);
    auto* R_B = sim.add_component<Resistor>("R_B", 0.5f);
    auto* L_B = sim.add_component<Inductor>("L_B", 0.5e-3f);

    // Half-bridge A: AH + AL
    auto* AH = sim.add_component<MOSFETTransistor>("AH",
        MOSFETTransistor::NChannel, 2.0f, 100.0f);
    auto* AL = sim.add_component<MOSFETTransistor>("AL",
        MOSFETTransistor::NChannel, 2.0f, 100.0f);

    // Half-bridge B: BH + BL
    auto* BH = sim.add_component<MOSFETTransistor>("BH",
        MOSFETTransistor::NChannel, 2.0f, 100.0f);
    auto* BL = sim.add_component<MOSFETTransistor>("BL",
        MOSFETTransistor::NChannel, 2.0f, 100.0f);

    // Gate drives: AH ON, BL ON
    auto* g_ah = sim.add_component<DCVoltageSource>("G_AH", 12.0f);
    auto* g_al = sim.add_component<DCVoltageSource>("G_AL", 0.0f);
    auto* g_bh = sim.add_component<DCVoltageSource>("G_BH", 0.0f);
    auto* g_bl = sim.add_component<DCVoltageSource>("G_BL", 12.0f);

    // Gate resistors (0.1Ω each)
    auto* Rg_ah = sim.add_component<Resistor>("RgAH", 0.1f);
    auto* Rg_al = sim.add_component<Resistor>("RgAL", 0.1f);
    auto* Rg_bh = sim.add_component<Resistor>("RgBH", 0.1f);
    auto* Rg_bl = sim.add_component<Resistor>("RgBL", 0.1f);

    // Gate connections (high-side: bootstrap referenced to source/phase)
    sim.connect("w_ah_gs", "G_AH", "V+", "RgAH", "1");
    sim.connect("w_ah_gm", "RgAH", "2", "AH", "gate");
    sim.connect("w_ah_gg", "G_AH", "GND", "AH", "source"); // bootstrap
    sim.connect("w_al_gs", "G_AL", "V+", "RgAL", "1");
    sim.connect("w_al_gm", "RgAL", "2", "AL", "gate");
    sim.connect("w_al_gg", "G_AL", "GND", "GND", "GND");
    sim.connect("w_bh_gs", "G_BH", "V+", "RgBH", "1");
    sim.connect("w_bh_gm", "RgBH", "2", "BH", "gate");
    sim.connect("w_bh_gg", "G_BH", "GND", "BH", "source"); // bootstrap
    sim.connect("w_bl_gs", "G_BL", "V+", "RgBL", "1");
    sim.connect("w_bl_gm", "RgBL", "2", "BL", "gate");
    sim.connect("w_bl_gg", "G_BL", "GND", "GND", "GND");

    // Power: BAT → high-side drains
    sim.connect("w_ah_drain", "BAT", "V+", "AH", "drain");
    sim.connect("w_bh_drain", "BAT", "V+", "BH", "drain");

    // Phase nodes
    sim.connect("w_phase_a", "AH", "source", "AL", "drain");
    sim.connect("w_phase_b", "BH", "source", "BL", "drain");

    // Low-side to GND
    sim.connect("w_al_gnd2", "AL", "source", "GND", "GND");
    sim.connect("w_bl_gnd2", "BL", "source", "GND", "GND");

    // Motor windings: Phase A → R_A → L_A → star point → L_B → R_B → Phase B
    sim.connect("w_ra1", "AH", "source", "R_A", "1");
    sim.connect("w_la", "R_A", "2", "L_A", "1");
    sim.connect("w_lb", "L_A", "2", "L_B", "1");
    sim.connect("w_rb2", "L_B", "2", "R_B", "1");
    sim.connect("w_rb1", "R_B", "2", "BH", "source");

    // Step 1: initial current should be low (inductor opposes change)
    sim.step(0.001);
    float i_initial = R_A->get_pins()[0]->current;

    // Run more steps: current should build up toward V/R = 12/(0.5+0.5) = 12A
    for (int i = 0; i < 50; i++) {
        sim.step(0.001);
    }
    float i_steady = R_A->get_pins()[0]->current;

    // After 50ms (50 steps of 1ms), current should be near steady state
    // Time constant τ = L/R = 0.5mH / 1Ω = 0.5ms
    // After 50τ, current should be fully settled
    check("Inductive load: steady-state current (A)", i_steady, 12.0, 0.15);
    check("Inductive load: initial < steady", i_initial < i_steady ? 1.0 : 0.0, 1.0);

    // KVL: V_supply ≈ I*(R_A+R_B) + V_L_A + V_L_B
    float v_supply = bat->get_pins()[0]->voltage;
    float v_resistive = i_steady * 1.0f; // I * (R_A + R_B)
    check("KVL: V_supply = I*R_total (at steady state)", v_supply, v_resistive, 0.1);
}

// ============================================================================
// Main
// ============================================================================
int main() {
    std::cout << "============================================" << std::endl;
    std::cout << "=== MNA Circuit Simulation Test Suite ===" << std::endl;
    std::cout << "============================================" << std::endl;

    test_simple_ohms_law();
    test_voltage_divider();
    test_parallel_resistors();
    test_three_parallel_paths();
    test_kirchhoff_voltage_law();
    test_kirchhoff_current_law();
    test_current_division();
    test_led_circuit();
    test_capacitor_dc_steady();
    test_wheatstone_bridge();
    test_voltage_source_current();
    test_multiple_sources();

    test_esc_half_bridge();

    std::cout << "\n============================================" << std::endl;
    std::cout << "Results: " << tests_passed << " passed, " << tests_failed << " failed" << std::endl;
    std::cout << "============================================" << std::endl;

    return tests_failed > 0 ? 1 : 0;
}
