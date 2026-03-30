// Circuit-Physics Bridge Demo
// Shows how circuit voltages control actuators

#include "core/SimulationOrchestrator.hpp"
#include "core/CircuitPhysicsBridge.hpp"
#include "core/Registry.hpp"
#include "electronics/CircuitSimulator.hpp"
#include "actuators/Actuator.hpp"
#include "physics/PhysicsWorld.hpp"
#include <iostream>
#include <iomanip>
#include <memory>

using namespace mechatron;

int main() {
    std::cout << "=== Circuit-Physics Bridge Demo ===" << std::endl;
    std::cout << "Circuit voltages will control actuators\n" << std::endl;

    // Create orchestrator
    SimulationOrchestrator sim;
    auto& physics = sim.physics_world();
    auto& registry = sim.registry();
    physics.set_gravity({0, -9.81f, 0});

    // Create circuit simulator
    CircuitSimulator circuit;

    // Create circuit components using CircuitSimulator
    auto* voltage_source = circuit.add_component<Resistor>("voltage_src", 1000.0f);
    auto* led = circuit.add_component<LED>("led1", 2.0f);

    // Connect bridge to circuit
    sim.circuit_bridge().set_circuit_simulator(&circuit);

    // Create solenoid actuator directly
    auto solenoid = std::make_unique<SolenoidActuator>();
    solenoid->transform().position = {0, 0.1f, 0};
    solenoid->set_resistance(12.0f);
    solenoid->set_stroke(10.0f);
    registry.add(std::move(solenoid), "solenoid1");
    auto* solenoid_ptr = registry.get_as<SolenoidActuator>("solenoid1");

    // Create DC motor directly
    auto motor = std::make_unique<DCMotor>();
    motor->transform().position = {0.5f, 0.1f, 0};
    motor->set_voltage_rating(12.0f);
    registry.add(std::move(motor), "motor1");
    auto* motor_ptr = registry.get_as<DCMotor>("motor1");

    // Create physics bodies
    CollisionShapeDef plunger_shape;
    plunger_shape.type = CollisionShape::Box;
    plunger_shape.box_extents = {0.02f, 0.02f, 0.05f};

    auto* plunger = physics.create_body("plunger", plunger_shape, {0, 0.1f, 0});
    plunger->mass = 0.005f;

    // Ground
    CollisionShapeDef ground_shape;
    ground_shape.type = CollisionShape::Box;
    ground_shape.box_extents = {0.5f, 0.01f, 0.5f};
    auto* ground = physics.create_body("ground", ground_shape, {0, 0, 0});
    ground->is_static = true;

    // Setup pin mappings
    std::cout << "Setting up circuit-bridge mappings..." << std::endl;

    // Mapping 1: Voltage source pin to solenoid input
    PinMapping map1;
    map1.circuit_pin_id = "voltage_src.1";
    map1.target_component_id = "solenoid1";
    map1.type = PinMappingType::VoltageToActuatorInput;
    map1.voltage_min = 0.0f;
    map1.voltage_max = 5.0f;
    sim.circuit_bridge().add_mapping(map1);

    // Mapping 2: LED anode to motor enable (digital)
    PinMapping map2;
    map2.circuit_pin_id = "led1.anode";
    map2.target_component_id = "motor1";
    map2.type = PinMappingType::DigitalToEnable;
    map2.voltage_min = 0.0f;
    map2.voltage_max = 3.0f;
    sim.circuit_bridge().add_mapping(map2);

    std::cout << "Starting simulation..." << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Time(s) | Voltage | Solenoid In | Motor En | Plunger Y | Motor RPM" << std::endl;
    std::cout << "--------|---------|-------------|----------|----------|----------" << std::endl;

    // Simulate for 5 seconds
    const double dt = 0.001;
    const int total_steps = 5000;

    for (int i = 0; i < total_steps; ++i) {
        double time = i * dt;

        // Simulate circuit voltage changes (sine wave)
        float voltage = 2.5f + 2.5f * std::sin(time * 2.0f);

        // Update voltage source pins
        auto vsrc_pins = voltage_source->get_pins();
        if (!vsrc_pins.empty()) {
            vsrc_pins[0]->voltage = voltage;
        }

        // Update LED state
        auto led_pins = led->get_pins();
        if (led_pins.size() >= 2) {
            led_pins[0]->voltage = voltage > 2.5f ? 5.0f : 0.0f;
            led_pins[1]->voltage = 0.0f;
        }
        led->update(dt);

        // Step simulation
        sim.update();

        // Print status every 0.5 seconds
        if (i % 500 == 0) {
            float solenoid_input = solenoid_ptr->get_input();
            bool motor_enabled = motor_ptr->is_enabled();
            float plunger_y = plunger->position.y;
            float motor_rpm = motor_ptr->get_rpm();

            std::cout << std::setw(7) << time << " | "
                      << std::setw(7) << voltage << " | "
                      << std::setw(11) << solenoid_input << " | "
                      << (motor_enabled ? "YES " : "NO  ") << "   | "
                      << std::setw(8) << plunger_y << " | "
                      << std::setw(8) << motor_rpm << std::endl;
        }
    }

    std::cout << "\n--- Final State ---" << std::endl;
    std::cout << "Active bridge mappings: " << sim.circuit_bridge().is_enabled() << std::endl;
    std::cout << "Solenoid position: " << solenoid_ptr->get_position() * 100 << "%" << std::endl;
    std::cout << "Motor RPM: " << motor_ptr->get_rpm() << std::endl;

    std::cout << "\nDemo complete!" << std::endl;

    return 0;
}
