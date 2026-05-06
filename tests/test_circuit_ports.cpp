// Test for CircuitComponentAdapter port mapping
#include "../src/electronics/CircuitComponentAdapter.hpp"
#include "../src/core/Registry.hpp"
#include "../src/plugins/electronics/passive/ElecPassivePlugin.hpp"
#include <cassert>
#include <iostream>

using namespace mechatron;

void test_resistor_ports() {
    auto resistor = std::make_unique<ResistorComponent>(
        std::make_unique<Resistor>(1000.0f)
    );

    auto ports = resistor->get_ports();

    std::cout << "Resistor port count: " << ports.size() << std::endl;
    assert(ports.size() == 2 && "Resistor should have 2 ports");

    // Check port 1
    assert(ports[0]->name() == "1");
    assert(ports[0]->domain() == PortDomain::Analog);
    assert(ports[0]->direction() == PortDirection::Bidirectional);
    std::cout << "Port 1: " << ports[0]->name() << " - OK" << std::endl;

    // Check port 2
    assert(ports[1]->name() == "2");
    assert(ports[1]->domain() == PortDomain::Analog);
    assert(ports[1]->direction() == PortDirection::Bidirectional);
    std::cout << "Port 2: " << ports[1]->name() << " - OK" << std::endl;
}

void test_led_ports() {
    auto led = std::make_unique<LEDComponent>(
        std::make_unique<LED>(2.0f)
    );

    auto ports = led->get_ports();

    std::cout << "\nLED port count: " << ports.size() << std::endl;
    assert(ports.size() == 2 && "LED should have 2 ports");

    // Check anode
    assert(ports[0]->name() == "anode");
    assert(ports[0]->domain() == PortDomain::Analog);
    assert(ports[0]->direction() == PortDirection::Input);
    std::cout << "Port anode: " << ports[0]->name() << " - OK" << std::endl;

    // Check cathode
    assert(ports[1]->name() == "cathode");
    assert(ports[1]->domain() == PortDomain::Analog);
    assert(ports[1]->direction() == PortDirection::Input);
    std::cout << "Port cathode: " << ports[1]->name() << " - OK" << std::endl;
}

void test_bjt_ports() {
    auto bjt = std::make_unique<BJTComponent>(
        std::make_unique<BJTTransistor>(BJTTransistor::NPN)
    );

    auto ports = bjt->get_ports();

    std::cout << "\nBJT port count: " << ports.size() << std::endl;
    assert(ports.size() == 3 && "BJT should have 3 ports");

    // Check base
    assert(ports[0]->name() == "base");
    std::cout << "Port base: " << ports[0]->name() << " - OK" << std::endl;

    // Check collector
    assert(ports[1]->name() == "collector");
    std::cout << "Port collector: " << ports[1]->name() << " - OK" << std::endl;

    // Check emitter
    assert(ports[2]->name() == "emitter");
    std::cout << "Port emitter: " << ports[2]->name() << " - OK" << std::endl;
}

void test_motor_driver_ports() {
    auto driver = std::make_unique<MotorDriverComponent>(
        std::make_unique<MotorDriver>()
    );

    auto ports = driver->get_ports();

    std::cout << "\nMotorDriver port count: " << ports.size() << std::endl;
    assert(ports.size() == 7 && "MotorDriver should have 7 ports");

    // Print all port names
    for (size_t i = 0; i < ports.size(); ++i) {
        std::cout << "Port " << i << ": " << ports[i]->name()
                  << " (domain: " << static_cast<int>(ports[i]->domain()) << ")" << std::endl;
    }

    // Check key ports
    bool has_pwm = false, has_dir = false, has_vcc = false, has_gnd = false;
    for (auto* port : ports) {
        if (port->name() == "PWM") has_pwm = true;
        if (port->name() == "DIR") has_dir = true;
        if (port->name() == "VCC") has_vcc = true;
        if (port->name() == "GND") has_gnd = true;
    }

    assert(has_pwm && "MotorDriver should have PWM port");
    assert(has_dir && "MotorDriver should have DIR port");
    assert(has_vcc && "MotorDriver should have VCC port");
    assert(has_gnd && "MotorDriver should have GND port");
}

int main() {
    std::cout << "=== CircuitComponentAdapter Port Tests ===" << std::endl;

    test_resistor_ports();
    test_led_ports();
    test_bjt_ports();
    test_motor_driver_ports();

    std::cout << "\n=== All tests passed! ===" << std::endl;
    return 0;
}
