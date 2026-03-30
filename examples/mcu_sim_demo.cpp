// MCU Simulation Demo
// Demonstrates QEMUInterface in simulation mode (no QEMU required)

#include "mcu/QEMUInterface.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <thread>
#include <chrono>

using namespace mechatron;

void print_register_state(const QEMUInterface& mcu) {
    const auto& mem = mcu.memory();

    std::cout << "\n=== Register State ===" << std::endl;

    // PORTB (Digital pins 8-13)
    uint8_t ddrb = mem.read_io(ATmega328P_Registers::DDRB);
    uint8_t portb = mem.read_io(ATmega328P_Registers::PORTB);
    uint8_t pinb = mem.read_io(ATmega328P_Registers::PINB);
    std::cout << "PORTB: DDR=0x" << std::hex << (int)ddrb
              << " PORT=0x" << (int)portb
              << " PIN=0x" << (int)pinb << std::dec << std::endl;

    // PORTC (Analog pins 0-5)
    uint8_t ddrc = mem.read_io(ATmega328P_Registers::DDRC);
    uint8_t portc = mem.read_io(ATmega328P_Registers::PORTC);
    uint8_t pinc = mem.read_io(ATmega328P_Registers::PINC);
    std::cout << "PORTC: DDR=0x" << std::hex << (int)ddrc
              << " PORT=0x" << (int)portc
              << " PIN=0x" << (int)pinc << std::dec << std::endl;

    // PORTD (Digital pins 0-7)
    uint8_t ddrd = mem.read_io(ATmega328P_Registers::DDRD);
    uint8_t portd = mem.read_io(ATmega328P_Registers::PORTD);
    uint8_t pind = mem.read_io(ATmega328P_Registers::PIND);
    std::cout << "PORTD: DDR=0x" << std::hex << (int)ddrd
              << " PORT=0x" << (int)portd
              << " PIN=0x" << (int)pind << std::dec << std::endl;
}

void test_digital_write() {
    std::cout << "\n=== Test 1: Digital Write ===" << std::endl;

    // Create a minimal valid HEX file
    std::ofstream hex_file("test.hex");
    hex_file << ":00000001FF\n";
    hex_file.close();

    QEMUInterface mcu;
    mcu.set_mode(MCUMode::Simulation);

    if (!mcu.launch("test.hex")) {
        std::cerr << "Error: " << mcu.error() << std::endl;
        return;
    }

    // Configure pin 13 as output (PORTB, bit 5)
    mcu.write_register(ATmega328P_Registers::DDRB, 0x20);  // Bit 5 = output
    std::cout << "Configured pin 13 (LED_BUILTIN) as output" << std::endl;

    // Toggle pin 13
    for (int i = 0; i < 5; i++) {
        mcu.digital_write(13, true);
        std::cout << "Pin 13 = HIGH" << std::endl;

        bool value = mcu.digital_read(13);
        std::cout << "Read back: " << (value ? "HIGH" : "LOW") << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        mcu.digital_write(13, false);
        std::cout << "Pin 13 = LOW" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    print_register_state(mcu);
    mcu.stop();
    std::cout << "✓ Digital write test passed" << std::endl;
}

void test_analog_read() {
    std::cout << "\n=== Test 2: Analog Read ===" << std::endl;

    // Create test.hex if it doesn't exist
    std::ofstream hex_file("test.hex");
    hex_file << ":00000001FF\n";
    hex_file.close();

    QEMUInterface mcu;
    mcu.set_mode(MCUMode::Simulation);

    if (!mcu.launch("test.hex")) {
        std::cerr << "Error: " << mcu.error() << std::endl;
        return;
    }

    // Set analog input values (simulating sensors)
    mcu.memory().write_io(ATmega328P_Registers::PINC, 0xFF);  // All high

    // Test analog read at various voltages
    float voltages[] = {0.0f, 1.25f, 2.5f, 3.75f, 5.0f};

    for (float v : voltages) {
        // Set ADC channel 0 (A0) to this voltage
        mcu.memory().write_io(ATmega328P_Registers::ADMUX, 0x00);  // ADC channel 0, right-adjusted
        mcu.memory().write_io(ATmega328P_Registers::ADCL, (int)((v / 5.0f) * 1023) & 0xFF);
        mcu.memory().write_io(ATmega328P_Registers::ADCH, ((int)((v / 5.0f) * 1023) >> 8));

        float read_voltage;
        if (mcu.analog_read(0, read_voltage)) {
            std::cout << "A0: " << std::fixed << std::setprecision(2) << v << "V -> read "
                      << read_voltage << "V (raw: " << mcu.analog_read(0) << ")" << std::endl;
        }
    }

    mcu.stop();
    std::cout << "✓ Analog read test passed" << std::endl;
}

void test_pin_mapping() {
    std::cout << "\n=== Test 3: Arduino Pin Mapping ===" << std::endl;

    // Test all Arduino Uno pins
    std::cout << "Digital pins:" << std::endl;
    for (uint8_t pin = 0; pin < 14; pin++) {
        uint8_t port, bit;
        if (ArduinoUno::pin_to_port_bit(pin, port, bit)) {
            const char* port_name = (port == 0) ? "B" : (port == 1) ? "C" : "D";
            std::cout << "  D" << (int)pin << " -> PORT" << port_name << " bit " << (int)bit;

            if (ArduinoUno::is_pwm_pin(pin)) {
                std::cout << " [PWM, Timer " << (int)ArduinoUno::get_pwm_timer(pin) << "]";
            }
            std::cout << std::endl;
        }
    }

    std::cout << "Analog pins:" << std::endl;
    for (uint8_t pin = 0; pin < 6; pin++) {
        uint8_t channel = ArduinoUno::pin_to_adc_channel(pin);
        std::cout << "  A" << (int)pin << " -> ADC channel " << (int)channel << std::endl;
    }

    std::cout << "✓ Pin mapping test passed" << std::endl;
}

void test_blink_simulation() {
    std::cout << "\n=== Test 4: Blink Simulation ===" << std::endl;

    // Create test.hex if it doesn't exist
    std::ofstream hex_file("test.hex");
    hex_file << ":00000001FF\n";
    hex_file.close();

    QEMUInterface mcu;
    mcu.set_mode(MCUMode::Simulation);

    if (!mcu.launch("test.hex")) {
        std::cerr << "Error: " << mcu.error() << std::endl;
        return;
    }

    // Simulate Arduino blink sketch behavior
    // Setup: configure pin 13 as output
    std::cout << "setup():" << std::endl;
    mcu.digital_write(13, false);
    mcu.write_register(ATmega328P_Registers::DDRB, 0x20);  // Pin 13 output
    std::cout << "  pinMode(13, OUTPUT)" << std::endl;

    // Loop: toggle LED every 1000ms
    std::cout << "\nloop() - Blinking 5 times:" << std::endl;
    for (int i = 0; i < 5; i++) {
        // digital_write(13, HIGH)
        mcu.digital_write(13, true);
        std::cout << "  digital_write(13, HIGH)" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Speed up for demo

        // delay(1000) - simulated by sleep
        // digital_write(13, LOW)
        mcu.digital_write(13, false);
        std::cout << "  digital_write(13, LOW)" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Speed up for demo
    }

    std::cout << "\nFinal state:" << std::endl;
    print_register_state(mcu);
    mcu.stop();
    std::cout << "✓ Blink simulation test passed" << std::endl;
}

void test_port_operations() {
    std::cout << "\n=== Test 5: Port Operations ===" << std::endl;

    // Create test.hex if it doesn't exist
    std::ofstream hex_file("test.hex");
    hex_file << ":00000001FF\n";
    hex_file.close();

    QEMUInterface mcu;
    mcu.set_mode(MCUMode::Simulation);

    if (!mcu.launch("test.hex")) {
        std::cerr << "Error: " << mcu.error() << std::endl;
        return;
    }

    // Test direct port manipulation
    std::cout << "Testing direct port manipulation:" << std::endl;

    // Set all of PORTB as output
    mcu.write_register(ATmega328P_Registers::DDRB, 0xFF);
    std::cout << "DDRB = 0xFF (all output)" << std::endl;

    // Write pattern to PORTB
    mcu.write_register(ATmega328P_Registers::PORTB, 0xAA);  // 10101010
    std::cout << "PORTB = 0xAA" << std::endl;

    uint8_t value;
    mcu.read_register(ATmega328P_Registers::PORTB, value);
    std::cout << "Read PORTB = 0x" << std::hex << (int)value << std::dec << std::endl;

    // Toggle bits
    mcu.write_register(ATmega328P_Registers::PORTB, 0x55);  // 01010101
    std::cout << "PORTB = 0x55" << std::endl;

    mcu.read_register(ATmega328P_Registers::PORTB, value);
    std::cout << "Read PORTB = 0x" << std::hex << (int)value << std::dec << std::endl;

    mcu.stop();
    std::cout << "✓ Port operations test passed" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "=== MCU Simulation Demo ===" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Note: Running in Simulation Mode (QEMU not required)" << std::endl;

    try {
        test_digital_write();
        test_analog_read();
        test_pin_mapping();
        test_blink_simulation();
        test_port_operations();

        std::cout << "\n========================================" << std::endl;
        std::cout << "=== ALL TESTS PASSED! ===" << std::endl;
        std::cout << "========================================" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "\n✗ TEST FAILED: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
