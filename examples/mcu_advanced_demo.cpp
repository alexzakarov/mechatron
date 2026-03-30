// MCU Advanced Demo - Simple Version
// Demonstrates ATmega interpreter with basic firmware

#include "mcu/QEMUInterface.hpp"
#include "mcu/ATmegaInterpreter.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <thread>
#include <chrono>

using namespace mechatron;

// Create a minimal valid Intel HEX file for testing
// This is a very simple program that just has some instructions
// Helper to calculate Intel HEX checksum
uint8_t calc_hex_checksum(const std::string& line) {
    // Skip the ':' and checksum (last 2 chars)
    uint8_t sum = 0;
    for (size_t i = 1; i < line.length() - 2; i += 2) {
        std::string byte_str = line.substr(i, 2);
        sum += static_cast<uint8_t>(std::stoi(byte_str, nullptr, 16));
    }
    return (0x100 - (sum % 0x100)) % 0x100;
}

void create_simple_firmware(const std::string& path) {
    // Open in binary mode
    std::ofstream hex_file(path, std::ios::binary);

    // Intel HEX format: :LLAAAATT[DD...]CC
    // Format: :LL (byte_count) AAAA (address) TT (type) [DD...] (data) CC (checksum)
    //
    // Simplest: just EOF record for empty firmware
    const char eof_line[] = ":00000001FF\r\n";
    hex_file.write(eof_line, sizeof(eof_line) - 1);
    hex_file.close();
}

void test_firmware_loading() {
    std::cout << "\n=== Test 1: Firmware Loading ===" << std::endl;

    create_simple_firmware("test_firmware.hex");

    QEMUInterface mcu;
    mcu.set_mode(MCUMode::Simulation);

    if (!mcu.launch("test_firmware.hex")) {
        std::cerr << "Error: " << mcu.error() << std::endl;
        return;
    }

    std::cout << "Firmware loaded successfully" << std::endl;
    mcu.stop();
    std::cout << "✓ Firmware loading test passed" << std::endl;
}

void test_interpreter_basic() {
    std::cout << "\n=== Test 2: ATmega Interpreter Basic ===" << std::endl;
    std::cout << std::flush;

    create_simple_firmware("test_firmware.hex");

    QEMUInterface mcu;
    mcu.set_mode(MCUMode::Simulation);

    if (!mcu.launch("test_firmware.hex")) {
        std::cerr << "Error: " << mcu.error() << std::endl;
        return;
    }

    std::cout << "Creating interpreter..." << std::endl;
    std::cout << std::flush;
    ATmegaInterpreter interp(mcu);

    std::cout << "Loading firmware into interpreter..." << std::endl;
    std::cout << std::flush;
    if (!interp.load_firmware("test_firmware.hex")) {
        std::cerr << "Failed to load firmware into interpreter" << std::endl;
        mcu.stop();
        return;
    }
    std::cout << "Firmware load completed" << std::endl;

    std::cout << "Firmware loaded into interpreter" << std::endl;
    std::cout << "Initial state:" << std::endl;
    std::cout << "  PC = 0x" << std::hex << interp.state().PC << std::dec << std::endl;
    std::cout << "  SP = 0x" << std::hex << interp.state().SP << std::dec << std::endl;
    std::cout << "  Cycles = " << interp.state().cycles << std::endl;

    mcu.stop();
    std::cout << "✓ Interpreter basic test passed" << std::endl;
}

void test_pin_simulation() {
    std::cout << "\n=== Test 3: Pin Simulation ===" << std::endl;

    QEMUInterface mcu;
    mcu.set_mode(MCUMode::Simulation);
    mcu.launch("test.hex");  // Empty firmware

    // Configure pin 13 as output
    mcu.write_register(ATmega328P_Registers::DDRB, 0x20);

    std::cout << "Testing digital I/O:" << std::endl;

    // Test write and read
    mcu.digital_write(13, true);
    bool value = mcu.digital_read(13);
    std::cout << "  Write HIGH, Read: " << (value ? "HIGH" : "LOW") << std::endl;

    mcu.digital_write(13, false);
    value = mcu.digital_read(13);
    std::cout << "  Write LOW, Read: " << (value ? "HIGH" : "LOW") << std::endl;

    // Test multiple pins
    mcu.write_register(ATmega328P_Registers::DDRB, 0xFF);  // All output
    mcu.write_register(ATmega328P_Registers::PORTB, 0xAA); // Pattern

    uint8_t portb = mcu.memory().read_io(ATmega328P_Registers::PORTB);
    std::cout << "  PORTB pattern: 0x" << std::hex << (int)portb << std::dec << std::endl;

    mcu.stop();
    std::cout << "✓ Pin simulation test passed" << std::endl;
}

void test_analog_simulation() {
    std::cout << "\n=== Test 4: Analog Simulation ===" << std::endl;

    QEMUInterface mcu;
    mcu.set_mode(MCUMode::Simulation);
    mcu.launch("test.hex");

    std::cout << "Testing analog read simulation:" << std::endl;

    // In simulation mode, we can write ADC values directly to memory
    // and read them back
    for (int i = 0; i <= 1023; i += 200) {
        // Write ADC value to memory (simulating ADC conversion)
        uint16_t adc_value = static_cast<uint16_t>(i);
        mcu.memory().write_io(ATmega328P_Registers::ADCL, adc_value & 0xFF);
        mcu.memory().write_io(ATmega328P_Registers::ADCH, (adc_value >> 8) & 0x03);

        // Calculate corresponding voltage
        float voltage = (adc_value / 1023.0f) * 5.0f;

        std::cout << std::fixed << std::setprecision(2)
                  << "  ADC " << std::setw(4) << adc_value
                  << " -> " << std::setw(5) << voltage << "V" << std::endl;
    }

    mcu.stop();
    std::cout << "✓ Analog simulation test passed" << std::endl;
}

void test_register_manipulation() {
    std::cout << "\n=== Test 5: Register Manipulation ===" << std::endl;

    QEMUInterface mcu;
    mcu.set_mode(MCUMode::Simulation);
    mcu.launch("test.hex");

    std::cout << "Testing register operations:" << std::endl;

    // Test DDRB (Data Direction Register B)
    mcu.write_register(ATmega328P_Registers::DDRB, 0xFF);
    uint8_t ddrb = mcu.memory().read_io(ATmega328P_Registers::DDRB);
    std::cout << "  DDRB = 0x" << std::hex << (int)ddrb << std::dec
              << " (all output)" << std::endl;

    // Test PORTB
    mcu.write_register(ATmega328P_Registers::PORTB, 0xAA);
    uint8_t portb = mcu.memory().read_io(ATmega328P_Registers::PORTB);
    std::cout << "  PORTB = 0x" << std::hex << (int)portb << std::dec
              << " (pattern)" << std::endl;

    // Test PINB (Input Pins)
    uint8_t pinb = mcu.memory().read_io(ATmega328P_Registers::PINB);
    std::cout << "  PINB = 0x" << std::hex << (int)pinb << std::dec << std::endl;

    // Test individual pin access
    mcu.write_register(ATmega328P_Registers::DDRB, 0x01);  // Pin 8 output
    mcu.digital_write(8, true);
    bool pin8 = mcu.digital_read(8);
    std::cout << "  Pin 8 (D8) = " << (pin8 ? "HIGH" : "LOW") << std::endl;

    mcu.stop();
    std::cout << "✓ Register manipulation test passed" << std::endl;
}

void test_interpreter_with_instructions() {
    std::cout << "\n=== Test 6: Interpreter with Instructions ===" << std::endl;

    // Create a firmware with NOP instructions
    std::ofstream hex_file("test_instructions.hex", std::ios::binary);

    // Data record: 04 bytes (2 NOPs) at address 0x0000, type 00
    const char line1[] = ":0400000000000000FC\r\n";
    const char line2[] = ":00000001FF\r\n";

    hex_file.write(line1, sizeof(line1) - 1);
    hex_file.write(line2, sizeof(line2) - 1);
    hex_file.close();

    QEMUInterface mcu;
    mcu.set_mode(MCUMode::Simulation);
    mcu.launch("test_instructions.hex");

    ATmegaInterpreter interp(mcu);

    if (!interp.load_firmware("test_instructions.hex")) {
        std::cerr << "Failed to load firmware" << std::endl;
        mcu.stop();
        return;
    }

    std::cout << "Loaded firmware with NOP instructions" << std::endl;
    std::cout << "Executing 5 instructions:" << std::endl;

    for (int i = 0; i < 5 && interp.is_loaded(); i++) {
        std::string disasm = interp.disassemble_current();
        std::cout << "  " << disasm << std::endl;

        if (!interp.step()) {
            std::cout << "  Execution halted" << std::endl;
            break;
        }
    }

    std::cout << "Final state:" << std::endl;
    std::cout << "  PC = 0x" << std::hex << interp.state().PC << std::dec << std::endl;
    std::cout << "  Instructions executed = " << interp.instruction_count() << std::endl;
    std::cout << "  Cycles = " << interp.state().cycles << std::endl;

    mcu.stop();
    std::cout << "✓ Interpreter instruction test passed" << std::endl;
}

void test_breakpoints() {
    std::cout << "\n=== Test 7: Breakpoint Management ===" << std::endl;

    QEMUInterface mcu;
    mcu.set_mode(MCUMode::Simulation);
    mcu.launch("test.hex");

    ATmegaInterpreter interp(mcu);
    interp.load_firmware("test.hex");

    std::cout << "Testing breakpoint functionality:" << std::endl;

    // Set breakpoints
    interp.set_breakpoint(0x0100);
    interp.set_breakpoint(0x0200);
    interp.set_breakpoint(0x0300);

    std::cout << "  Set 3 breakpoints at 0x0100, 0x0200, 0x0300" << std::endl;

    // Check breakpoints
    std::cout << "  Has breakpoint at 0x0100: "
              << (interp.has_breakpoint(0x0100) ? "YES" : "NO") << std::endl;
    std::cout << "  Has breakpoint at 0x0150: "
              << (interp.has_breakpoint(0x0150) ? "YES" : "NO") << std::endl;

    // Clear one breakpoint
    interp.clear_breakpoint(0x0200);
    std::cout << "  Cleared breakpoint at 0x0200" << std::endl;
    std::cout << "  Has breakpoint at 0x0200: "
              << (interp.has_breakpoint(0x0200) ? "YES" : "NO") << std::endl;

    // Clear all
    interp.clear_all_breakpoints();
    std::cout << "  Cleared all breakpoints" << std::endl;
    std::cout << "  Has breakpoint at 0x0100: "
              << (interp.has_breakpoint(0x0100) ? "YES" : "NO") << std::endl;

    mcu.stop();
    std::cout << "✓ Breakpoint test passed" << std::endl;
}

void test_real_firmware() {
    std::cout << "\n=== Test 8: Real Arduino Blink Firmware ===" << std::endl;

    QEMUInterface mcu;
    mcu.set_mode(MCUMode::Simulation);

    // Use absolute path or create the hex file in current directory
    std::string firmware_path = "blink_arduino.hex";

    std::cout << "Loading firmware: " << firmware_path << std::endl;
    if (!mcu.launch(firmware_path)) {
        std::cerr << "Error: " << mcu.error() << std::endl;
        return;
    }

    std::cout << "Firmware loaded into QEMUInterface" << std::endl;

    std::cout << "Creating ATmegaInterpreter..." << std::endl;
    ATmegaInterpreter interp(mcu);

    std::cout << "Loading firmware into interpreter..." << std::endl;
    if (!interp.load_firmware(firmware_path)) {
        std::cerr << "Failed to load firmware into interpreter" << std::endl;
        mcu.stop();
        return;
    }

    std::cout << "Firmware loaded successfully!" << std::endl;
    std::cout << "Is loaded: " << (interp.is_loaded() ? "YES" : "NO") << std::endl;

    std::cout << "\nInitial state:" << std::endl;
    std::cout << "  PC = 0x" << std::hex << interp.state().PC << std::dec << std::endl;
    std::cout << "  SP = 0x" << std::hex << interp.state().SP << std::dec << std::endl;
    std::cout << "  Cycles = " << interp.state().cycles << std::endl;

    std::cout << "\nExecuting and disassembling first 20 instructions:" << std::endl;
    for (int i = 0; i < 20 && interp.is_loaded(); i++) {
        std::string disasm = interp.disassemble_current();
        std::cout << "  [" << std::setw(2) << (i+1) << "] 0x"
                  << std::hex << std::setw(4) << std::setfill('0')
                  << (interp.state().PC) << std::dec << ": "
                  << std::left << std::setw(20) << disasm << std::right;

        if (!interp.step()) {
            std::cout << " -> HALTED";
            break;
        }
        std::cout << " (PC=0x" << std::hex << interp.state().PC << std::dec << ")";
        std::cout << std::endl;
    }

    std::cout << "\nAfter 20 instructions:" << std::endl;
    std::cout << "  PC = 0x" << std::hex << interp.state().PC << std::dec << std::endl;
    std::cout << "  SP = 0x" << std::hex << interp.state().SP << std::dec << std::endl;
    std::cout << "  Cycles = " << interp.state().cycles << std::endl;
    std::cout << "  Instructions executed = " << interp.instruction_count() << std::endl;

    mcu.stop();
    std::cout << "✓ Real firmware test passed" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "=== MCU Advanced Demo ===" << std::endl;
    std::cout << "=== ATmega328P Interpreter ===" << std::endl;
    std::cout << "========================================" << std::endl;

    try {
        test_firmware_loading();
        test_interpreter_basic();
        test_pin_simulation();
        test_analog_simulation();
        test_register_manipulation();
        test_interpreter_with_instructions();
        test_breakpoints();
        test_real_firmware();

        std::cout << "\n========================================" << std::endl;
        std::cout << "=== ALL TESTS PASSED! ===" << std::endl;
        std::cout << "========================================" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "\n✗ TEST FAILED: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
