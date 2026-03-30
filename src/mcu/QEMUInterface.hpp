#pragma once

#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <array>
#include <map>

#ifdef _WIN32
#include <winsock2.h>
#endif

namespace mechatron {

// Operation mode for MCU emulation
enum class MCUMode {
    Simulation,    // Internal simulation (no QEMU required)
    QEMU,          // Real QEMU emulation
    Hybrid         // Simulation with QEMU fallback
};

// Pin state representation
struct PinState {
    uint8_t port;      // Port letter (A=0, B=1, C=2, etc.)
    uint8_t bit;       // Bit number (0-7)
    bool value;        // Digital state (true=HIGH, false=LOW)
    float analog_value; // Analog value (0.0-5.0V for ADC pins)
    bool is_analog;    // True if this is an analog read
    bool is_pwm;       // True if PWM output
    uint8_t pwm_duty;  // PWM duty cycle (0-255)

    PinState() : port(0), bit(0), value(false), analog_value(0.0f),
                 is_analog(false), is_pwm(false), pwm_duty(0) {}

    uint8_t to_mask() const { return 1 << bit; }
};

// ATmega328P register addresses (I/O memory)
namespace ATmega328P_Registers {
    constexpr uint16_t PORTB = 0x25;
    constexpr uint16_t DDRB  = 0x24;
    constexpr uint16_t PINB  = 0x23;

    constexpr uint16_t PORTC = 0x28;
    constexpr uint16_t DDRC  = 0x27;
    constexpr uint16_t PINC  = 0x26;

    constexpr uint16_t PORTD = 0x2B;
    constexpr uint16_t DDRD  = 0x2A;
    constexpr uint16_t PIND  = 0x29;

    constexpr uint16_t ADCW  = 0x78;  // ADC result low/high
    constexpr uint16_t ADMUX = 0x7C;
    constexpr uint16_t ADCSRA = 0x7A;
    constexpr uint16_t ADCL  = 0x78;
    constexpr uint16_t ADCH  = 0x79;

    // Timer registers for PWM
    constexpr uint16_t OCR0A = 0x47;
    constexpr uint16_t OCR0B = 0x48;
    constexpr uint16_t OCR1A = 0x88;
    constexpr uint16_t OCR1B = 0x89;
    constexpr uint16_t OCR2A = 0xB3;
    constexpr uint16_t OCR2B = 0xB4;
}

// MCU memory state (for simulation mode)
struct MCUMemory {
    std::array<uint8_t, 256> io_registers;  // I/O registers 0x00-0xFF
    std::array<uint8_t, 2048> sram;         // Internal SRAM
    std::array<uint8_t, 32> gp_registers;   // General purpose registers R0-R31

    MCUMemory() {
        io_registers.fill(0);
        sram.fill(0);
        gp_registers.fill(0);

        // Initialize DDR registers to inputs (0)
        io_registers[ATmega328P_Registers::DDRB & 0xFF] = 0;
        io_registers[ATmega328P_Registers::DDRC & 0xFF] = 0;
        io_registers[ATmega328P_Registers::DDRD & 0xFF] = 0;
    }

    uint8_t read_io(uint16_t addr) const {
        return io_registers[addr & 0xFF];
    }

    void write_io(uint16_t addr, uint8_t value) {
        io_registers[addr & 0xFF] = value;
    }
};

// QEMU subprocess interface
class QEMUInterface {
public:
    QEMUInterface();
    ~QEMUInterface();

    // Set operation mode
    void set_mode(MCUMode mode) { m_mode = mode; }
    MCUMode mode() const { return m_mode; }

    // Launch QEMU process with firmware
    bool launch(const std::string& firmware_path, const std::string& machine_type = "arduino-uno");

    // Stop QEMU process
    void stop();

    // Check if QEMU is running
    bool is_running() const { return m_running; }

    // Pin operations
    bool pin_read(uint8_t port, uint8_t bit, bool& value);
    bool pin_write(uint8_t port, uint8_t bit, bool value);

    // Analog operations
    bool analog_read(uint8_t pin, float& voltage);
    bool analog_write(uint8_t pin, float voltage);  // For DAC/PWM

    // Register access (for simulation mode)
    bool read_register(uint16_t addr, uint8_t& value);
    bool write_register(uint16_t addr, uint8_t value);

    // Port access (Arduino style)
    bool digital_read(uint8_t arduino_pin);
    void digital_write(uint8_t arduino_pin, bool value);
    int analog_read(uint8_t analog_pin);

    // Step emulation (run N instructions)
    bool step_instructions(uint32_t count);

    // Update simulation (call each frame)
    void update_simulation(double dt);

    // Get last error message
    const std::string& error() const { return m_error; }

    // Get memory state (simulation mode only)
    const MCUMemory& memory() const { return m_memory; }
    MCUMemory& memory() { return m_memory; }

private:
    // Process handle (platform-specific)
    void* m_process_handle;
    std::string m_firmware_path;
    std::string m_machine_type;
    bool m_running;
    std::string m_error;

    // Operation mode
    MCUMode m_mode;

    // Simulation state
    MCUMemory m_memory;
    double m_sim_time;
    std::map<uint8_t, float> m_analog_inputs;  // ADC pin -> voltage

    // Communication channels
#ifdef _WIN32
    SOCKET m_monitor_socket;   // QEMU monitor socket
    SOCKET m_serial_socket;    // Serial/UART socket
#else
    int m_monitor_socket;   // QEMU monitor socket
    int m_serial_socket;    // Serial/UART socket
#endif

    // Simulation mode functions
    bool sim_pin_read(uint8_t port, uint8_t bit, bool& value);
    bool sim_pin_write(uint8_t port, uint8_t bit, bool value);
    bool sim_analog_read(uint8_t pin, float& voltage);

    // QEMU mode functions
    bool qemu_read_register(uint16_t addr, uint8_t& value);
    bool qemu_write_register(uint16_t addr, uint8_t value);

    bool send_monitor_command(const std::string& cmd);
    bool read_response(std::string& response);
};

// Arduino Uno specific pin mapping
class ArduinoUno {
public:
    static constexpr uint8_t NUM_DIGITAL_PINS = 14;
    static constexpr uint8_t NUM_ANALOG_PINS = 6;

    // Convert Arduino pin number to port/bit
    static bool pin_to_port_bit(uint8_t pin, uint8_t& port, uint8_t& bit);

    // Convert Arduino pin to ADC channel
    static uint8_t pin_to_adc_channel(uint8_t analog_pin);

    // Port registers
    static constexpr uint8_t PORT_B = 0;  // Digital pins 8-13
    static constexpr uint8_t PORT_C = 1;  // Analog pins 0-5
    static constexpr uint8_t PORT_D = 2;  // Digital pins 0-7

    // PWM pins
    static bool is_pwm_pin(uint8_t pin);
    static uint8_t get_pwm_timer(uint8_t pin);  // Returns 0, 1, or 2
};

} // namespace mechatron
