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

// MCU variant register map - configurable per MCU type
struct MCUVariant {
    std::string name;
    uint16_t PORTB = 0x25, DDRB = 0x24, PINB = 0x23;
    uint16_t PORTC = 0x28, DDRC = 0x27, PINC = 0x26;
    uint16_t PORTD = 0x2B, DDRD = 0x2A, PIND = 0x29;
    uint16_t ADCW = 0x78, ADMUX = 0x7C, ADCSRA = 0x7A, ADCL = 0x78, ADCH = 0x79;
    uint16_t OCR0A = 0x47, OCR0B = 0x48;
    uint16_t OCR1A = 0x88, OCR1B = 0x8A;
    uint16_t OCR2A = 0xB3, OCR2B = 0xB4;
    // Timer0 registers (data space). Used by interpreter timing and PWM detection.
    uint16_t TCCR0A = 0x44, TCCR0B = 0x45, TCNT0 = 0x46;
    uint16_t TIFR0 = 0x35, TIMSK0 = 0x6E;

    // Device sizes / core characteristics
    uint32_t flash_size_words = 16 * 1024; // default 328P: 32KB flash => 16K words
    uint16_t ramstart_data_addr = 0x0100;  // default 328P RAMSTART in data space
    uint16_t ramend_data_addr = 0x08FF;    // default 328P RAMEND in data space
    uint32_t default_clock_hz = 16000000;  // default 16 MHz
    bool supported_by_interpreter = true;
    uint8_t pc_bytes = 2;                 // 2 for <= 128KB flash, 3 for > 128KB (e.g. ATmega2560)

    // Default constructor for ATmega328P (Arduino Uno)
    static MCUVariant atmega328p() {
        return MCUVariant{"ATmega328P"};
    }

    // ATmega2560 (Arduino Mega)
    static MCUVariant atmega2560() {
        MCUVariant v{"ATmega2560"};
        v.PORTB = 0x25; v.DDRB = 0x24; v.PINB = 0x23;
        v.PORTC = 0x28; v.DDRC = 0x27; v.PINC = 0x26;
        // Note: ATmega2560 has additional ports and different addresses
        // Requires 3-byte PC and larger flash.
        v.supported_by_interpreter = true;
        v.flash_size_words = 128 * 1024; // 256KB flash => 128K words (for future)
        v.ramstart_data_addr = 0x0200;    // 8KB SRAM starts after extended I/O
        v.ramend_data_addr = 0x21FF;     // 8KB SRAM RAMEND (for future)
        v.default_clock_hz = 16000000;
        v.pc_bytes = 3;
        return v;
    }

    // ATmega32U4 (Arduino Leonardo/Micro)
    static MCUVariant atmega32u4() {
        MCUVariant v{"ATmega32U4"};
        v.PORTB = 0x25; v.DDRB = 0x24; v.PINB = 0x23;
        // Note: ATmega32U4 has different register layout
        v.supported_by_interpreter = false;
        return v;
    }

    // ATtiny85
    static MCUVariant attiny85() {
        MCUVariant v{"ATtiny85"};
        // iotn85 / iotnx5 IO addresses converted to data space by adding 0x20.
        // PINB=0x16->0x36, DDRB=0x17->0x37, PORTB=0x18->0x38
        v.PINB = 0x36; v.DDRB = 0x37; v.PORTB = 0x38;
        // Tiny85 has only PORTB; leave PORTC/PORTD unused.
        v.PORTC = v.DDRC = v.PINC = 0;
        v.PORTD = v.DDRD = v.PIND = 0;

        // ADC registers: ADMUX IO 0x07->0x27, ADCSRA IO 0x06->0x26, ADCL IO 0x04->0x24, ADCH IO 0x05->0x25
        v.ADMUX = 0x27; v.ADCSRA = 0x26; v.ADCL = 0x24; v.ADCH = 0x25; v.ADCW = 0x24;

        // Timer0: OCR0A IO 0x29->0x49, OCR0B IO 0x28->0x48
        v.OCR0A = 0x49; v.OCR0B = 0x48;
        // Timer0: TCCR0A IO 0x2A->0x4A, TCCR0B IO 0x33->0x53, TCNT0 IO 0x32->0x52
        // TIFR IO 0x18->0x38, TIMSK IO 0x19->0x39 (single timer interrupt mask/flag on tiny85)
        v.TCCR0A = 0x4A; v.TCCR0B = 0x53; v.TCNT0 = 0x52;
        v.TIFR0 = 0x38; v.TIMSK0 = 0x39;
        // Tiny85 has Timer1 too but PWM mapping differs; not modeled yet.
        v.OCR1A = v.OCR1B = 0;
        v.OCR2A = v.OCR2B = 0;

        v.flash_size_words = 4 * 1024;   // 8KB flash => 4K words
        v.ramstart_data_addr = 0x0060;    // from iotn85.h RAMSTART
        v.ramend_data_addr = 0x025F;     // from iotn85.h RAMEND
        v.default_clock_hz = 8000000;    // typical default without fuse changes
        return v;
    }
};

// ATmega328P register addresses (I/O memory) - for backward compatibility
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
    constexpr uint16_t TCNT0 = 0x46;
    constexpr uint16_t TCCR0B = 0x45;
    constexpr uint16_t TIFR0 = 0x35;
    constexpr uint16_t TIMSK0 = 0x6E;
    constexpr uint16_t OCR1A = 0x88;
    constexpr uint16_t OCR1B = 0x8A;
    constexpr uint16_t OCR2A = 0xB3;
    constexpr uint16_t OCR2B = 0xB4;
}

// MCU memory state (for simulation mode)
struct MCUMemory {
    // Data space includes:
    // - GPR:   0x0000-0x001F
    // - I/O:   0x0020-0x00FF (classic)
    // - ExtIO: device-specific range before SRAM
    std::array<uint8_t, 512> io_registers;  // I/O + extended I/O 0x00-0x1FF
    std::array<uint8_t, 8192> sram;         // Internal SRAM (large enough for ATmega2560)
    std::array<uint8_t, 32> gp_registers;   // General purpose registers R0-R31
    uint16_t sram_start_data_addr = 0x0100;

    MCUMemory() {
        reset();
    }

    void reset() {
        io_registers.fill(0);
        sram.fill(0);
        gp_registers.fill(0);

        // Initialize DDR registers to inputs (0)
        io_registers[ATmega328P_Registers::DDRB & 0xFF] = 0;
        io_registers[ATmega328P_Registers::DDRC & 0xFF] = 0;
        io_registers[ATmega328P_Registers::DDRD & 0xFF] = 0;
    }

    void configure_for_variant(const MCUVariant& variant) {
        sram_start_data_addr = variant.ramstart_data_addr;
        reset();
    }

    uint8_t read_io(uint16_t addr) const {
        return io_registers[addr & 0x1FF];
    }

    void write_io(uint16_t addr, uint8_t value) {
        io_registers[addr & 0x1FF] = value;
    }

    uint8_t read_data(uint16_t addr) const {
        if (addr < 0x20) {
            return gp_registers[addr];
        }
        if (addr < sram_start_data_addr) {
            return io_registers[addr & 0x1FF];
        }
        uint16_t offset = addr - sram_start_data_addr;
        return offset < sram.size() ? sram[offset] : 0;
    }

    void write_data(uint16_t addr, uint8_t value) {
        if (addr < 0x20) {
            gp_registers[addr] = value;
            return;
        }
        if (addr < sram_start_data_addr) {
            io_registers[addr & 0x1FF] = value;
            return;
        }
        uint16_t offset = addr - sram_start_data_addr;
        if (offset < sram.size()) {
            sram[offset] = value;
        }
    }
};

// QEMU subprocess interface
class QEMUInterface {
public:
    QEMUInterface();
    ~QEMUInterface();

    // Network configuration (make QEMU addresses/ports configurable)
    void set_monitor_host(const std::string& host) { m_monitor_host = host; }
    void set_monitor_port(int port) { m_monitor_port = port; }
    void set_serial_host(const std::string& host) { m_serial_host = host; }
    void set_serial_port(int port) { m_serial_port = port; }

    const std::string& get_monitor_host() const { return m_monitor_host; }
    int get_monitor_port() const { return m_monitor_port; }
    const std::string& get_serial_host() const { return m_serial_host; }
    int get_serial_port() const { return m_serial_port; }

    // MCU clock frequency configuration
    void set_clock_frequency(uint32_t freq_hz) { m_clock_frequency = freq_hz; }
    uint32_t clock_frequency() const { return m_clock_frequency; }

    // MCU variant configuration
    void set_mcu_variant(const MCUVariant& variant) {
        m_mcu_variant = variant;
        m_memory.configure_for_variant(m_mcu_variant);
    }
    const MCUVariant& mcu_variant() const { return m_mcu_variant; }
    void set_mcu_variant_atmega328p() { set_mcu_variant(MCUVariant::atmega328p()); }
    void set_mcu_variant_atmega2560() { set_mcu_variant(MCUVariant::atmega2560()); }
    void set_mcu_variant_atmega32u4() { set_mcu_variant(MCUVariant::atmega32u4()); }
    void set_mcu_variant_attiny85() { set_mcu_variant(MCUVariant::attiny85()); }

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
    bool set_digital_input(uint8_t arduino_pin, bool value);
    bool is_digital_output(uint8_t arduino_pin) const;
    float digital_output_voltage(uint8_t arduino_pin) const;
    bool is_pwm_enabled(uint8_t arduino_pin) const;
    bool pwm_duty_fraction(uint8_t arduino_pin, float& duty) const;
    double pwm_frequency_hz(uint8_t arduino_pin) const;
    bool pwm_output_voltage(uint8_t arduino_pin, double time_s, float high_voltage, float low_voltage, float& voltage) const;
    bool set_analog_input(uint8_t analog_pin, float voltage);

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
    uint32_t m_clock_frequency = 16000000;  // Default: 16 MHz (ATmega328P)
    MCUVariant m_mcu_variant = MCUVariant::atmega328p();  // Default MCU variant
    std::map<uint8_t, float> m_analog_inputs;  // ADC pin -> voltage

    // Network configuration (configurable instead of hardcoded)
    std::string m_monitor_host = "127.0.0.1";
    int m_monitor_port = 4444;
    std::string m_serial_host = "127.0.0.1";
    int m_serial_port = 4445;

    // Communication channels
#ifdef _WIN32
    SOCKET m_monitor_socket = INVALID_SOCKET;   // QEMU monitor socket
    SOCKET m_serial_socket = INVALID_SOCKET;    // Serial/UART socket
#else
    int m_monitor_socket = -1;   // QEMU monitor socket
    int m_serial_socket = -1;    // Serial/UART socket
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
