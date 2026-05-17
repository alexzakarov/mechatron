#include "QEMUInterface.hpp"
#include "FirmwareLoader.hpp"
#include <spdlog/spdlog.h>
#include <cstring>
#include <algorithm>
#include <cmath>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#define close closesocket
typedef int socklen_t;
#else
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#if defined(__linux__)
#include <sys/prctl.h>
#endif
#endif

namespace mechatron {

namespace {
struct ResolvedGpio {
    uint16_t pin_addr = 0;
    uint16_t ddr_addr = 0;
    uint16_t port_addr = 0;
    uint8_t bit = 0;
    bool valid = false;
};

static ResolvedGpio resolve_mega2560_gpio(uint8_t arduino_pin) {
    // Arduino Mega 2560 mapping derived from Arduino AVR core:
    // tools/arduino-cli/data/packages/arduino/hardware/avr/1.8.7/variants/mega/pins_arduino.h
    // Supports digital pins 0..53 and analog pins A0..A15 as 54..69.
    static const char kPort[70] = {
        'E','E','E','E','G','E','H','H','H','H','B','B','B','B','J','J','H','H','D','D','D','D',
        'A','A','A','A','A','A','A','A','C','C','C','C','C','C','C','C','D','G','G','G',
        'L','L','L','L','L','L','L','L','B','B','B','B',
        'F','F','F','F','F','F','F','F','K','K','K','K','K','K','K','K'
    };
    static const uint8_t kBit[70] = {
        0,1,4,5,5,3,3,4,5,6,4,5,6,7,1,0,1,0,3,2,1,0,
        0,1,2,3,4,5,6,7,7,6,5,4,3,2,1,0,7,2,1,0,
        7,6,5,4,3,2,1,0,3,2,1,0,
        0,1,2,3,4,5,6,7,0,1,2,3,4,5,6,7
    };

    ResolvedGpio r;
    if (arduino_pin >= 70) return r;

    const char port = kPort[arduino_pin];
    const uint8_t bit = kBit[arduino_pin];
    r.bit = bit;

    auto set_io_port = [&](uint16_t pin_io, uint16_t ddr_io, uint16_t port_io) {
        // _SFR_IO8 addresses map to data space by +0x20.
        r.pin_addr = static_cast<uint16_t>(0x20 + pin_io);
        r.ddr_addr = static_cast<uint16_t>(0x20 + ddr_io);
        r.port_addr = static_cast<uint16_t>(0x20 + port_io);
        r.valid = true;
    };

    switch (port) {
        case 'A': set_io_port(0x00, 0x01, 0x02); break;
        case 'B': set_io_port(0x03, 0x04, 0x05); break;
        case 'C': set_io_port(0x06, 0x07, 0x08); break;
        case 'D': set_io_port(0x09, 0x0A, 0x0B); break;
        case 'E': set_io_port(0x0C, 0x0D, 0x0E); break;
        case 'F': set_io_port(0x0F, 0x10, 0x11); break;
        case 'G': set_io_port(0x12, 0x13, 0x14); break;
        // Ports H/J/K/L are in extended I/O space on ATmega2560.
        case 'H': r.pin_addr = 0x100; r.ddr_addr = 0x101; r.port_addr = 0x102; r.valid = true; break;
        case 'J': r.pin_addr = 0x103; r.ddr_addr = 0x104; r.port_addr = 0x105; r.valid = true; break;
        case 'K': r.pin_addr = 0x106; r.ddr_addr = 0x107; r.port_addr = 0x108; r.valid = true; break;
        case 'L': r.pin_addr = 0x109; r.ddr_addr = 0x10A; r.port_addr = 0x10B; r.valid = true; break;
        default: break;
    }

    return r;
}
} // namespace

// Constants
constexpr uint8_t NUM_ANALOG_PINS = 6;

static uint8_t analog_pin_count_for_variant(const MCUVariant& variant) {
    if (variant.name == "ATmega2560") return 16;
    if (variant.name == "ATtiny85") return 4;
    return NUM_ANALOG_PINS;
}

static bool pwm_enabled_for_pin(const MCUVariant& v, const MCUMemory& mem, uint8_t arduino_pin) {
    // Returns true only when the timer output compare is actively driving the pin (COM bits set).
    const bool is_tiny = (v.name == "ATtiny85");
    const bool is_mega = (v.name == "ATmega2560");

    uint16_t tccr_addr = 0;
    uint8_t com_shift = 0;

    if (is_tiny) {
        // ATtiny85: Timer0 PWM on pins 0(OC0A),1(OC0B), TCCR0A=0x4A
        if (arduino_pin == 0) { tccr_addr = 0x4A; com_shift = 6; }
        else if (arduino_pin == 1) { tccr_addr = 0x4A; com_shift = 4; }
        else return false;
    } else if (!is_mega) {
        switch (arduino_pin) {
            case 6:  tccr_addr = 0x44; com_shift = 6; break; // TCCR0A COM0A
            case 5:  tccr_addr = 0x44; com_shift = 4; break; // TCCR0A COM0B
            case 9:  tccr_addr = 0x80; com_shift = 6; break; // TCCR1A COM1A
            case 10: tccr_addr = 0x80; com_shift = 4; break; // TCCR1A COM1B
            case 11: tccr_addr = 0xB0; com_shift = 6; break; // TCCR2A COM2A
            case 3:  tccr_addr = 0xB0; com_shift = 4; break; // TCCR2A COM2B
            default: return false;
        }
    } else {
        // Arduino Mega PWM pins:
        // 2(OC3B),3(OC3C),4(OC0B),5(OC3A),6(OC4A),7(OC4B),8(OC4C),9(OC2B),
        // 10(OC2A),11(OC1A),12(OC1B),13(OC0A),44(OC5C),45(OC5B),46(OC5A)
        switch (arduino_pin) {
            case 4:  tccr_addr = 0x44; com_shift = 4; break;   // TCCR0A COM0B
            case 13: tccr_addr = 0x44; com_shift = 6; break;   // TCCR0A COM0A
            case 10: tccr_addr = 0xB0; com_shift = 6; break;   // TCCR2A COM2A
            case 9:  tccr_addr = 0xB0; com_shift = 4; break;   // TCCR2A COM2B
            case 11: tccr_addr = 0x80; com_shift = 6; break;   // TCCR1A COM1A
            case 12: tccr_addr = 0x80; com_shift = 4; break;   // TCCR1A COM1B
            case 5:  tccr_addr = 0x90; com_shift = 6; break;   // TCCR3A COM3A
            case 2:  tccr_addr = 0x90; com_shift = 4; break;   // TCCR3A COM3B
            case 3:  tccr_addr = 0x90; com_shift = 2; break;   // TCCR3A COM3C
            case 6:  tccr_addr = 0xA0; com_shift = 6; break;   // TCCR4A COM4A
            case 7:  tccr_addr = 0xA0; com_shift = 4; break;   // TCCR4A COM4B
            case 8:  tccr_addr = 0xA0; com_shift = 2; break;   // TCCR4A COM4C
            case 46: tccr_addr = 0x120; com_shift = 6; break;  // TCCR5A COM5A
            case 45: tccr_addr = 0x120; com_shift = 4; break;  // TCCR5A COM5B
            case 44: tccr_addr = 0x120; com_shift = 2; break;  // TCCR5A COM5C
            default: return false;
        }
    }

    const uint8_t tccr = mem.read_io(tccr_addr);
    const uint8_t com_mode = static_cast<uint8_t>((tccr >> com_shift) & 0x03);
    return com_mode != 0;
}

struct PwmChannel {
    uint16_t tccr_addr = 0;
    uint16_t ocr_addr = 0;
    uint8_t com_shift = 0;
    bool valid = false;
};

static PwmChannel resolve_pwm_channel(const MCUVariant& v, uint8_t arduino_pin) {
    const bool is_tiny = (v.name == "ATtiny85");
    const bool is_mega = (v.name == "ATmega2560");

    PwmChannel channel;
    if (is_tiny) {
        channel.tccr_addr = 0x4A;
        if (arduino_pin == 0) {
            channel.ocr_addr = v.OCR0A;
            channel.com_shift = 6;
            channel.valid = true;
        } else if (arduino_pin == 1) {
            channel.ocr_addr = v.OCR0B;
            channel.com_shift = 4;
            channel.valid = true;
        }
        return channel;
    }

    if (!is_mega) {
        switch (arduino_pin) {
            case 6:  channel = {0x44, v.OCR0A, 6, true}; break;
            case 5:  channel = {0x44, v.OCR0B, 4, true}; break;
            case 9:  channel = {0x80, v.OCR1A, 6, true}; break;
            case 10: channel = {0x80, v.OCR1B, 4, true}; break;
            case 11: channel = {0xB0, v.OCR2A, 6, true}; break;
            case 3:  channel = {0xB0, v.OCR2B, 4, true}; break;
            default: break;
        }
        return channel;
    }

    switch (arduino_pin) {
        case 4:  channel = {0x44, 0x48, 4, true}; break;
        case 13: channel = {0x44, 0x47, 6, true}; break;
        case 10: channel = {0xB0, 0xB3, 6, true}; break;
        case 9:  channel = {0xB0, 0xB4, 4, true}; break;
        case 11: channel = {0x80, 0x88, 6, true}; break;
        case 12: channel = {0x80, 0x8A, 4, true}; break;
        case 5:  channel = {0x90, 0x98, 6, true}; break;
        case 2:  channel = {0x90, 0x9A, 4, true}; break;
        case 3:  channel = {0x90, 0x9C, 2, true}; break;
        case 6:  channel = {0xA0, 0xA8, 6, true}; break;
        case 7:  channel = {0xA0, 0xAA, 4, true}; break;
        case 8:  channel = {0xA0, 0xAC, 2, true}; break;
        case 46: channel = {0x120, 0x128, 6, true}; break;
        case 45: channel = {0x120, 0x12A, 4, true}; break;
        case 44: channel = {0x120, 0x12C, 2, true}; break;
        default: break;
    }
    return channel;
}

QEMUInterface::QEMUInterface()
    : m_process_handle(nullptr)
    , m_running(false)
    , m_mode(MCUMode::Simulation)  // Default to simulation mode
    , m_sim_time(0.0)
#ifdef _WIN32
    , m_monitor_socket(INVALID_SOCKET)
    , m_serial_socket(INVALID_SOCKET)
#else
    , m_monitor_socket(-1)
    , m_serial_socket(-1)
#endif
{
#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
}

QEMUInterface::~QEMUInterface() {
    stop();
}

bool QEMUInterface::launch(const std::string& firmware_path, const std::string& machine_type) {
    m_firmware_path = firmware_path;
    m_machine_type = machine_type;

    spdlog::debug("Launching MCU with firmware: {} (mode: {})",
                 firmware_path,
                 m_mode == MCUMode::Simulation ? "Simulation" : "QEMU");

    // Load firmware to verify it exists
    FirmwareLoader loader;
    if (!loader.load(firmware_path)) {
        m_error = "Failed to load firmware: " + loader.error();
        spdlog::error(m_error);
        return false;
    }

    // If in simulation mode, we don't need QEMU
    if (m_mode == MCUMode::Simulation) {
        m_running = true;
        spdlog::debug("MCU simulation initialized (firmware: {} bytes)", loader.size());
        return true;
    }

#ifdef _WIN32
    // Windows QEMU launch
    std::string qemu_cmd = "qemu-system-avr";
    std::string args = " -machine " + machine_type +
                       " -bios " + firmware_path +
                       " -nographic" +
                       " -monitor tcp:" + m_monitor_host + ":" + std::to_string(m_monitor_port) + ",server,nowait" +
                       " -serial tcp:" + m_serial_host + ":" + std::to_string(m_serial_port) + ",server,nowait";

    STARTUPINFOA si;
    PROCESS_INFORMATION pi;
    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);
    ZeroMemory(&pi, sizeof(pi));

    // Create command line
    std::string cmd_line = qemu_cmd + args;

    // Launch QEMU process
    if (!CreateProcessA(
        NULL,
        const_cast<char*>(cmd_line.c_str()),
        NULL,
        NULL,
        FALSE,
        CREATE_NO_WINDOW,
        NULL,
        NULL,
        &si,
        &pi
    )) {
        m_error = "Failed to create QEMU process";
        spdlog::warn("{} - falling back to simulation mode", m_error);
        m_mode = MCUMode::Simulation;
        m_running = true;
        return true;
    }

    m_process_handle = pi.hProcess;
    CloseHandle(pi.hThread);

    // Give QEMU time to start
    Sleep(500);

    // Try to connect to monitor
    m_monitor_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (m_monitor_socket == INVALID_SOCKET) {
        m_error = "Failed to create monitor socket";
        stop();
        return false;
    }

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(m_monitor_host.c_str());
    addr.sin_port = htons(m_monitor_port);

    if (connect(m_monitor_socket, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        spdlog::warn("Could not connect to QEMU monitor socket");
        closesocket(m_monitor_socket);
        m_monitor_socket = INVALID_SOCKET;
    }

#else
    // Unix/Linux QEMU launch
    pid_t pid = fork();

    if (pid < 0) {
        m_error = "Failed to fork process";
        spdlog::warn("{} - falling back to simulation mode", m_error);
        m_mode = MCUMode::Simulation;
        m_running = true;
        return true;
    } else if (pid == 0) {
        // Child process - exec QEMU
#if defined(__linux__)
        prctl(PR_SET_PDEATHSIG, SIGTERM);
#endif

        std::string qemu_exe = "qemu-system-avr";
        std::string monitor_arg = "tcp:" + m_monitor_host + ":" + std::to_string(m_monitor_port) + ",server,nowait";
        std::string serial_arg = "tcp:" + m_serial_host + ":" + std::to_string(m_serial_port) + ",server,nowait";

        execlp(qemu_exe.c_str(), qemu_exe.c_str(),
               "-machine", machine_type.c_str(),
               "-bios", firmware_path.c_str(),
               "-nographic",
               "-monitor", monitor_arg.c_str(),
               "-serial", serial_arg.c_str(),
               NULL);

        // If exec fails
        _exit(1);
    }

    // Parent process
    m_process_handle = new pid_t(pid);

    // Give QEMU time to start
    usleep(500000);

    // Try to connect to monitor socket
    m_monitor_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (m_monitor_socket >= 0) {
        sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr(m_monitor_host.c_str());
        addr.sin_port = htons(m_monitor_port);

        if (connect(m_monitor_socket, (sockaddr*)&addr, sizeof(addr)) < 0) {
            close(m_monitor_socket);
            m_monitor_socket = -1;
        }
    }
#endif

    m_running = true;
    spdlog::info("MCU launched successfully (mode: QEMU)");
    return true;
}

void QEMUInterface::stop() {
    if (!m_running) return;

    spdlog::debug("Stopping MCU");

#ifdef _WIN32
    if (m_monitor_socket != INVALID_SOCKET) {
        closesocket(m_monitor_socket);
        m_monitor_socket = INVALID_SOCKET;
    }
    if (m_serial_socket != INVALID_SOCKET) {
        closesocket(m_serial_socket);
        m_serial_socket = INVALID_SOCKET;
    }

    if (m_process_handle) {
        TerminateProcess((HANDLE)m_process_handle, 0);
        WaitForSingleObject((HANDLE)m_process_handle, 1000);
        CloseHandle((HANDLE)m_process_handle);
        m_process_handle = nullptr;
    }

    WSACleanup();
#else
    if (m_monitor_socket != -1) {
        close(m_monitor_socket);
        m_monitor_socket = -1;
    }
    if (m_serial_socket != -1) {
        close(m_serial_socket);
        m_serial_socket = -1;
    }

    if (m_process_handle) {
        pid_t pid = *(pid_t*)m_process_handle;
        kill(pid, SIGTERM);
        waitpid(pid, nullptr, 0);
        delete (pid_t*)m_process_handle;
        m_process_handle = nullptr;
    }
#endif

    m_running = false;
}

bool QEMUInterface::pin_read(uint8_t port, uint8_t bit, bool& value) {
    if (!m_running) {
        m_error = "MCU not running";
        return false;
    }

    if (m_mode == MCUMode::Simulation) {
        return sim_pin_read(port, bit, value);
    }

    // QEMU mode: read from PIN register
    uint16_t pin_addr = 0;
    switch (port) {
        case 0: pin_addr = m_mcu_variant.PINB; break;
        case 1: pin_addr = m_mcu_variant.PINC; break;
        case 2: pin_addr = m_mcu_variant.PIND; break;
        default: return false;
    }

    uint8_t reg_value;
    if (!qemu_read_register(pin_addr, reg_value)) {
        return false;
    }

    value = (reg_value & (1 << bit)) != 0;
    return true;
}

bool QEMUInterface::pin_write(uint8_t port, uint8_t bit, bool value) {
    if (!m_running) {
        m_error = "MCU not running";
        return false;
    }

    if (m_mode == MCUMode::Simulation) {
        return sim_pin_write(port, bit, value);
    }

    // QEMU mode: write to PORT register
    uint16_t port_addr = 0;
    switch (port) {
        case 0: port_addr = m_mcu_variant.PORTB; break;
        case 1: port_addr = m_mcu_variant.PORTC; break;
        case 2: port_addr = m_mcu_variant.PORTD; break;
        default: return false;
    }

    uint8_t reg_value;
    if (!qemu_read_register(port_addr, reg_value)) {
        return false;
    }

    if (value) {
        reg_value |= (1 << bit);
    } else {
        reg_value &= ~(1 << bit);
    }

    return qemu_write_register(port_addr, reg_value);
}

bool QEMUInterface::analog_read(uint8_t pin, float& voltage) {
    if (!m_running) {
        m_error = "MCU not running";
        return false;
    }

    if (m_mode == MCUMode::Simulation) {
        return sim_analog_read(pin, voltage);
    }

    // QEMU mode: read ADC result
    // For now, return a default value with a warning
    // TODO: Implement proper QEMU ADC register reading
    static bool qemu_adc_warned = false;
    if (!qemu_adc_warned) {
        spdlog::warn("[QEMUInterface] ADC read in QEMU mode not fully implemented - returning default voltage");
        qemu_adc_warned = true;
    }
    voltage = 0.0f;
    return true;  // Return success to avoid breaking simulations, but log warning
}

bool QEMUInterface::analog_write(uint8_t pin, float voltage) {
    if (!m_running) {
        m_error = "MCU not running";
        return false;
    }

    // Clamp voltage to 0-5V range
    voltage = (std::max)(0.0f, (std::min)(5.0f, voltage));

    spdlog::debug("Analog write: pin={} voltage={}", (int)pin, voltage);
    return true;
}

bool QEMUInterface::read_register(uint16_t addr, uint8_t& value) {
    if (!m_running) {
        m_error = "MCU not running";
        return false;
    }

    if (m_mode == MCUMode::Simulation) {
        value = m_memory.read_io(addr);
        return true;
    }

    return qemu_read_register(addr, value);
}

bool QEMUInterface::write_register(uint16_t addr, uint8_t value) {
    if (!m_running) {
        m_error = "MCU not running";
        return false;
    }

    if (m_mode == MCUMode::Simulation) {
        m_memory.write_io(addr, value);
        return true;
    }

    return qemu_write_register(addr, value);
}

bool QEMUInterface::digital_read(uint8_t arduino_pin) {
    if (m_mcu_variant.name == "ATmega2560") {
        const auto r = resolve_mega2560_gpio(arduino_pin);
        if (!r.valid) return false;
        const uint8_t ddr = m_memory.read_io(r.ddr_addr);
        if (ddr & (1 << r.bit)) {
            return (m_memory.read_io(r.port_addr) & (1 << r.bit)) != 0;
        }
        return (m_memory.read_io(r.pin_addr) & (1 << r.bit)) != 0;
    }

    uint8_t port, bit;
    if (m_mcu_variant.name == "ATtiny85") {
        if (arduino_pin > 5) return false;
        port = ArduinoUno::PORT_B;
        bit = arduino_pin;
    } else if (!ArduinoUno::pin_to_port_bit(arduino_pin, port, bit)) {
        return false;
    }

    bool value;
    if (!pin_read(port, bit, value)) {
        return false;
    }

    return value;
}

void QEMUInterface::digital_write(uint8_t arduino_pin, bool value) {
    if (m_mcu_variant.name == "ATmega2560") {
        const auto r = resolve_mega2560_gpio(arduino_pin);
        if (!r.valid) return;
        const uint8_t ddr = m_memory.read_io(r.ddr_addr);
        if ((ddr & (1 << r.bit)) == 0) return;
        uint8_t port_reg = m_memory.read_io(r.port_addr);
        if (value) port_reg |= (1 << r.bit);
        else port_reg &= ~(1 << r.bit);
        m_memory.write_io(r.port_addr, port_reg);
        return;
    }

    uint8_t port, bit;
    if (m_mcu_variant.name == "ATtiny85") {
        if (arduino_pin > 5) return;
        port = ArduinoUno::PORT_B;
        bit = arduino_pin;
    } else if (!ArduinoUno::pin_to_port_bit(arduino_pin, port, bit)) {
        return;
    }

    pin_write(port, bit, value);
}

int QEMUInterface::analog_read(uint8_t analog_pin) {
    float voltage;
    if (!analog_read(analog_pin, voltage)) {
        return 0;
    }

    // Convert 0-5V to 0-1023 (10-bit ADC)
    return static_cast<int>((voltage / 5.0f) * 1023.0f);
}

bool QEMUInterface::set_digital_input(uint8_t arduino_pin, bool value) {
    if (m_mcu_variant.name == "ATmega2560") {
        const auto r = resolve_mega2560_gpio(arduino_pin);
        if (!r.valid) return false;
        uint8_t pin_reg = m_memory.read_io(r.pin_addr);
        if (value) pin_reg |= (1 << r.bit);
        else pin_reg &= ~(1 << r.bit);
        m_memory.write_io(r.pin_addr, pin_reg);
        return true;
    }

    uint8_t port, bit;
    if (m_mcu_variant.name == "ATtiny85") {
        if (arduino_pin > 5) return false;
        port = ArduinoUno::PORT_B;
        bit = arduino_pin;
    } else if (!ArduinoUno::pin_to_port_bit(arduino_pin, port, bit)) {
        return false;
    }

    uint16_t pin_addr = 0;
    switch (port) {
        case ArduinoUno::PORT_B: pin_addr = m_mcu_variant.PINB; break;
        case ArduinoUno::PORT_C: pin_addr = m_mcu_variant.PINC; break;
        case ArduinoUno::PORT_D: pin_addr = m_mcu_variant.PIND; break;
        default: return false;
    }

    uint8_t pin_reg = m_memory.read_io(pin_addr);
    if (value) {
        pin_reg |= (1 << bit);
    } else {
        pin_reg &= ~(1 << bit);
    }
    m_memory.write_io(pin_addr, pin_reg);
    return true;
}

bool QEMUInterface::is_digital_output(uint8_t arduino_pin) const {
    if (m_mcu_variant.name == "ATmega2560") {
        const auto r = resolve_mega2560_gpio(arduino_pin);
        if (!r.valid) return false;
        return (m_memory.read_io(r.ddr_addr) & (1 << r.bit)) != 0;
    }

    uint8_t port, bit;
    if (m_mcu_variant.name == "ATtiny85") {
        if (arduino_pin > 5) return false;
        port = ArduinoUno::PORT_B;
        bit = arduino_pin;
    } else if (!ArduinoUno::pin_to_port_bit(arduino_pin, port, bit)) {
        return false;
    }

    uint16_t ddr_addr = 0;
    switch (port) {
        case ArduinoUno::PORT_B: ddr_addr = m_mcu_variant.DDRB; break;
        case ArduinoUno::PORT_C: ddr_addr = m_mcu_variant.DDRC; break;
        case ArduinoUno::PORT_D: ddr_addr = m_mcu_variant.DDRD; break;
        default: return false;
    }

    return (m_memory.read_io(ddr_addr) & (1 << bit)) != 0;
}

float QEMUInterface::digital_output_voltage(uint8_t arduino_pin) const {
    if (m_mcu_variant.name == "ATmega2560") {
        const auto r = resolve_mega2560_gpio(arduino_pin);
        if (!r.valid) return 0.0f;
        const bool is_output = (m_memory.read_io(r.ddr_addr) & (1 << r.bit)) != 0;
        if (!is_output) return 0.0f;
        auto pwm_voltage = [&](uint16_t tccrA_addr, uint16_t ocr_addr, uint8_t com_shift, bool ocr_is_16bit) -> float {
            const uint8_t tccrA = m_memory.read_io(tccrA_addr);
            const uint8_t com = (tccrA >> com_shift) & 0x3;
            if (com == 0) return -1.0f; // not enabled

            const uint8_t duty8 = ocr_is_16bit ? m_memory.read_io(ocr_addr) : m_memory.read_io(ocr_addr);
            const float duty = static_cast<float>(duty8) / 255.0f;
            return duty * 5.0f;
        };

        // Arduino Mega PWM pins:
        // 2(OC3B),3(OC3C),4(OC0B),5(OC3A),6(OC4A),7(OC4B),8(OC4C),9(OC2B),
        // 10(OC2A),11(OC1A),12(OC1B),13(OC0A),44(OC5C),45(OC5B),46(OC5A)
        float v = -1.0f;
        switch (arduino_pin) {
            case 4:  v = pwm_voltage(0x44, 0x48, 4, false); break; // TCCR0A, OCR0B, COM0B
            case 13: v = pwm_voltage(0x44, 0x47, 6, false); break; // TCCR0A, OCR0A, COM0A
            case 10: v = pwm_voltage(0xB0, 0xB3, 6, false); break; // TCCR2A, OCR2A, COM2A
            case 9:  v = pwm_voltage(0xB0, 0xB4, 4, false); break; // TCCR2A, OCR2B, COM2B
            case 11: v = pwm_voltage(0x80, 0x88, 6, true);  break; // TCCR1A, OCR1A, COM1A
            case 12: v = pwm_voltage(0x80, 0x8A, 4, true);  break; // TCCR1A, OCR1B, COM1B
            case 5:  v = pwm_voltage(0x90, 0x98, 6, true);  break; // TCCR3A, OCR3A, COM3A
            case 2:  v = pwm_voltage(0x90, 0x9A, 4, true);  break; // TCCR3A, OCR3B, COM3B
            case 3:  v = pwm_voltage(0x90, 0x9C, 2, true);  break; // TCCR3A, OCR3C, COM3C
            case 6:  v = pwm_voltage(0xA0, 0xA8, 6, true);  break; // TCCR4A, OCR4A, COM4A
            case 7:  v = pwm_voltage(0xA0, 0xAA, 4, true);  break; // TCCR4A, OCR4B, COM4B
            case 8:  v = pwm_voltage(0xA0, 0xAC, 2, true);  break; // TCCR4A, OCR4C, COM4C
            case 46: v = pwm_voltage(0x120, 0x128, 6, true); break; // TCCR5A, OCR5A, COM5A
            case 45: v = pwm_voltage(0x120, 0x12A, 4, true); break; // TCCR5A, OCR5B, COM5B
            case 44: v = pwm_voltage(0x120, 0x12C, 2, true); break; // TCCR5A, OCR5C, COM5C
            default: break;
        }
        if (v >= 0.0f) return v;

        const bool high = (m_memory.read_io(r.port_addr) & (1 << r.bit)) != 0;
        return high ? 5.0f : 0.0f;
    }

    uint8_t port, bit;
    const bool is_tiny = (m_mcu_variant.name == "ATtiny85");
    if (is_tiny) {
        if (arduino_pin > 5) return 0.0f;
        port = ArduinoUno::PORT_B;
        bit = arduino_pin;
    } else if (!ArduinoUno::pin_to_port_bit(arduino_pin, port, bit)) {
        return 0.0f;
    }

    uint16_t port_addr = 0;
    switch (port) {
        case ArduinoUno::PORT_B: port_addr = m_mcu_variant.PORTB; break;
        case ArduinoUno::PORT_C: port_addr = m_mcu_variant.PORTC; break;
        case ArduinoUno::PORT_D: port_addr = m_mcu_variant.PORTD; break;
        default: return 0.0f;
    }

    // ATmega328P Timer Control Register addresses and COM bit positions
    // Timer0: TCCR0A=0x44, OCR0A=0x47, OCR0B=0x48
    //   Pin 6 (OC0A): COM0A[1:0]=bits[7:6] in TCCR0A
    //   Pin 5 (OC0B): COM0B[1:0]=bits[5:4] in TCCR0A
    // Timer1: TCCR1A=0x80, OCR1AL=0x88, OCR1BL=0x8A
    //   Pin 9  (OC1A): COM1A[1:0]=bits[7:6] in TCCR1A
    //   Pin 10 (OC1B): COM1B[1:0]=bits[5:4] in TCCR1A
    // Timer2: TCCR2A=0xB0, OCR2A=0xB3, OCR2B=0xB4
    //   Pin 11 (OC2A): COM2A[1:0]=bits[7:6] in TCCR2A
    //   Pin 3  (OC2B): COM2B[1:0]=bits[5:4] in TCCR2A
    const bool pwm_pin = is_tiny ? (arduino_pin == 0 || arduino_pin == 1) : ArduinoUno::is_pwm_pin(arduino_pin);
    if (pwm_pin) {
        uint16_t tccr_addr = 0;
        uint16_t ocr_addr = 0;
        uint8_t com_shift = 0;

        if (is_tiny) {
            // ATtiny85 Timer0: TCCR0A=0x4A (data space), OCR0A=0x49, OCR0B=0x48
            tccr_addr = 0x4A;
            if (arduino_pin == 0) { ocr_addr = m_mcu_variant.OCR0A; com_shift = 6; } // COM0A
            if (arduino_pin == 1) { ocr_addr = m_mcu_variant.OCR0B; com_shift = 4; } // COM0B
        } else {
            switch (arduino_pin) {
                case 6:  tccr_addr = 0x44; ocr_addr = m_mcu_variant.OCR0A; com_shift = 6; break;
                case 5:  tccr_addr = 0x44; ocr_addr = m_mcu_variant.OCR0B; com_shift = 4; break;
                case 9:  tccr_addr = 0x80; ocr_addr = m_mcu_variant.OCR1A; com_shift = 6; break;
                case 10: tccr_addr = 0x80; ocr_addr = m_mcu_variant.OCR1B; com_shift = 4; break;
                case 11: tccr_addr = 0xB0; ocr_addr = m_mcu_variant.OCR2A; com_shift = 6; break;
                case 3:  tccr_addr = 0xB0; ocr_addr = m_mcu_variant.OCR2B; com_shift = 4; break;
                default: break;
            }
        }

        if (ocr_addr != 0 && tccr_addr != 0) {
            uint8_t tccr = m_memory.read_io(tccr_addr);
            uint8_t com_mode = static_cast<uint8_t>((tccr >> com_shift) & 0x03);
            if (com_mode != 0) {
                uint8_t duty = m_memory.read_io(ocr_addr);
                return (static_cast<float>(duty) / 255.0f) * 5.0f;
            }
        }
    }

    return (m_memory.read_io(port_addr) & (1 << bit)) ? 5.0f : 0.0f;
}

bool QEMUInterface::is_pwm_enabled(uint8_t arduino_pin) const {
    // Only meaningful for output pins.
    if (!is_digital_output(arduino_pin)) return false;
    return pwm_enabled_for_pin(m_mcu_variant, m_memory, arduino_pin);
}

bool QEMUInterface::pwm_duty_fraction(uint8_t arduino_pin, float& duty) const {
    duty = 0.0f;
    if (!is_pwm_enabled(arduino_pin)) {
        return false;
    }

    const auto channel = resolve_pwm_channel(m_mcu_variant, arduino_pin);
    if (!channel.valid || channel.ocr_addr == 0 || channel.tccr_addr == 0) {
        return false;
    }

    const uint8_t tccr = m_memory.read_io(channel.tccr_addr);
    const uint8_t com_mode = static_cast<uint8_t>((tccr >> channel.com_shift) & 0x03);
    if (com_mode == 0) {
        return false;
    }

    const uint8_t duty8 = m_memory.read_io(channel.ocr_addr);
    duty = std::clamp(static_cast<float>(duty8) / 255.0f, 0.0f, 1.0f);
    return true;
}

double QEMUInterface::pwm_frequency_hz(uint8_t arduino_pin) const {
    if (m_mcu_variant.name == "ATmega328P") {
        if (arduino_pin == 5 || arduino_pin == 6) {
            return 976.5625;
        }
        return 490.196078;
    }

    if (m_mcu_variant.name == "ATmega2560") {
        if (arduino_pin == 4 || arduino_pin == 13) {
            return 976.5625;
        }
        return 490.196078;
    }

    if (m_mcu_variant.name == "ATtiny85") {
        return 976.5625;
    }

    return 490.196078;
}

bool QEMUInterface::pwm_output_voltage(uint8_t arduino_pin, double time_s, float high_voltage, float low_voltage, float& voltage) const {
    float duty = 0.0f;
    if (!pwm_duty_fraction(arduino_pin, duty)) {
        return false;
    }

    if (duty <= 0.0f) {
        voltage = low_voltage;
        return true;
    }
    if (duty >= 1.0f) {
        voltage = high_voltage;
        return true;
    }

    const double frequency = pwm_frequency_hz(arduino_pin);
    const double phase = std::fmod(std::max(0.0, time_s) * frequency, 1.0);
    voltage = phase < static_cast<double>(duty) ? high_voltage : low_voltage;
    return true;
}

bool QEMUInterface::set_analog_input(uint8_t analog_pin, float voltage) {
    if (analog_pin >= analog_pin_count_for_variant(m_mcu_variant)) {
        return false;
    }

    voltage = (std::max)(0.0f, (std::min)(5.0f, voltage));
    m_analog_inputs[analog_pin] = voltage;
    return true;
}

bool QEMUInterface::step_instructions(uint32_t count) {
    if (!m_running) {
        m_error = "MCU not running";
        return false;
    }

    if (m_mode == MCUMode::Simulation) {
        // In simulation mode, just update time
        // Each instruction takes ~62.5ns at 16MHz
        m_sim_time += (count * 62.5e-9);
        return true;
    }

    // Send step command to QEMU monitor
    std::string cmd = "stepi " + std::to_string(count);
    return send_monitor_command(cmd);
}

void QEMUInterface::update_simulation(double dt) {
    if (!m_running || m_mode != MCUMode::Simulation) {
        return;
    }

    m_sim_time += dt;

    // Simulate ADC conversions
    if (m_mcu_variant.ADCSRA == 0 || m_mcu_variant.ADMUX == 0) {
        return;
    }
    uint8_t adcsra = m_memory.read_io(m_mcu_variant.ADCSRA);
    if (adcsra & (1 << 6)) {  // ADSC (ADC Start Conversion) bit
        // ADC conversion takes ~13 ADC cycles = ~104µs at 16MHz prescaled by 128
        // For simplicity, complete conversion immediately
        uint8_t completed_adcsra = static_cast<uint8_t>((adcsra & ~(1 << 6)) | (1 << 4));

        uint8_t admux = m_memory.read_io(m_mcu_variant.ADMUX);
        uint8_t channel = admux & 0x0F;  // MUX bits

        if (channel < analog_pin_count_for_variant(m_mcu_variant)) {
            float voltage = m_analog_inputs[channel];
            int adc_value = static_cast<int>((voltage / 5.0f) * 1023.0f);

            // Write to ADCL/ADCH
            if (admux & (1 << 5)) {  // ADLAR bit
                m_memory.write_io(m_mcu_variant.ADCL, (adc_value << 6) & 0xC0);
                m_memory.write_io(m_mcu_variant.ADCH, (adc_value >> 2) & 0xFF);
            } else {
                m_memory.write_io(m_mcu_variant.ADCL, adc_value & 0xFF);
                m_memory.write_io(m_mcu_variant.ADCH, (adc_value >> 8) & 0x03);
            }
        }
        m_memory.write_io(m_mcu_variant.ADCSRA, completed_adcsra);
    }
}

// Simulation mode functions
bool QEMUInterface::sim_pin_read(uint8_t port, uint8_t bit, bool& value) {
    uint16_t pin_addr = 0;
    uint16_t port_addr = 0;
    uint16_t ddr_addr = 0;

    switch (port) {
        case 0:  // PORTB
            pin_addr = m_mcu_variant.PINB;
            port_addr = m_mcu_variant.PORTB;
            ddr_addr = m_mcu_variant.DDRB;
            break;
        case 1:  // PORTC
            if (m_mcu_variant.PORTC == 0) return false;
            pin_addr = m_mcu_variant.PINC;
            port_addr = m_mcu_variant.PORTC;
            ddr_addr = m_mcu_variant.DDRC;
            break;
        case 2:  // PORTD
            if (m_mcu_variant.PORTD == 0) return false;
            pin_addr = m_mcu_variant.PIND;
            port_addr = m_mcu_variant.PORTD;
            ddr_addr = m_mcu_variant.DDRD;
            break;
        default:
            return false;
    }

    uint8_t ddr = m_memory.read_io(ddr_addr);
    uint8_t port_reg = m_memory.read_io(port_addr);

    // If pin is output, read PORT bit
    // If pin is input, read from external (for now, read PIN bit)
    if (ddr & (1 << bit)) {
        // Output pin - read from PORT
        value = (port_reg & (1 << bit)) != 0;
    } else {
        // Input pin - read from PIN (external value)
        value = (m_memory.read_io(pin_addr) & (1 << bit)) != 0;
    }

    return true;
}

bool QEMUInterface::sim_pin_write(uint8_t port, uint8_t bit, bool value) {
    uint16_t port_addr = 0;
    uint16_t ddr_addr = 0;

    switch (port) {
        case 0:  // PORTB
            port_addr = m_mcu_variant.PORTB;
            ddr_addr = m_mcu_variant.DDRB;
            break;
        case 1:  // PORTC
            if (m_mcu_variant.PORTC == 0) return false;
            port_addr = m_mcu_variant.PORTC;
            ddr_addr = m_mcu_variant.DDRC;
            break;
        case 2:  // PORTD
            if (m_mcu_variant.PORTD == 0) return false;
            port_addr = m_mcu_variant.PORTD;
            ddr_addr = m_mcu_variant.DDRD;
            break;
        default:
            return false;
    }

    uint8_t ddr = m_memory.read_io(ddr_addr);
    uint8_t port_reg = m_memory.read_io(port_addr);

    // Only write if pin is configured as output
    if (ddr & (1 << bit)) {
        if (value) {
            port_reg |= (1 << bit);
        } else {
            port_reg &= ~(1 << bit);
        }
        m_memory.write_io(port_addr, port_reg);

        spdlog::trace("Sim: pin write port={} bit={} value={}", (int)port, (int)bit, value);
    }

    return true;
}

bool QEMUInterface::sim_analog_read(uint8_t pin, float& voltage) {
    if (pin >= analog_pin_count_for_variant(m_mcu_variant)) {
        return false;
    }

    voltage = m_analog_inputs[pin];
    return true;
}

// QEMU mode functions
bool QEMUInterface::qemu_read_register(uint16_t addr, uint8_t& value) {
    if (m_monitor_socket < 0) {
        // Monitor not available - use simulated value
        value = m_memory.read_io(addr);
        spdlog::trace("QEMU read fallback (no socket): addr=0x{:04X} value=0x{:02X}", addr, value);
        return true;
    }

    char buf[32];
    snprintf(buf, sizeof(buf), "x /1b 0x%04X", addr);
    std::string cmd = buf;
    if (!send_monitor_command(cmd)) {
        // Command failed - fall back to simulation
        value = m_memory.read_io(addr);
        spdlog::warn("QEMU read command failed, using simulation: addr=0x{:04X}", addr);
        return false;  // Return false to indicate fallback occurred
    }

    std::string response;
    if (!read_response(response)) {
        // Read failed - fall back to simulation
        value = m_memory.read_io(addr);
        spdlog::warn("QEMU read response failed, using simulation: addr=0x{:04X}", addr);
        return false;  // Return false to indicate fallback occurred
    }

    // Parse response (format: "0xXXXX: 0xYY")
    // QEMU monitor response format varies, try to extract the value
    // Common formats:
    // - "0x0025: 0x00"
    // - "0025: 00"
    // - "0x00 00 00 ..."

    // Try to find the hex value after the colon
    size_t colon_pos = response.find(':');
    if (colon_pos != std::string::npos) {
        // Look for "0x" followed by hex digits
        size_t value_start = response.find("0x", colon_pos);
        if (value_start != std::string::npos) {
            value_start += 2;  // Skip "0x"
            size_t value_end = response.find_first_not_of("0123456789abcdefABCDEF", value_start);
            if (value_end != std::string::npos) {
                std::string hex_str = response.substr(value_start, value_end - value_start);
                try {
                    value = static_cast<uint8_t>(std::stoi(hex_str, nullptr, 16));
                    spdlog::trace("QEMU read: addr=0x{:04X} value=0x{:02X}", addr, value);
                    return true;
                } catch (...) {
                    // Parse failed, fall back to simulation
                }
            }
        }
    }

    // Parse failed - fall back to simulation
    value = m_memory.read_io(addr);
    spdlog::warn("QEMU read parse failed, using simulation: addr=0x{:04X} response='{}'", addr, response);
    return false;  // Return false to indicate fallback occurred
}

bool QEMUInterface::qemu_write_register(uint16_t addr, uint8_t value) {
    if (m_monitor_socket < 0) {
        // Monitor not available - store locally
        m_memory.write_io(addr, value);
        return true;
    }

    char buf[32];
    snprintf(buf, sizeof(buf), "writel 0x%04X 0x%02X", addr, value);
    std::string cmd = buf;
    return send_monitor_command(cmd);
}

bool QEMUInterface::send_monitor_command(const std::string& cmd) {
#ifdef _WIN32
    if (m_monitor_socket == INVALID_SOCKET) {
#else
    if (m_monitor_socket < 0) {
#endif
        // Monitor socket not available
        return false;
    }

    std::string full_cmd = cmd + "\n";
#ifdef _WIN32
    if (send(m_monitor_socket, full_cmd.c_str(), full_cmd.length(), 0) < 0) {
#else
    if (send(m_monitor_socket, full_cmd.c_str(), full_cmd.length(), 0) < 0) {
#endif
        m_error = "Failed to send monitor command";
        return false;
    }

    return true;
}

bool QEMUInterface::read_response(std::string& response) {
#ifdef _WIN32
    if (m_monitor_socket == INVALID_SOCKET) {
#else
    if (m_monitor_socket < 0) {
#endif
        response = "";
        return false;
    }

    char buffer[1024];
#ifdef _WIN32
    int len = recv(m_monitor_socket, buffer, sizeof(buffer) - 1, 0);
#else
    ssize_t len = recv(m_monitor_socket, buffer, sizeof(buffer) - 1, 0);
#endif

    if (len > 0) {
        buffer[len] = '\0';
        response = buffer;
        return true;
    }

    response = "";
    return false;
}

// Arduino Uno pin mapping
bool ArduinoUno::pin_to_port_bit(uint8_t pin, uint8_t& port, uint8_t& bit) {
    if (pin <= 7) {
        port = PORT_D;
        bit = pin;
    } else if (pin <= 13) {
        port = PORT_B;
        bit = pin - 8;
    } else if (pin >= 14 && pin <= 19) {
        // Analog pins A0-A5 mapped as digital
        port = PORT_C;
        bit = pin - 14;
    } else {
        return false;
    }
    return true;
}

uint8_t ArduinoUno::pin_to_adc_channel(uint8_t analog_pin) {
    // A0-A5 map directly to ADC channels 0-5
    if (analog_pin < NUM_ANALOG_PINS) {
        return analog_pin;
    }
    return 0xFF;  // Invalid
}

bool ArduinoUno::is_pwm_pin(uint8_t pin) {
    // PWM pins on Arduino Uno: 3, 5, 6, 9, 10, 11
    return (pin == 3 || pin == 5 || pin == 6 || pin == 9 || pin == 10 || pin == 11);
}

uint8_t ArduinoUno::get_pwm_timer(uint8_t pin) {
    switch (pin) {
        case 9:
        case 10:
            return 1;  // Timer1
        case 11:
        case 3:
            return 2;  // Timer2
        case 5:
        case 6:
            return 0;  // Timer0
        default:
            return 0xFF;  // Not a PWM pin
    }
}

} // namespace mechatron
