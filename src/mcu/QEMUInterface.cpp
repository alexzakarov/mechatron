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
#include <sys/prctl.h>
#endif

namespace mechatron {

// Constants
constexpr uint8_t NUM_ANALOG_PINS = 6;

// ATmega328P clock frequency
constexpr uint32_t F_CPU = 16000000;  // 16 MHz

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

    spdlog::info("Launching MCU with firmware: {} (mode: {})",
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
        spdlog::info("MCU simulation initialized (firmware: {} bytes)", loader.size());
        return true;
    }

#ifdef _WIN32
    // Windows QEMU launch
    std::string qemu_cmd = "qemu-system-avr";
    std::string args = " -machine " + machine_type +
                       " -bios " + firmware_path +
                       " -nographic" +
                       " -monitor tcp:127.0.0.1:4444,server,nowait" +
                       " -serial tcp:127.0.0.1:4445,server,nowait";

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
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    addr.sin_port = htons(4444);

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
        prctl(PR_SET_PDEATHSIG, SIGTERM);

        std::string qemu_exe = "qemu-system-avr";
        execlp(qemu_exe.c_str(), qemu_exe.c_str(),
               "-machine", machine_type.c_str(),
               "-bios", firmware_path.c_str(),
               "-nographic",
               "-monitor", "tcp:127.0.0.1:4444,server,nowait",
               "-serial", "tcp:127.0.0.1:4445,server,nowait",
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
        addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        addr.sin_port = htons(4444);

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

    spdlog::info("Stopping MCU");

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
        case 0: pin_addr = ATmega328P_Registers::PINB; break;
        case 1: pin_addr = ATmega328P_Registers::PINC; break;
        case 2: pin_addr = ATmega328P_Registers::PIND; break;
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
        case 0: port_addr = ATmega328P_Registers::PORTB; break;
        case 1: port_addr = ATmega328P_Registers::PORTC; break;
        case 2: port_addr = ATmega328P_Registers::PORTD; break;
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
    // For now, return a default value
    voltage = 0.0f;
    return true;
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
    uint8_t port, bit;
    if (!ArduinoUno::pin_to_port_bit(arduino_pin, port, bit)) {
        return false;
    }

    bool value;
    if (!pin_read(port, bit, value)) {
        return false;
    }

    return value;
}

void QEMUInterface::digital_write(uint8_t arduino_pin, bool value) {
    uint8_t port, bit;
    if (!ArduinoUno::pin_to_port_bit(arduino_pin, port, bit)) {
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
    uint8_t adcsra = m_memory.read_io(ATmega328P_Registers::ADCSRA);
    if (adcsra & (1 << 6)) {  // ADSC (ADC Start Conversion) bit
        // ADC conversion takes ~13 ADC cycles = ~104µs at 16MHz prescaled by 128
        // For simplicity, complete conversion immediately
        m_memory.write_io(ATmega328P_Registers::ADCSRA, adcsra & ~(1 << 6));

        uint8_t admux = m_memory.read_io(ATmega328P_Registers::ADMUX);
        uint8_t channel = admux & 0x0F;  // MUX bits

        if (channel < NUM_ANALOG_PINS) {
            float voltage = m_analog_inputs[channel];
            int adc_value = static_cast<int>((voltage / 5.0f) * 1023.0f);

            // Write to ADCL/ADCH
            if (admux & (1 << 5)) {  // ADLAR bit
                m_memory.write_io(ATmega328P_Registers::ADCL, adc_value & 0xFF);
                m_memory.write_io(ATmega328P_Registers::ADCH, (adc_value >> 8) & 0x03);
            } else {
                m_memory.write_io(ATmega328P_Registers::ADCL, adc_value & 0xFF);
                m_memory.write_io(ATmega328P_Registers::ADCH, (adc_value >> 8) & 0x03);
            }

            // Set ADC interrupt flag
            m_memory.write_io(ATmega328P_Registers::ADCSRA, adcsra | (1 << 4));
        }
    }
}

// Simulation mode functions
bool QEMUInterface::sim_pin_read(uint8_t port, uint8_t bit, bool& value) {
    uint16_t pin_addr = 0;
    uint16_t port_addr = 0;
    uint16_t ddr_addr = 0;

    switch (port) {
        case 0:  // PORTB
            pin_addr = ATmega328P_Registers::PINB;
            port_addr = ATmega328P_Registers::PORTB;
            ddr_addr = ATmega328P_Registers::DDRB;
            break;
        case 1:  // PORTC
            pin_addr = ATmega328P_Registers::PINC;
            port_addr = ATmega328P_Registers::PORTC;
            ddr_addr = ATmega328P_Registers::DDRC;
            break;
        case 2:  // PORTD
            pin_addr = ATmega328P_Registers::PIND;
            port_addr = ATmega328P_Registers::PORTD;
            ddr_addr = ATmega328P_Registers::DDRD;
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
            port_addr = ATmega328P_Registers::PORTB;
            ddr_addr = ATmega328P_Registers::DDRB;
            break;
        case 1:  // PORTC
            port_addr = ATmega328P_Registers::PORTC;
            ddr_addr = ATmega328P_Registers::DDRC;
            break;
        case 2:  // PORTD
            port_addr = ATmega328P_Registers::PORTD;
            ddr_addr = ATmega328P_Registers::DDRD;
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
    if (pin >= NUM_ANALOG_PINS) {
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
        return true;
    }

    char buf[32];
    snprintf(buf, sizeof(buf), "x /1b 0x%04X", addr);
    std::string cmd = buf;
    if (!send_monitor_command(cmd)) {
        return false;
    }

    std::string response;
    if (!read_response(response)) {
        return false;
    }

    // Parse response (format: "0xXXXX: 0xYY")
    // For simplicity, return simulated value
    value = m_memory.read_io(addr);
    return true;
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
