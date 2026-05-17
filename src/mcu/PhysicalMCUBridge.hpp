#pragma once

#include "io/SerialPort.hpp"
#include "mcu/PhysicalMCUProtocol.hpp"
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace mechatron {

// Two-way bridge between simulated MCU and a physical ATmega running "mechatron_bridge".
// The physical device executes simple IO instructions (pinMode/digitalWrite/PWM) received
// over serial, and periodically reports input states (digital + analog).
class PhysicalMCUBridge {
public:
    PhysicalMCUBridge();
    ~PhysicalMCUBridge();

    PhysicalMCUBridge(const PhysicalMCUBridge&) = delete;
    PhysicalMCUBridge& operator=(const PhysicalMCUBridge&) = delete;

    bool connect(const std::string& port, int baud_rate);
    void disconnect();
    bool is_connected() const;

    std::string port() const { return m_port; }
    int baud_rate() const { return m_baud_rate; }
    std::string board() const { return m_board; }

    // Host -> device (outputs)
    void set_digital_outputs(uint32_t output_mask, uint32_t digital_bits); // D0..D13 + A0..A5 as 14..19
    void set_pwm_duty(uint8_t pin, uint8_t duty_0_255);        // Arduino pin numbering
    void set_pwm_bulk(uint32_t pwm_mask, const std::array<uint8_t, 20>& duty); // index by Arduino pin (0..19)

    // Host <-> device (inputs)
    void request_inputs(uint32_t digital_mask, uint8_t analog_mask);
    void poll(); // pump RX and update latest inputs

    std::optional<physical_mcu::InputsReportPayload> latest_inputs() const;

private:
    void send_frame(const physical_mcu::Frame& f);
    void pump_rx_locked();
    void handle_frame_locked(const physical_mcu::Frame& f);

    bool handshake_locked();

    mutable std::mutex m_mutex;
    SerialPort m_serial;
    std::string m_port;
    int m_baud_rate = 115200;

    std::string m_board;
    uint32_t m_capabilities = 0;

    std::vector<uint8_t> m_rx_buffer;
    physical_mcu::RequestInputsPayload m_req{};
    std::optional<physical_mcu::InputsReportPayload> m_latest_inputs;

    // Output caching to avoid flooding serial
    uint32_t m_last_output_mask = 0xFFFFFFFFu;
    uint32_t m_last_digital_bits = 0xFFFFFFFFu;
    uint32_t m_last_pwm_mask = 0xFFFFFFFFu;
    std::array<uint8_t, 20> m_last_pwm{};
    bool m_pwm_dirty = false;
    bool m_digital_dirty = false;
    std::chrono::steady_clock::time_point m_last_request_sent{};
};

} // namespace mechatron
