#include "PhysicalMCUBridge.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <chrono>
#include <thread>

namespace mechatron {

PhysicalMCUBridge::PhysicalMCUBridge() {
    m_last_pwm.fill(0);
}

PhysicalMCUBridge::~PhysicalMCUBridge() {
    disconnect();
}

bool PhysicalMCUBridge::connect(const std::string& port, int baud_rate) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_port = port;
    m_baud_rate = baud_rate;
    m_board.clear();
    m_capabilities = 0;
    m_rx_buffer.clear();
    m_latest_inputs.reset();
    m_last_output_mask = 0xFFFFFFFFu;
    m_last_digital_bits = 0xFFFFFFFFu;
    m_last_pwm_mask = 0xFFFFFFFFu;
    m_last_pwm.fill(0);
    m_pwm_dirty = false;
    m_digital_dirty = false;
    m_last_request_sent = {};

    if (!m_serial.open(port, baud_rate)) {
        return false;
    }

    if (!handshake_locked()) {
        m_serial.close();
        return false;
    }

    // Default: request all inputs
    m_req.digital_mask = (1u << 20) - 1u;
    m_req.analog_mask = 0x3F;
    physical_mcu::Frame req;
    req.type = physical_mcu::MsgType::RequestInputs;
    req.payload = physical_mcu::encode_request_inputs(m_req);
    send_frame(req);
    m_last_request_sent = std::chrono::steady_clock::now();
    return true;
}

void PhysicalMCUBridge::disconnect() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_serial.close();
    m_board.clear();
    m_capabilities = 0;
    m_rx_buffer.clear();
    m_latest_inputs.reset();
}

bool PhysicalMCUBridge::is_connected() const {
    return m_serial.is_open();
}

void PhysicalMCUBridge::send_frame(const physical_mcu::Frame& f) {
    const auto bytes = physical_mcu::encode_frame(f);
    (void)m_serial.write(bytes.data(), bytes.size());
}

bool PhysicalMCUBridge::handshake_locked() {
    physical_mcu::Frame hello;
    hello.type = physical_mcu::MsgType::Hello;
    hello.flags = 0;
    hello.payload = physical_mcu::encode_hello({ "mechatron_host", 0 });
    send_frame(hello);

    // Wait briefly for HelloAck
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(800)) {
        pump_rx_locked();
        if (!m_board.empty()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    spdlog::warn("[PhysicalMCUBridge] Handshake timeout (no HelloAck)");
    return false;
}

void PhysicalMCUBridge::pump_rx_locked() {
    uint8_t buf[256];
    for (;;) {
        const ssize_t n = m_serial.read(buf, sizeof(buf));
        if (n <= 0) break;
        m_rx_buffer.insert(m_rx_buffer.end(), buf, buf + n);
    }

    for (;;) {
        auto frame = physical_mcu::try_decode_one(m_rx_buffer);
        if (!frame.has_value()) break;
        handle_frame_locked(*frame);
    }
}

void PhysicalMCUBridge::handle_frame_locked(const physical_mcu::Frame& f) {
    using physical_mcu::MsgType;
    switch (f.type) {
        case MsgType::HelloAck: {
            auto p = physical_mcu::decode_hello(f.payload);
            if (p) {
                m_board = p->board;
                m_capabilities = p->capabilities;
                spdlog::info("[PhysicalMCUBridge] Paired device: {} (caps=0x{:08X})", m_board, m_capabilities);
            }
            break;
        }
        case MsgType::InputsReport: {
            auto p = physical_mcu::decode_inputs_report(f.payload);
            if (p) m_latest_inputs = *p;
            break;
        }
        case MsgType::LogLine: {
            std::string s(reinterpret_cast<const char*>(f.payload.data()), f.payload.size());
            spdlog::info("[PhysicalMCU] {}", s);
            break;
        }
        case MsgType::Error: {
            std::string s(reinterpret_cast<const char*>(f.payload.data()), f.payload.size());
            spdlog::warn("[PhysicalMCU][ERR] {}", s);
            break;
        }
        default:
            break;
    }
}

void PhysicalMCUBridge::set_digital_outputs(uint32_t output_mask, uint32_t digital_bits) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_serial.is_open()) return;
    output_mask &= ((1u << 20) - 1u);
    digital_bits &= output_mask;
    if (output_mask == m_last_output_mask && digital_bits == m_last_digital_bits) return;
    m_last_output_mask = output_mask;
    m_last_digital_bits = digital_bits;
    m_digital_dirty = true;
}

void PhysicalMCUBridge::set_pwm_duty(uint8_t pin, uint8_t duty_0_255) {
    if (pin >= m_last_pwm.size()) return;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_serial.is_open()) return;
    if (m_last_pwm[pin] == duty_0_255) return;
    m_last_pwm[pin] = duty_0_255;
    m_last_pwm_mask |= (1u << pin);
    m_pwm_dirty = true;
}

void PhysicalMCUBridge::set_pwm_bulk(uint32_t pwm_mask, const std::array<uint8_t, 20>& duty) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_serial.is_open()) return;
    pwm_mask &= ((1u << 20) - 1u);
    if (pwm_mask == m_last_pwm_mask && duty == m_last_pwm) return;
    m_last_pwm_mask = pwm_mask;
    m_last_pwm = duty;
    m_pwm_dirty = true;
}

void PhysicalMCUBridge::request_inputs(uint32_t digital_mask, uint8_t analog_mask) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_serial.is_open()) return;
    m_req.digital_mask = digital_mask & ((1u << 20) - 1u);
    m_req.analog_mask = analog_mask & 0x3F;

    physical_mcu::Frame f;
    f.type = physical_mcu::MsgType::RequestInputs;
    f.payload = physical_mcu::encode_request_inputs(m_req);
    send_frame(f);
    m_last_request_sent = std::chrono::steady_clock::now();
}

void PhysicalMCUBridge::poll() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_serial.is_open()) return;

    // Flush pending output updates first (bulk messages)
    if (m_digital_dirty) {
        physical_mcu::Frame f;
        f.type = physical_mcu::MsgType::SetDigitalMask;
        f.payload = physical_mcu::encode_digital_outputs({m_last_output_mask, m_last_digital_bits});
        send_frame(f);
        m_digital_dirty = false;
    }
    if (m_pwm_dirty) {
        physical_mcu::Frame f;
        f.type = physical_mcu::MsgType::SetPwmBulk;
        physical_mcu::PwmOutputsPayload payload;
        payload.pwm_mask = m_last_pwm_mask;
        std::copy(m_last_pwm.begin(), m_last_pwm.end(), std::begin(payload.duty));
        f.payload = physical_mcu::encode_pwm_outputs(payload);
        send_frame(f);
        m_pwm_dirty = false;
    }

    const auto now = std::chrono::steady_clock::now();
    if (m_last_request_sent.time_since_epoch().count() == 0 ||
        now - m_last_request_sent >= std::chrono::milliseconds(100)) {
        physical_mcu::Frame req;
        req.type = physical_mcu::MsgType::RequestInputs;
        req.payload = physical_mcu::encode_request_inputs(m_req);
        send_frame(req);
        m_last_request_sent = now;
    }

    pump_rx_locked();
}

std::optional<physical_mcu::InputsReportPayload> PhysicalMCUBridge::latest_inputs() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_latest_inputs;
}

} // namespace mechatron
