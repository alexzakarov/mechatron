#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <optional>

namespace mechatron::physical_mcu {

// Binary framing:
//   magic[4] = "MCHT"
//   version  = 1
//   type     = MsgType
//   flags    = reserved
//   len_le16 = payload bytes
//   crc_le16 = CRC16-CCITT over header (version..len) + payload
//   payload  = len bytes

constexpr uint8_t kVersion = 1;

enum class MsgType : uint8_t {
    Hello = 1,
    HelloAck = 2,

    // Host -> device
    SetPinMode = 10,
    DigitalWrite = 11,
    PwmWrite = 12,
    SetDigitalMask = 13,   // output mask + digital value bitmap
    SetPwmBulk = 14,       // pwm output mask + pwm values
    RequestInputs = 15,

    // Device -> host
    InputsReport = 30,
    LogLine = 31,
    Error = 32,
};

enum class PinMode : uint8_t {
    Input = 0,
    Output = 1,
    InputPullup = 2,
};

struct HelloPayload {
    std::string board;   // e.g. "atmega328p_uno"
    uint32_t capabilities = 0;
};

struct RequestInputsPayload {
    uint32_t digital_mask = 0; // bit i corresponds to D{i} (0..13) and A0..A5 as 14..19
    uint8_t analog_mask = 0;   // bit i corresponds to A{i} (0..5)
};

struct DigitalOutputsPayload {
    uint32_t output_mask = 0; // pins that should be driven by the physical MCU
    uint32_t value_bits = 0;  // driven values for pins selected by output_mask
};

struct PwmOutputsPayload {
    uint32_t pwm_mask = 0;          // PWM pins currently driven by simulation
    uint8_t duty[20] = {0};         // index by Arduino pin number
};

struct InputsReportPayload {
    uint32_t digital_bits = 0;        // same indexing as RequestInputsPayload.digital_mask
    uint16_t analog[6] = {0,0,0,0,0,0}; // raw ADC 0..1023
};

struct Frame {
    MsgType type = MsgType::Error;
    uint8_t flags = 0;
    std::vector<uint8_t> payload;
};

uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t seed = 0xFFFF);

std::vector<uint8_t> encode_frame(const Frame& f);
std::optional<Frame> try_decode_one(std::vector<uint8_t>& inout_buffer);

// Helpers (payload encoding/decoding)
std::vector<uint8_t> encode_hello(const HelloPayload& p);
std::optional<HelloPayload> decode_hello(const std::vector<uint8_t>& bytes);

std::vector<uint8_t> encode_request_inputs(const RequestInputsPayload& p);
std::optional<RequestInputsPayload> decode_request_inputs(const std::vector<uint8_t>& bytes);

std::vector<uint8_t> encode_digital_outputs(const DigitalOutputsPayload& p);
std::optional<DigitalOutputsPayload> decode_digital_outputs(const std::vector<uint8_t>& bytes);

std::vector<uint8_t> encode_pwm_outputs(const PwmOutputsPayload& p);
std::optional<PwmOutputsPayload> decode_pwm_outputs(const std::vector<uint8_t>& bytes);

std::vector<uint8_t> encode_inputs_report(const InputsReportPayload& p);
std::optional<InputsReportPayload> decode_inputs_report(const std::vector<uint8_t>& bytes);

} // namespace mechatron::physical_mcu
