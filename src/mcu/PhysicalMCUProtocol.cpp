#include "PhysicalMCUProtocol.hpp"
#include <algorithm>
#include <cstring>

namespace mechatron::physical_mcu {

static inline void append_u16_le(std::vector<uint8_t>& out, uint16_t v) {
    out.push_back(static_cast<uint8_t>(v & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

static inline void append_u32_le(std::vector<uint8_t>& out, uint32_t v) {
    out.push_back(static_cast<uint8_t>(v & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
}

static inline bool read_u16_le(const std::vector<uint8_t>& in, size_t off, uint16_t& out) {
    if (off + 2 > in.size()) return false;
    out = static_cast<uint16_t>(in[off]) | (static_cast<uint16_t>(in[off + 1]) << 8);
    return true;
}

static inline bool read_u32_le(const std::vector<uint8_t>& in, size_t off, uint32_t& out) {
    if (off + 4 > in.size()) return false;
    out = static_cast<uint32_t>(in[off]) |
          (static_cast<uint32_t>(in[off + 1]) << 8) |
          (static_cast<uint32_t>(in[off + 2]) << 16) |
          (static_cast<uint32_t>(in[off + 3]) << 24);
    return true;
}

uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t seed) {
    uint16_t crc = seed;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

std::vector<uint8_t> encode_frame(const Frame& f) {
    std::vector<uint8_t> out;
    out.reserve(4 + 1 + 1 + 1 + 2 + 2 + f.payload.size());
    out.push_back('M'); out.push_back('C'); out.push_back('H'); out.push_back('T');
    out.push_back(kVersion);
    out.push_back(static_cast<uint8_t>(f.type));
    out.push_back(f.flags);
    append_u16_le(out, static_cast<uint16_t>(f.payload.size()));

    // CRC placeholder
    append_u16_le(out, 0);
    out.insert(out.end(), f.payload.begin(), f.payload.end());

    // Compute CRC over version..len + payload (i.e. skip magic[4], and write into crc field)
    const size_t crc_start = 4;
    const size_t crc_len = out.size() - crc_start;
    uint16_t crc = crc16_ccitt(out.data() + crc_start, crc_len, 0xFFFF);
    out[4 + 1 + 1 + 1 + 2 + 0] = static_cast<uint8_t>(crc & 0xFF);
    out[4 + 1 + 1 + 1 + 2 + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    return out;
}

std::optional<Frame> try_decode_one(std::vector<uint8_t>& inout_buffer) {
    // Find magic
    size_t i = 0;
    while (i + 4 <= inout_buffer.size()) {
        if (inout_buffer[i] == 'M' && inout_buffer[i + 1] == 'C' &&
            inout_buffer[i + 2] == 'H' && inout_buffer[i + 3] == 'T') {
            break;
        }
        ++i;
    }
    if (i > 0) {
        inout_buffer.erase(inout_buffer.begin(), inout_buffer.begin() + static_cast<long>(i));
    }
    if (inout_buffer.size() < 4 + 1 + 1 + 1 + 2 + 2) return std::nullopt;

    const uint8_t version = inout_buffer[4];
    if (version != kVersion) {
        // Drop magic byte and resync
        inout_buffer.erase(inout_buffer.begin());
        return std::nullopt;
    }

    const MsgType type = static_cast<MsgType>(inout_buffer[5]);
    const uint8_t flags = inout_buffer[6];
    uint16_t len = 0;
    uint16_t crc = 0;
    if (!read_u16_le(inout_buffer, 7, len)) return std::nullopt;
    if (!read_u16_le(inout_buffer, 9, crc)) return std::nullopt;

    const size_t frame_size = 4 + 1 + 1 + 1 + 2 + 2 + len;
    if (inout_buffer.size() < frame_size) return std::nullopt;

    // Validate CRC over version..len + payload (same bytes we used in encoder)
    const size_t crc_start = 4;
    const size_t crc_len = (frame_size - crc_start);
    std::vector<uint8_t> temp(inout_buffer.begin() + static_cast<long>(crc_start),
                              inout_buffer.begin() + static_cast<long>(frame_size));
    // zero crc bytes inside temp (at offset 1+1+1+2)
    temp[1 + 1 + 1 + 2 + 0] = 0;
    temp[1 + 1 + 1 + 2 + 1] = 0;
    uint16_t calc = crc16_ccitt(temp.data(), crc_len, 0xFFFF);
    if (calc != crc) {
        // Corrupt; drop first byte and resync
        inout_buffer.erase(inout_buffer.begin());
        return std::nullopt;
    }

    Frame f;
    f.type = type;
    f.flags = flags;
    f.payload.assign(inout_buffer.begin() + static_cast<long>(4 + 1 + 1 + 1 + 2 + 2),
                     inout_buffer.begin() + static_cast<long>(frame_size));

    inout_buffer.erase(inout_buffer.begin(), inout_buffer.begin() + static_cast<long>(frame_size));
    return f;
}

std::vector<uint8_t> encode_hello(const HelloPayload& p) {
    std::vector<uint8_t> out;
    if (p.board.size() > 63) {
        // hard cap for safety
        out.push_back(0);
    } else {
        out.push_back(static_cast<uint8_t>(p.board.size()));
        out.insert(out.end(), p.board.begin(), p.board.end());
    }
    append_u32_le(out, p.capabilities);
    return out;
}

std::optional<HelloPayload> decode_hello(const std::vector<uint8_t>& bytes) {
    if (bytes.empty()) return std::nullopt;
    const uint8_t n = bytes[0];
    if (1 + n + 4 > bytes.size()) return std::nullopt;
    HelloPayload p;
    p.board.assign(reinterpret_cast<const char*>(bytes.data() + 1), n);
    uint32_t caps = 0;
    if (!read_u32_le(bytes, 1 + n, caps)) return std::nullopt;
    p.capabilities = caps;
    return p;
}

std::vector<uint8_t> encode_request_inputs(const RequestInputsPayload& p) {
    std::vector<uint8_t> out;
    append_u32_le(out, p.digital_mask);
    out.push_back(p.analog_mask);
    return out;
}

std::optional<RequestInputsPayload> decode_request_inputs(const std::vector<uint8_t>& bytes) {
    if (bytes.size() < 5) return std::nullopt;
    RequestInputsPayload p;
    if (!read_u32_le(bytes, 0, p.digital_mask)) return std::nullopt;
    p.analog_mask = bytes[4];
    return p;
}

std::vector<uint8_t> encode_digital_outputs(const DigitalOutputsPayload& p) {
    std::vector<uint8_t> out;
    append_u32_le(out, p.output_mask);
    append_u32_le(out, p.value_bits);
    return out;
}

std::optional<DigitalOutputsPayload> decode_digital_outputs(const std::vector<uint8_t>& bytes) {
    if (bytes.size() < 8) return std::nullopt;
    DigitalOutputsPayload p;
    if (!read_u32_le(bytes, 0, p.output_mask)) return std::nullopt;
    if (!read_u32_le(bytes, 4, p.value_bits)) return std::nullopt;
    return p;
}

std::vector<uint8_t> encode_pwm_outputs(const PwmOutputsPayload& p) {
    std::vector<uint8_t> out;
    append_u32_le(out, p.pwm_mask);
    out.insert(out.end(), std::begin(p.duty), std::end(p.duty));
    return out;
}

std::optional<PwmOutputsPayload> decode_pwm_outputs(const std::vector<uint8_t>& bytes) {
    if (bytes.size() < 24) return std::nullopt;
    PwmOutputsPayload p;
    if (!read_u32_le(bytes, 0, p.pwm_mask)) return std::nullopt;
    std::copy(bytes.begin() + 4, bytes.begin() + 24, std::begin(p.duty));
    return p;
}

std::vector<uint8_t> encode_inputs_report(const InputsReportPayload& p) {
    std::vector<uint8_t> out;
    append_u32_le(out, p.digital_bits);
    for (int i = 0; i < 6; ++i) {
        append_u16_le(out, p.analog[i]);
    }
    return out;
}

std::optional<InputsReportPayload> decode_inputs_report(const std::vector<uint8_t>& bytes) {
    if (bytes.size() < 4 + 6 * 2) return std::nullopt;
    InputsReportPayload p;
    if (!read_u32_le(bytes, 0, p.digital_bits)) return std::nullopt;
    for (int i = 0; i < 6; ++i) {
        if (!read_u16_le(bytes, 4 + i * 2, p.analog[i])) return std::nullopt;
    }
    return p;
}

} // namespace mechatron::physical_mcu
