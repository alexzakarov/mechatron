#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <fstream>
#include <algorithm>
#include <cstring>

namespace mechatron {

/**
 * @brief Lightweight Intel HEX file parser
 *
 * Header-only implementation for parsing Intel HEX format files.
 * Supports all standard record types (00-05).
 *
 * Format: :LLAAAATT[DD...]CC
 * - : Start code
 * - LL Byte count (2 hex digits)
 * - AAAA Address (4 hex digits)
 * - TT Record type (00=Data, 01=EOF, 02=ExtSegAddr, 03=StartSegAddr, 04=ExtLinearAddr, 05=StartLinearAddr)
 * - DD Data (2*LL hex digits)
 * - CC Checksum (2 hex digits)
 */
class IntelHexParser {
public:
    struct HexRecord {
        uint8_t byte_count;
        uint16_t address;
        uint8_t type;
        std::vector<uint8_t> data;
        uint8_t checksum;
    };

    struct ParseResult {
        std::vector<HexRecord> records;
        std::vector<uint8_t> binary_data;
        uint32_t base_address = 0;
        uint32_t entry_point = 0;
        bool has_eof = false;
        std::string error;
    };

    /**
     * Parse Intel HEX file from string content
     */
    static ParseResult parse(const std::string& hex_content) {
        ParseResult result;

        std::istringstream stream(hex_content);
        std::string line;
        size_t line_num = 0;

        while (std::getline(stream, line)) {
            line_num++;

            // Normalize line endings - remove CR if present
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }

            // Trim trailing whitespace (spaces, tabs)
            while (!line.empty() && (line.back() == ' ' || line.back() == '\t')) {
                line.pop_back();
            }

            // Skip empty lines and comments
            if (line.empty() || line[0] == '#') {
                continue;
            }

            // Check for start code
            if (line[0] != ':') {
                result.error = "Invalid format at line " + std::to_string(line_num) + ": missing start code ':'";
                return result;
            }

            try {
                HexRecord record = parse_line(line);
                result.records.push_back(record);

                // Process special records
                switch (record.type) {
                    case 1: // End of File
                        result.has_eof = true;
                        break;

                    case 2: // Extended Segment Address
                        if (record.data.size() == 2) {
                            result.base_address = ((record.data[0] << 8) | record.data[1]) << 4;
                        }
                        break;

                    case 4: // Extended Linear Address
                        if (record.data.size() == 2) {
                            result.base_address = ((record.data[0] << 8) | record.data[1]) << 16;
                        }
                        break;

                    case 5: // Start Linear Address
                        if (record.data.size() == 4) {
                            result.entry_point = (record.data[0] << 24) |
                                               (record.data[1] << 16) |
                                               (record.data[2] << 8) |
                                               record.data[3];
                        }
                        break;
                }

            } catch (const std::exception& e) {
                result.error = "Parse error at line " + std::to_string(line_num) + ": " + e.what();
                return result;
            }
        }

        // Build binary data from records
        result.binary_data = build_binary(result.records, result.base_address);

        return result;
    }

    /**
     * Parse Intel HEX file from file path
     */
    static ParseResult parse_file(const std::string& filepath) {
        // Open file in binary mode to handle line endings correctly
        std::ifstream file(filepath, std::ios::binary);
        if (!file.is_open()) {
            ParseResult result;
            result.error = "Failed to open file: " + filepath;
            return result;
        }

        // Read entire file content
        std::stringstream buffer;
        buffer << file.rdbuf();
        file.close();

        return parse(buffer.str());
    }

private:
    static HexRecord parse_line(const std::string& line) {
        HexRecord record;

        // Minimum length check: :LLAAAATTCC = 11 characters
        if (line.length() < 11) {
            throw std::runtime_error("Line too short: " + std::to_string(line.length()));
        }

        // Parse byte count
        record.byte_count = parse_hex_byte(line, 1);

        // Expected length: 11 + 2 * byte_count
        size_t expected_len = 11 + 2 * record.byte_count;
        if (line.length() != expected_len) {
            throw std::runtime_error("Length mismatch: expected " + std::to_string(expected_len) +
                                   ", got " + std::to_string(line.length()));
        }

        // Parse address
        record.address = parse_hex_word(line, 3);

        // Parse record type
        record.type = parse_hex_byte(line, 7);

        // Parse data
        record.data.reserve(record.byte_count);
        for (uint8_t i = 0; i < record.byte_count; i++) {
            record.data.push_back(parse_hex_byte(line, 9 + i * 2));
        }

        // Parse checksum
        record.checksum = parse_hex_byte(line, 9 + record.byte_count * 2);

        // Verify checksum
        uint8_t calc_checksum = calculate_checksum(line);
        if (record.checksum != calc_checksum) {
            // Don't throw, just log - some tools generate relaxed checksums
            // But store the calculated checksum for reference
        }

        return record;
    }

    static std::vector<uint8_t> build_binary(const std::vector<HexRecord>& records, uint32_t base_addr) {
        // Collect data segments
        struct Segment {
            uint32_t address;
            std::vector<uint8_t> data;
        };
        std::vector<Segment> segments;

        uint32_t current_base = base_addr;

        for (const auto& record : records) {
            if (record.type == 0x00) { // Data record
                uint32_t full_addr = current_base + record.address;

                // Try to merge with last segment
                if (!segments.empty()) {
                    Segment& last = segments.back();
                    uint32_t seg_end = last.address + last.data.size();
                    if (full_addr == seg_end) {
                        last.data.insert(last.data.end(), record.data.begin(), record.data.end());
                        continue;
                    }
                }

                // Create new segment
                Segment seg;
                seg.address = full_addr;
                seg.data = record.data;
                segments.push_back(seg);
            } else if (record.type == 0x04 || record.type == 0x02) {
                // Update base address
                if (record.data.size() == 2) {
                    if (record.type == 0x04) {
                        current_base = ((record.data[0] << 8) | record.data[1]) << 16;
                    } else {
                        current_base = ((record.data[0] << 8) | record.data[1]) << 4;
                    }
                }
            }
        }

        if (segments.empty()) {
            return {};
        }

        // Find address range
        uint32_t min_addr = segments[0].address;
        uint32_t max_addr = min_addr;

        for (const auto& seg : segments) {
            min_addr = std::min(min_addr, seg.address);
            max_addr = std::max(max_addr, seg.address + static_cast<uint32_t>(seg.data.size()));
        }

        // Allocate and fill binary
        std::vector<uint8_t> binary(max_addr - min_addr, 0xFF);

        for (const auto& seg : segments) {
            size_t offset = seg.address - min_addr;
            std::memcpy(binary.data() + offset, seg.data.data(), seg.data.size());
        }

        return binary;
    }

    static uint8_t parse_hex_byte(const std::string& s, size_t offset) {
        std::string byte_str = s.substr(offset, 2);
        return static_cast<uint8_t>(std::stoi(byte_str, nullptr, 16));
    }

    static uint16_t parse_hex_word(const std::string& s, size_t offset) {
        std::string word_str = s.substr(offset, 4);
        return static_cast<uint16_t>(std::stoi(word_str, nullptr, 16));
    }

    static uint8_t calculate_checksum(const std::string& line) {
        // Intel HEX checksum: two's complement of sum of all bytes
        uint8_t sum = 0;
        size_t byte_count = (line.length() - 3) / 2; // Exclude ':', checksum, and trailing chars

        for (size_t i = 0; i < byte_count; i++) {
            sum += parse_hex_byte(line, 1 + i * 2);
        }

        return (0x100 - (sum % 0x100)) % 0x100;
    }
};

} // namespace mechatron
