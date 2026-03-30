#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <cstddef>
#include <cstring>

namespace mechatron {

/**
 * Intel HEX record type
 */
enum class HexRecordType : uint8_t {
    Data = 0x00,              // Data record
    EndOfFile = 0x01,         // End of file record
    ExtendedSegmentAddr = 0x02, // Extended segment address record
    StartSegmentAddr = 0x03,   // Start segment address record
    ExtendedLinearAddr = 0x04, // Extended linear address record
    StartLinearAddr = 0x05     // Start linear address record
};

/**
 * Parsed Intel HEX record
 */
struct HexRecord {
    uint8_t byte_count;
    uint16_t address;
    HexRecordType type;
    std::vector<uint8_t> data;
    uint8_t checksum;
};

/**
 * Firmware loader for Intel HEX format (.hex files)
 *
 * Intel HEX format:
 * :LLAAAATTDDDD...DDCC
 * L = Byte count (2 hex digits)
 * A = Address (4 hex digits)
 * T = Record type (2 hex digits)
 * D = Data (2 hex digits per byte)
 * C = Checksum (2 hex digits)
 */
class FirmwareLoader {
public:
    FirmwareLoader();
    ~FirmwareLoader() = default;

    /**
     * Load firmware from .hex file
     * @param path Path to .hex file
     * @return true if successful
     */
    bool load(const std::string& path);

    /**
     * Load firmware from hex string
     * @param hex_data Intel HEX formatted string
     * @return true if successful
     */
    bool load_from_string(const std::string& hex_data);

    /**
     * Get firmware binary data
     * @return Vector of bytes
     */
    const std::vector<uint8_t>& get_binary() const { return m_binary_data; }

    /**
     * Get firmware as memory map (address -> data)
     * Useful for flashing to specific memory addresses
     */
    const std::vector<std::pair<uint32_t, std::vector<uint8_t>>>& get_memory_map() const {
        return m_memory_map;
    }

    /**
     * Get entry point address (if specified)
     */
    uint32_t get_entry_point() const { return m_entry_point; }

    /**
     * Get total size of firmware in bytes
     */
    size_t size() const { return m_binary_data.size(); }

    /**
     * Check if firmware is loaded
     */
    bool is_loaded() const { return m_loaded; }

    /**
     * Get last error message
     */
    const std::string& error() const { return m_error; }

    /**
     * Clear loaded data
     */
    void clear();

    /**
     * Export firmware to raw binary file
     * @param path Output file path
     * @return true if successful
     */
    bool export_binary(const std::string& path);

private:
    void build_memory_map();

    std::vector<HexRecord> m_records;
    std::vector<uint8_t> m_binary_data;
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> m_memory_map;
    uint32_t m_entry_point;
    uint32_t m_base_address;  // Extended linear address
    bool m_loaded;
    std::string m_error;
};

} // namespace mechatron
