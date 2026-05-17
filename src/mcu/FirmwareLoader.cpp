#include "FirmwareLoader.hpp"
#include "IntelHexParser.hpp"
#include <spdlog/spdlog.h>
#include <fstream>
#include <sstream>
#include <algorithm>

namespace mechatron {

FirmwareLoader::FirmwareLoader()
    : m_entry_point(0)
    , m_base_address(0)
    , m_loaded(false)
    , m_error("")
{
}

bool FirmwareLoader::load(const std::string& path) {
    spdlog::debug("Loading firmware from: {}", path);

    // Clear previous data
    clear();

    // Use the new parser
    auto result = IntelHexParser::parse_file(path);

    if (!result.error.empty()) {
        m_error = result.error;
        spdlog::error(m_error);
        return false;
    }

    if (!result.has_eof) {
        spdlog::warn("Missing EOF record in firmware file");
    }

    // Convert parser records to our format
    for (const auto& pr : result.records) {
        HexRecord record;
        record.byte_count = pr.byte_count;
        record.address = pr.address;
        record.type = static_cast<HexRecordType>(pr.type);
        record.data = pr.data;
        record.checksum = pr.checksum;
        m_records.push_back(record);
    }

    m_binary_data = result.binary_data;
    m_entry_point = result.entry_point;
    m_base_address = result.base_address;

    // Build memory map
    build_memory_map();

    m_loaded = true;
    spdlog::debug("Firmware loaded: {} bytes, {} records",
                 m_binary_data.size(), m_records.size());

    return true;
}

bool FirmwareLoader::load_from_string(const std::string& hex_data) {
    spdlog::debug("Loading firmware from string ({} bytes)", hex_data.size());

    // Clear previous data
    clear();

    // Use the new parser
    auto result = IntelHexParser::parse(hex_data);

    if (!result.error.empty()) {
        m_error = result.error;
        spdlog::error(m_error);
        return false;
    }

    if (!result.has_eof) {
        spdlog::warn("Missing EOF record in firmware data");
    }

    // Convert parser records to our format
    for (const auto& pr : result.records) {
        HexRecord record;
        record.byte_count = pr.byte_count;
        record.address = pr.address;
        record.type = static_cast<HexRecordType>(pr.type);
        record.data = pr.data;
        record.checksum = pr.checksum;
        m_records.push_back(record);
    }

    m_binary_data = result.binary_data;
    m_entry_point = result.entry_point;
    m_base_address = result.base_address;

    // Build memory map
    build_memory_map();

    m_loaded = true;
    spdlog::debug("Firmware loaded from string: {} bytes", m_binary_data.size());

    return true;
}

void FirmwareLoader::build_memory_map() {
    uint32_t current_base = m_base_address;

    for (const auto& record : m_records) {
        if (record.type == HexRecordType::Data) {
            uint32_t full_addr = current_base + record.address;
            bool merged = false;

            // Try to merge with existing segment
            for (auto& segment : m_memory_map) {
                uint32_t seg_end = segment.first + segment.second.size();
                if (full_addr == seg_end) {
                    // Append to existing segment
                    segment.second.insert(segment.second.end(),
                                        record.data.begin(), record.data.end());
                    merged = true;
                    break;
                }
            }

            // Create new segment if not merged
            if (!merged) {
                m_memory_map.push_back({full_addr, record.data});
            }
        } else if (record.type == HexRecordType::ExtendedLinearAddr) {
            if (record.data.size() == 2) {
                m_base_address = (record.data[0] << 8) | record.data[1];
                m_base_address <<= 16;
                current_base = m_base_address;
            }
        } else if (record.type == HexRecordType::ExtendedSegmentAddr) {
            if (record.data.size() == 2) {
                m_base_address = (record.data[0] << 8) | record.data[1];
                m_base_address <<= 4;
                current_base = m_base_address;
            }
        }
    }
}

void FirmwareLoader::clear() {
    m_records.clear();
    m_binary_data.clear();
    m_memory_map.clear();
    m_entry_point = 0;
    m_base_address = 0;
    m_loaded = false;
    m_error.clear();
}

bool FirmwareLoader::export_binary(const std::string& path) {
    if (!m_loaded || m_binary_data.empty()) {
        m_error = "No firmware data to export";
        return false;
    }

    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) {
        m_error = "Failed to create file: " + path;
        return false;
    }

    file.write(reinterpret_cast<const char*>(m_binary_data.data()),
              m_binary_data.size());
    file.close();

    spdlog::info("Exported {} bytes to {}", m_binary_data.size(), path);
    return true;
}

} // namespace mechatron
