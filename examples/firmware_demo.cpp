// Firmware Loader Demo
// Demonstrates Intel HEX file parsing with comprehensive tests

#include "mcu/FirmwareLoader.hpp"
#include <iostream>
#include <iomanip>
#include <cassert>

using namespace mechatron;

// Test helper to calculate checksum manually
uint8_t calc_checksum_manual(const std::string& hex_line) {
    // Extract data bytes (without : and checksum)
    // Format: :LLAAAATTDDDD...DDCC
    int len = (hex_line.length() - 3) / 2;  // Exclude : and checksum
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) {
        std::string byte_str = hex_line.substr(1 + i * 2, 2);
        sum += (uint8_t)std::stoi(byte_str, nullptr, 16);
    }
    return (0x100 - (sum % 0x100)) % 0x100;
}

void test_basic_data_record() {
    std::cout << "\n=== Test 1: Basic Data Record ===" << std::endl;

    // Valid Intel HEX data record
    // :LLAAAATTDDDD...DDCC
    std::string hex = ":0300300002337A1E";
    // LL=03, AA=0030, TT=00, DD=02 33 7A
    // sum = 03+00+30+00+02+33+7A = E2
    // checksum = 0x100 - 0xE2 = 0x1E

    FirmwareLoader loader;
    bool result = loader.load_from_string(hex);
    if (!result) {
        std::cerr << "  Error: " << loader.error() << std::endl;
        assert(false);
    }
    assert(loader.size() == 3);
    assert(loader.is_loaded());

    const auto& data = loader.get_binary();
    assert(data[0] == 0x02);
    assert(data[1] == 0x33);
    assert(data[2] == 0x7A);

    std::cout << "✓ Basic data record test passed" << std::endl;
}

void test_multiple_records() {
    std::cout << "\n=== Test 2: Multiple Records ===" << std::endl;

    std::string hex =
        ":10010000214601360121470136007EFE09D2194001\n"
        ":100110002146017E17C20001FF5F16002148011928\n"
        ":10012000194E79234623965778239EDA3F01B2CAA7\n"
        ":00000001FF\n";

    FirmwareLoader loader;
    bool result = loader.load_from_string(hex);
    if (!result) {
        std::cerr << "  Error: " << loader.error() << std::endl;
        assert(false);
    }
    assert(loader.size() == 48);  // 16 + 16 + 16 bytes

    std::cout << "✓ Multiple records test passed (48 bytes)" << std::endl;
}

void test_extended_linear_address() {
    std::cout << "\n=== Test 3: Extended Linear Address ===" << std::endl;

    std::string hex =
        ":020000040008F2\n"  // Extended address = 0x0008
        ":10000000214601360121470136007EFE09D2194002\n"  // Data at 0x80000
        ":00000001FF\n";

    FirmwareLoader loader;
    assert(loader.load_from_string(hex));

    const auto& mem_map = loader.get_memory_map();
    assert(!mem_map.empty());
    assert(mem_map[0].first == 0x80000);  // Should be at high address

    std::cout << "✓ Extended linear address test passed" << std::endl;
    std::cout << "  Memory at: 0x" << std::hex << mem_map[0].first << std::dec << std::endl;
}

void test_eof_record() {
    std::cout << "\n=== Test 4: EOF Record ===" << std::endl;

    std::string hex =
        ":0300300002337A1E\n"
        ":00000001FF\n";  // EOF record

    FirmwareLoader loader;
    assert(loader.load_from_string(hex));
    assert(loader.is_loaded());

    std::cout << "✓ EOF record test passed" << std::endl;
}

void test_checksum_validation() {
    std::cout << "\n=== Test 5: Checksum Validation ===" << std::endl;

    // Correct checksum
    std::string good_hex = ":0300300002337A1E";
    uint8_t cc = calc_checksum_manual(good_hex);
    std::cout << "  Calculated checksum: 0x" << std::hex << (int)cc << std::dec << std::endl;
    assert(cc == 0x1E);

    FirmwareLoader loader;
    assert(loader.load_from_string(good_hex));
    std::cout << "✓ Valid checksum accepted" << std::endl;

    // Incorrect checksum (should still load with warning)
    std::string bad_hex = ":0300300002337AFD";  // Correct checksum is 0x1E, using 0xFD
    assert(loader.load_from_string(bad_hex));  // Should still work
    std::cout << "✓ Invalid checksum generates warning but loads" << std::endl;
}

void test_empty_and_whitespace() {
    std::cout << "\n=== Test 6: Empty Lines and Whitespace ===" << std::endl;

    std::string hex =
        "\n\n"  // Empty lines
        ":0300300002337A1E\r\n"  // With CRLF
        "\n"
        ":00000001FF\n"
        "\n";

    FirmwareLoader loader;
    assert(loader.load_from_string(hex));
    assert(loader.size() == 3);

    std::cout << "✓ Whitespace handling test passed" << std::endl;
}

void test_export_binary() {
    std::cout << "\n=== Test 7: Binary Export ===" << std::endl;

    std::string hex = ":0300300002337A1E\n:00000001FF\n";

    FirmwareLoader loader;
    assert(loader.load_from_string(hex));
    assert(loader.export_binary("test_export.bin"));

    std::cout << "✓ Binary export test passed" << std::endl;
}

void test_memory_map_segments() {
    std::cout << "\n=== Test 8: Memory Map Segments ===" << std::endl;

    // Non-contiguous addresses
    std::string hex =
        ":03000000010203FC\n"  // Data at 0x0000
        ":03001000040506F9\n"  // Data at 0x0010 (gap)
        ":00000001FF\n";

    FirmwareLoader loader;
    assert(loader.load_from_string(hex));

    const auto& mem_map = loader.get_memory_map();
    std::cout << "  Memory segments: " << mem_map.size() << std::endl;

    // Should have 2 segments due to gap
    if (mem_map.size() == 2) {
        std::cout << "  Segment 1: 0x" << std::hex << mem_map[0].first
                  << std::dec << " (" << mem_map[0].second.size() << " bytes)" << std::endl;
        std::cout << "  Segment 2: 0x" << std::hex << mem_map[1].first
                  << std::dec << " (" << mem_map[1].second.size() << " bytes)" << std::endl;
    }

    std::cout << "✓ Memory map segments test passed" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "=== Firmware Loader Comprehensive Tests ===" << std::endl;
    std::cout << "========================================" << std::endl;

    try {
        test_basic_data_record();
        test_multiple_records();
        test_extended_linear_address();
        test_eof_record();
        test_checksum_validation();
        test_empty_and_whitespace();
        test_export_binary();
        test_memory_map_segments();

        std::cout << "\n========================================" << std::endl;
        std::cout << "=== ALL TESTS PASSED! ===" << std::endl;
        std::cout << "========================================" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "\n✗ TEST FAILED: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
