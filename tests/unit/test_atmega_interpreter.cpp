#include "mcu/ATmegaInterpreter.hpp"
#include "mcu/QEMUInterface.hpp"

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

using namespace mechatron;

namespace {

std::string hex_record(uint16_t address, uint8_t type, const std::vector<uint8_t>& data) {
    uint8_t sum = static_cast<uint8_t>(data.size());
    sum = static_cast<uint8_t>(sum + (address >> 8));
    sum = static_cast<uint8_t>(sum + (address & 0xFF));
    sum = static_cast<uint8_t>(sum + type);
    for (uint8_t b : data) {
        sum = static_cast<uint8_t>(sum + b);
    }
    uint8_t checksum = static_cast<uint8_t>(~sum + 1);

    std::ostringstream out;
    out << ':';
    out << std::uppercase << std::hex;
    out.width(2); out.fill('0'); out << static_cast<int>(data.size());
    out.width(4); out.fill('0'); out << address;
    out.width(2); out.fill('0'); out << static_cast<int>(type);
    for (uint8_t b : data) {
        out.width(2); out.fill('0'); out << static_cast<int>(b);
    }
    out.width(2); out.fill('0'); out << static_cast<int>(checksum);
    return out.str();
}

std::filesystem::path write_hex_fixture(const std::string& name, const std::vector<uint16_t>& words) {
    std::vector<uint8_t> bytes;
    bytes.reserve(words.size() * 2);
    for (uint16_t w : words) {
        bytes.push_back(static_cast<uint8_t>(w & 0xFF));
        bytes.push_back(static_cast<uint8_t>((w >> 8) & 0xFF));
    }

    auto path = std::filesystem::temp_directory_path() / name;
    std::ofstream out(path);
    out << hex_record(0x0000, 0x00, bytes) << "\n";
    out << hex_record(0x0000, 0x01, {}) << "\n";
    return path;
}

} // namespace

TEST(ATmegaInterpreter, UsesVariantSpecificSramStart) {
    QEMUInterface mcu;
    mcu.set_mcu_variant_atmega328p();
    mcu.memory().write_data(0x0100, 0x42);

    EXPECT_EQ(mcu.memory().read_data(0x0100), 0x42);
    EXPECT_EQ(mcu.memory().sram[0], 0x42);

    mcu.memory().write_io(0x0100, 0x99);
    EXPECT_EQ(mcu.memory().read_data(0x0100), 0x42);

    mcu.set_mcu_variant_atmega2560();
    mcu.memory().write_data(0x0100, 0x77);
    EXPECT_EQ(mcu.memory().read_io(0x0100), 0x77);
    EXPECT_EQ(mcu.memory().sram[0], 0x00);

    mcu.memory().write_data(0x0200, 0x55);
    EXPECT_EQ(mcu.memory().sram[0], 0x55);
}

TEST(ATmegaInterpreter, CallAndRetRestoreWordAddressBeforeContinuing) {
    // 0: CALL 3
    // 2: SBI DDRB,5   ; Arduino D13 output
    // 3: RET
    auto hex = write_hex_fixture("mechatron_call_ret.hex", {
        0x940E, 0x0003, 0x9A25, 0x9508
    });

    QEMUInterface mcu;
    mcu.set_mcu_variant_atmega328p();
    ATmegaInterpreter interp(mcu);

    ASSERT_TRUE(mcu.launch(hex.string()));
    ASSERT_TRUE(interp.load_firmware(hex.string()));

    ASSERT_TRUE(interp.step()); // CALL 3
    ASSERT_TRUE(interp.step()); // RET -> 2
    ASSERT_TRUE(interp.step()); // SBI DDRB,5

    EXPECT_TRUE(mcu.is_digital_output(13));
}

TEST(QEMUInterface, PwmOutputSamplesSquareWaveInsteadOfAverageVoltage) {
    QEMUInterface mcu;
    mcu.set_mcu_variant_atmega328p();

    const auto& variant = mcu.mcu_variant();
    mcu.memory().write_io(variant.DDRD, static_cast<uint8_t>(1 << 3)); // D3/OC2B output
    mcu.memory().write_io(0xB0, 0x20); // TCCR2A COM2B non-inverting PWM enabled
    mcu.memory().write_io(variant.OCR2B, 120);

    float duty = 0.0f;
    ASSERT_TRUE(mcu.pwm_duty_fraction(3, duty));
    EXPECT_NEAR(duty, 120.0f / 255.0f, 0.001f);

    float voltage = -1.0f;
    ASSERT_TRUE(mcu.pwm_output_voltage(3, 0.0, 5.0f, 0.0f, voltage));
    EXPECT_FLOAT_EQ(voltage, 5.0f);

    ASSERT_TRUE(mcu.pwm_output_voltage(3, 0.001, 5.0f, 0.0f, voltage));
    EXPECT_FLOAT_EQ(voltage, 0.0f);

    EXPECT_NEAR(mcu.digital_output_voltage(3), (120.0f / 255.0f) * 5.0f, 0.001f);
}
