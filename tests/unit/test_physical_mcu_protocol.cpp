#include "mcu/PhysicalMCUProtocol.hpp"
#include <gtest/gtest.h>

using namespace mechatron::physical_mcu;

TEST(PhysicalMCUProtocol, FrameRoundTripWithNoise) {
    Frame frame;
    frame.type = MsgType::InputsReport;
    frame.payload = {1, 2, 3, 4, 5};

    auto encoded = encode_frame(frame);
    std::vector<uint8_t> rx = {0xAA, 0x55, 0x00};
    rx.insert(rx.end(), encoded.begin(), encoded.end());

    auto decoded = try_decode_one(rx);
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->type, MsgType::InputsReport);
    EXPECT_EQ(decoded->payload, frame.payload);
    EXPECT_TRUE(rx.empty());
}

TEST(PhysicalMCUProtocol, CorruptFrameResyncsToNextFrame) {
    Frame bad;
    bad.type = MsgType::Hello;
    bad.payload = encode_hello({"host", 0});

    Frame good;
    good.type = MsgType::HelloAck;
    good.payload = encode_hello({"atmega328p_uno", 0x1234});

    auto bad_bytes = encode_frame(bad);
    bad_bytes.back() ^= 0x80;
    auto good_bytes = encode_frame(good);

    std::vector<uint8_t> rx;
    rx.insert(rx.end(), bad_bytes.begin(), bad_bytes.end());
    rx.insert(rx.end(), good_bytes.begin(), good_bytes.end());

    EXPECT_FALSE(try_decode_one(rx).has_value());
    auto decoded = try_decode_one(rx);
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->type, MsgType::HelloAck);

    auto hello = decode_hello(decoded->payload);
    ASSERT_TRUE(hello.has_value());
    EXPECT_EQ(hello->board, "atmega328p_uno");
    EXPECT_EQ(hello->capabilities, 0x1234u);
}

TEST(PhysicalMCUProtocol, PreservesTwentyDigitalPins) {
    RequestInputsPayload request;
    request.digital_mask = (1u << 20) - 1u;
    request.analog_mask = 0x3F;

    auto decoded_request = decode_request_inputs(encode_request_inputs(request));
    ASSERT_TRUE(decoded_request.has_value());
    EXPECT_EQ(decoded_request->digital_mask, request.digital_mask);
    EXPECT_EQ(decoded_request->analog_mask, request.analog_mask);

    InputsReportPayload report;
    report.digital_bits = (1u << 19) | (1u << 14) | (1u << 3);
    report.analog[0] = 17;
    report.analog[5] = 1023;

    auto decoded_report = decode_inputs_report(encode_inputs_report(report));
    ASSERT_TRUE(decoded_report.has_value());
    EXPECT_EQ(decoded_report->digital_bits, report.digital_bits);
    EXPECT_EQ(decoded_report->analog[0], 17);
    EXPECT_EQ(decoded_report->analog[5], 1023);
}

TEST(PhysicalMCUProtocol, OutputAndPwmPayloadsRoundTrip) {
    DigitalOutputsPayload digital;
    digital.output_mask = (1u << 19) | (1u << 13) | (1u << 2);
    digital.value_bits = (1u << 19) | (1u << 2);

    auto decoded_digital = decode_digital_outputs(encode_digital_outputs(digital));
    ASSERT_TRUE(decoded_digital.has_value());
    EXPECT_EQ(decoded_digital->output_mask, digital.output_mask);
    EXPECT_EQ(decoded_digital->value_bits, digital.value_bits);

    PwmOutputsPayload pwm;
    pwm.pwm_mask = (1u << 3) | (1u << 11);
    pwm.duty[3] = 128;
    pwm.duty[11] = 64;

    auto decoded_pwm = decode_pwm_outputs(encode_pwm_outputs(pwm));
    ASSERT_TRUE(decoded_pwm.has_value());
    EXPECT_EQ(decoded_pwm->pwm_mask, pwm.pwm_mask);
    EXPECT_EQ(decoded_pwm->duty[3], 128);
    EXPECT_EQ(decoded_pwm->duty[11], 64);
}

