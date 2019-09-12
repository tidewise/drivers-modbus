#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <modbus/RTU.hpp>
#include <modbus/Exceptions.hpp>

using namespace std;
using namespace modbus;
using testing::ElementsAreArray;

struct RTUTest : public ::testing::Test {
};

TEST_F(RTUTest, it_computes_the_CRC) {
    uint8_t bytes[] = { 1, 2, 3, 4, 5, 6 };
    auto crc = RTU::crc(bytes, bytes + 6);
    // Computed with https://www.lammertbies.nl/comm/info/crc-calculation.html
    // First byte is LSB
    ASSERT_EQ(0xBA, crc[0]);
    ASSERT_EQ(0xDD, crc[1]);
}

TEST_F(RTUTest, it_computes_the_interframe_timeout_for_bitrates_smaller_than_19200) {
    auto time = RTU::interframeDuration(9600);
    ASSERT_EQ(4011, time.toMicroseconds());
}

TEST_F(RTUTest, it_computes_the_interframe_timeout_for_19200_bps) {
    auto time = RTU::interframeDuration(19200);
    ASSERT_EQ(2006, time.toMicroseconds());
}

TEST_F(RTUTest, it_computes_the_interframe_timeout_for_bitrates_higher_than_19200) {
    auto time = RTU::interframeDuration(36400);
    ASSERT_EQ(1750, time.toMicroseconds());
}

TEST_F(RTUTest, it_formats_a_frame) {
    uint8_t buffer[256];
    uint8_t payload[5] = { 1, 2, 3, 4, 5 };
    uint8_t* bufferEnd = RTU::formatFrame(buffer, 0x2, 0x10, payload, payload + 5);
    ASSERT_EQ(9, bufferEnd - buffer);

    // CRC computed with https://www.lammertbies.nl/comm/info/crc-calculation.html
    uint8_t expected[] = { 0x02, 0x10, 1, 2, 3, 4, 5, 0x34, 0xEB };
    ASSERT_THAT(std::vector<uint8_t>(buffer, bufferEnd),
                ElementsAreArray(expected));
}

TEST_F(RTUTest, it_throws_if_the_resulting_frame_would_be_bigger_than_256_bytes) {
    uint8_t buffer[0];
    ASSERT_THROW(RTU::formatFrame(nullptr, 0x2, 0x10, buffer, buffer + 253),
                 std::invalid_argument);
}

TEST_F(RTUTest, it_parses_a_frame) {
    uint8_t bytes[] = { 0x02, 0x10, 1, 2, 3, 4, 5, 0x34, 0xEB };
    Frame frame = RTU::parseFrame(bytes, bytes + 9);
    ASSERT_EQ(0x02, frame.address);
    ASSERT_EQ(0x10, frame.function);

    // CRC computed with https://www.lammertbies.nl/comm/info/crc-calculation.html
    uint8_t payload[5] = { 1, 2, 3, 4, 5 };
    ASSERT_THAT(frame.payload, ElementsAreArray(payload));
}

TEST_F(RTUTest, it_throws_if_attempting_to_parse_a_buffer_that_is_too_small) {
    uint8_t bytes[0];
    ASSERT_THROW(RTU::parseFrame(bytes, bytes + 3), RTU::TooSmall);
}

TEST_F(RTUTest, it_throws_if_the_CRC_check_fails) {
    uint8_t bytes[] = { 0x02, 0x10, 1, 2, 3, 4, 5, 0x34, 0xEA };
    ASSERT_THROW(RTU::parseFrame(bytes, bytes + 9), RTU::InvalidCRC);
}

TEST_F(RTUTest, it_throws_if_the_buffer_is_too_small_to_contain_a_RTU_frame) {
    uint8_t bytes[0];
    ASSERT_THROW(RTU::isCRCValid(bytes, bytes + 3), RTU::TooSmall);
}

TEST_F(RTUTest, it_recognizes_a_valid_CRC) {
    uint8_t frame[] = { 0x02, 0x10, 1, 2, 3, 4, 5, 0x34, 0xEB };
    ASSERT_TRUE(RTU::isCRCValid(frame, frame + 9));
}

TEST_F(RTUTest, it_recognizes_an_invalid_CRC_first_byte) {
    uint8_t frame[] = { 0x02, 0x10, 1, 2, 3, 4, 5, 0, 0xEB };
    ASSERT_FALSE(RTU::isCRCValid(frame, frame + 9));
}

TEST_F(RTUTest, it_recognizes_an_invalid_CRC_second_byte) {
    uint8_t frame[] = { 0x02, 0x10, 1, 2, 3, 4, 5, 0x34, 0x0 };
    ASSERT_FALSE(RTU::isCRCValid(frame, frame + 9));
}

TEST_F(RTUTest, it_formats_a_read_holding_registers_command) {
    uint8_t buffer[8];
    uint8_t* end = RTU::formatReadRegisters(buffer, 0x10, false, 0x1020, 0x05);
    ASSERT_EQ(end - buffer, 8);

    uint8_t expected[] = { 0x10, 0x03, 0x10, 0x20, 0x00, 0x05 };
    ASSERT_THAT(std::vector<uint8_t>(buffer, end - 2),
                ElementsAreArray(expected));
}

TEST_F(RTUTest, it_formats_a_read_input_registers_command) {
    uint8_t buffer[8];
    uint8_t* end = RTU::formatReadRegisters(buffer, 0x10, true, 0x1020, 0x05);
    ASSERT_EQ(end - buffer, 8);

    uint8_t expected[] = { 0x10, 0x04, 0x10, 0x20, 0x00, 0x05 };
    ASSERT_THAT(std::vector<uint8_t>(buffer, end - 2),
                ElementsAreArray(expected));
}

TEST_F(RTUTest, it_throws_if_attempting_to_read_beyond_register_65536) {
    ASSERT_THROW(RTU::formatReadRegisters(nullptr, 0x10, false, 0xfffe, 2),
                 std::invalid_argument);
}

TEST_F(RTUTest, it_throws_if_attempting_to_read_more_than_128_registers) {
    ASSERT_THROW(RTU::formatReadRegisters(nullptr, 0x10, false, 0, 129),
                 std::invalid_argument);
}

TEST_F(RTUTest, it_parses_a_read_register_reply) {
    Frame frame = { 0x10, 0x03, { 0x04, 0x1, 0x2, 0x3, 0x4 } };
    uint16_t values[2];
    RTU::parseReadRegisters(values, frame, 2);

    uint16_t expected[] = { 0x0102, 0x0304 };
    ASSERT_THAT(vector<uint16_t>(values, values + 2), ElementsAreArray(expected));
}

TEST_F(RTUTest, it_throws_if_the_amount_of_registers_in_the_reply_is_smaller_than_the_expected) {
    Frame frame = { 0x10, 0x03, { 2, 1, 2 } };
    ASSERT_THROW(RTU::parseReadRegisters(nullptr, frame, 2), UnexpectedReply);
}

TEST_F(RTUTest, it_throws_if_the_amount_of_registers_in_the_reply_is_greater_than_the_expected) {
    Frame frame = { 0x10, 0x03, { 6, 1, 2, 3, 4, 5, 6 } };
    ASSERT_THROW(RTU::parseReadRegisters(nullptr, frame, 2), UnexpectedReply);
}

TEST_F(RTUTest, it_throws_if_the_byte_count_is_odd) {
    Frame frame = { 0x10, 0x03, { 5, 1, 2, 3, 4, 5 } };
    ASSERT_THROW(RTU::parseReadRegisters(nullptr, frame, 2), UnexpectedReply);
}

TEST_F(RTUTest, it_throws_if_the_amount_of_registers_in_the_reply_would_take_more_space_than_the_frame_payload) {
    Frame frame = { 0x10, 0x03, { 0x06, 0x1, 0x2, 0x3, 0x4, 0x5 } };
    ASSERT_THROW(RTU::parseReadRegisters(nullptr, frame, 2),
                 UnexpectedReply);
}

TEST_F(RTUTest, it_throws_if_the_amount_of_registers_in_the_reply_would_take_fewer_space_than_the_frame_payload) {
    Frame frame = { 0x10, 0x03, { 0x06, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7 } };
    ASSERT_THROW(RTU::parseReadRegisters(nullptr, frame, 2),
                 UnexpectedReply);
}
