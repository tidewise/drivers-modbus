#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <modbus/TCP.hpp>
#include <modbus/Exceptions.hpp>

using namespace std;
using namespace modbus;
using testing::ElementsAreArray;

struct TCPTest : public ::testing::Test {
};

TEST_F(TCPTest, it_formats_a_frame) {
    uint8_t buffer[256];
    uint8_t payload[5] = { 1, 2, 3, 4, 5 };
    uint8_t* bufferEnd = TCP::formatFrame(buffer, 0xabcd, 0x2, 0x10, payload, payload + 5);
    ASSERT_EQ(13, bufferEnd - buffer);

    uint8_t expected[] = { 0xab, 0xcd, 0, 0, 0, 7, 2, 0x10, 1, 2, 3, 4, 5 };
    ASSERT_THAT(std::vector<uint8_t>(buffer, bufferEnd),
                ElementsAreArray(expected));
}

TEST_F(TCPTest, it_handles_frames_up_to_the_maximum_allowed_by_the_frame_length) {
    std::vector<uint8_t> payload;
    payload.resize(65533);
    for (auto& v : payload) {
        v = rand();
    }

    std::vector<uint8_t> buffer;
    buffer.resize(65541);
    uint8_t* bufferEnd = TCP::formatFrame(
        &buffer[0], 0x1234, 0x5, 0xa, &payload[0], &payload[payload.size()]
    );

    uint8_t expected[] = { 0x12, 0x34, 0, 0, 0xff, 0xff, 0x5, 0xa };
    ASSERT_THAT(std::vector<uint8_t>(&buffer[0], &buffer[8]), ElementsAreArray(expected));
    ASSERT_EQ(std::vector<uint8_t>(&buffer[8], bufferEnd), payload);
}

TEST_F(TCPTest, it_throws_if_the_resulting_frame_payload_cannot_be_represented_in_the_lenght_field) {
    uint8_t buffer[0];
    ASSERT_THROW(TCP::formatFrame(nullptr, 0xabcd, 0x2, 0x10, buffer, buffer + 65534),
                 std::invalid_argument);
}

TEST_F(TCPTest, it_parses_a_frame) {
    uint8_t bytes[] = { 0x12, 0x34, 0, 0, 0, 9, 0x34, 0xEB, 1, 2, 3, 4, 5, 6, 7 };
    Frame frame = TCP::parseFrame(0x1234, bytes, bytes + sizeof(bytes));
    ASSERT_EQ(0x34, frame.address);
    ASSERT_EQ(0xeb, frame.function);

    uint8_t payload[7] = { 1, 2, 3, 4, 5, 6, 7 };
    ASSERT_THAT(frame.payload, ElementsAreArray(payload));
}

TEST_F(TCPTest, it_rejects_a_frame_if_its_transaction_ID_does_not_match) {
    uint8_t bytes[] = { 0x12, 0x34, 0, 0, 0, 9, 0x34, 0xEB, 1, 2, 3, 4, 5, 6, 7 };
    ASSERT_THROW(TCP::parseFrame(0x0000, bytes, bytes + sizeof(bytes)),
                 TCP::TransactionIDMismatch);
}

TEST_F(TCPTest, it_throws_if_attempting_to_parse_a_buffer_that_does_not_contain_the_advertised_payload) {
    uint8_t bytes[8];
    bytes[0] = 0;
    bytes[1] = 0;
    bytes[2] = 0;
    bytes[3] = 0;
    bytes[4] = 0;
    bytes[5] = 5;
    ASSERT_THROW(TCP::parseFrame(0, bytes, bytes + 10), TCP::TooSmall);
}

TEST_F(TCPTest, it_formats_a_read_holding_registers_command) {
    constexpr int EXPECTED_SIZE = 12;
    uint8_t buffer[EXPECTED_SIZE];
    uint8_t* end = TCP::formatReadRegisters(buffer, 0xabcd, 0x10, false, 0x1020, 0x05);
    ASSERT_EQ(end - buffer, EXPECTED_SIZE);

    uint8_t expected[EXPECTED_SIZE] = { 0xab, 0xcd, 0, 0, 0, 6, 0x10, 3, 0x10, 0x20, 0x0, 0x05 };
    ASSERT_THAT(std::vector<uint8_t>(buffer, end), ElementsAreArray(expected));
}

TEST_F(TCPTest, it_formats_a_read_input_registers_command) {
    constexpr int EXPECTED_SIZE = 12;
    uint8_t buffer[EXPECTED_SIZE];
    uint8_t* end = TCP::formatReadRegisters(buffer, 0xabcd, 0x10, true, 0x1020, 0x05);
    ASSERT_EQ(end - buffer, EXPECTED_SIZE);

    uint8_t expected[EXPECTED_SIZE] = { 0xab, 0xcd, 0, 0, 0, 6, 0x10, 4, 0x10, 0x20, 0x0, 0x05 };
    ASSERT_THAT(std::vector<uint8_t>(buffer, end), ElementsAreArray(expected));
}

TEST_F(TCPTest, it_throws_if_attempting_to_read_beyond_register_65536) {
    ASSERT_THROW(TCP::formatReadRegisters(nullptr, 0xabcd, 0x10, false, 0xfffe, 2),
                 std::invalid_argument);
}

TEST_F(TCPTest, it_throws_if_attempting_to_read_more_than_128_registers) {
    ASSERT_THROW(TCP::formatReadRegisters(nullptr, 0xabcd, 0x10, false, 0, 129),
                 std::invalid_argument);
}

TEST_F(TCPTest, it_parses_a_read_register_reply) {
    Frame frame = { 0x10, 0x03, { 0x04, 0x1, 0x2, 0x3, 0x4 } };
    uint16_t values[2];
    TCP::parseReadRegisters(values, frame, 2);

    uint16_t expected[] = { 0x0102, 0x0304 };
    ASSERT_THAT(vector<uint16_t>(values, values + 2), ElementsAreArray(expected));
}

TEST_F(TCPTest, it_throws_if_the_amount_of_registers_in_the_reply_is_smaller_than_the_expected) {
    Frame frame = { 0x10, 0x03, { 2, 1, 2 } };
    ASSERT_THROW(TCP::parseReadRegisters(nullptr, frame, 2), UnexpectedReply);
}

TEST_F(TCPTest, it_throws_if_the_amount_of_registers_in_the_reply_is_greater_than_the_expected) {
    Frame frame = { 0x10, 0x03, { 6, 1, 2, 3, 4, 5, 6 } };
    ASSERT_THROW(TCP::parseReadRegisters(nullptr, frame, 2), UnexpectedReply);
}

TEST_F(TCPTest, it_throws_if_the_byte_count_is_odd) {
    Frame frame = { 0x10, 0x03, { 5, 1, 2, 3, 4, 5 } };
    ASSERT_THROW(TCP::parseReadRegisters(nullptr, frame, 2), UnexpectedReply);
}

TEST_F(TCPTest, it_throws_if_the_amount_of_registers_in_the_reply_would_take_more_space_than_the_frame_payload) {
    Frame frame = { 0x10, 0x03, { 0x06, 0x1, 0x2, 0x3, 0x4, 0x5 } };
    ASSERT_THROW(TCP::parseReadRegisters(nullptr, frame, 2),
                 UnexpectedReply);
}

TEST_F(TCPTest, it_throws_if_the_amount_of_registers_in_the_reply_would_take_fewer_space_than_the_frame_payload) {
    Frame frame = { 0x10, 0x03, { 0x06, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7 } };
    ASSERT_THROW(TCP::parseReadRegisters(nullptr, frame, 2),
                 UnexpectedReply);
}

TEST_F(TCPTest, it_formats_a_single_register_write) {
    constexpr int EXPECTED_SIZE = 12;
    uint8_t buffer[EXPECTED_SIZE];
    uint8_t* end = TCP::formatWriteRegister(buffer, 0xabcd, 0x10, 0x1020, 0x1121);
    ASSERT_EQ(end - buffer, EXPECTED_SIZE);

    uint8_t expected[EXPECTED_SIZE] = { 0xab, 0xcd, 0, 0, 0, 6, 0x10, 6, 0x10, 0x20, 0x11, 0x21 };
    ASSERT_THAT(std::vector<uint8_t>(buffer, end), ElementsAreArray(expected));
}
