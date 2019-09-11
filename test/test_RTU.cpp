#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <modbus/RTU.hpp>

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
