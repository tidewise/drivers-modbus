#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <modbus/common.hpp>
#include <modbus/Exceptions.hpp>

using namespace modbus;
using namespace std;

using testing::ElementsAreArray;

struct CommonTest : public ::testing::Test {
};

TEST_F(CommonTest, it_parses_a_read_register_reply) {
    Frame frame = { 0x10, 0x03, { 0x04, 0x1, 0x2, 0x3, 0x4 } };
    uint16_t values[2];
    common::parseReadRegisters(values, frame, 2);

    uint16_t expected[] = { 0x0102, 0x0304 };
    ASSERT_THAT(vector<uint16_t>(values, values + 2), ElementsAreArray(expected));
}

TEST_F(CommonTest, it_throws_if_the_amount_of_registers_in_the_reply_is_smaller_than_the_expected) {
    Frame frame = { 0x10, 0x03, { 2, 1, 2 } };
    ASSERT_THROW(common::parseReadRegisters(nullptr, frame, 2), UnexpectedReply);
}

TEST_F(CommonTest, it_throws_if_the_amount_of_registers_in_the_reply_is_greater_than_the_expected) {
    Frame frame = { 0x10, 0x03, { 6, 1, 2, 3, 4, 5, 6 } };
    ASSERT_THROW(common::parseReadRegisters(nullptr, frame, 2), UnexpectedReply);
}

TEST_F(CommonTest, it_throws_if_the_byte_count_is_odd) {
    Frame frame = { 0x10, 0x03, { 5, 1, 2, 3, 4, 5 } };
    ASSERT_THROW(common::parseReadRegisters(nullptr, frame, 2), UnexpectedReply);
}

TEST_F(CommonTest, it_throws_if_the_amount_of_registers_in_the_reply_would_take_more_space_than_the_frame_payload) {
    Frame frame = { 0x10, 0x03, { 0x06, 0x1, 0x2, 0x3, 0x4, 0x5 } };
    ASSERT_THROW(common::parseReadRegisters(nullptr, frame, 2),
                 UnexpectedReply);
}

TEST_F(CommonTest, it_throws_if_the_amount_of_registers_in_the_reply_would_take_fewer_space_than_the_frame_payload) {
    Frame frame = { 0x10, 0x03, { 0x06, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7 } };
    ASSERT_THROW(common::parseReadRegisters(nullptr, frame, 2),
                 UnexpectedReply);
}

TEST_F(CommonTest, it_parses_a_read_digital_inputs_reply) {
    Frame frame = { 0x10, 0x03, { 0x2, 0xa2, 0x05 } };
    std::vector<bool> values;
    common::parseReadDigitalInputs(values, frame, 9);

    uint16_t expected[9] = { false, true, false, false, false, true, false, true, true };
    ASSERT_THAT(values, ElementsAreArray(expected));
}
