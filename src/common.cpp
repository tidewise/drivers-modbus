#include <modbus/Exceptions.hpp>
#include <modbus/common.hpp>

using namespace modbus;
using namespace std;

uint8_t* common::format16(uint8_t* buffer, uint16_t value)
{
    buffer[0] = (value >> 8) & 0xFF;
    buffer[1] = (value >> 0) & 0xFF;
    return buffer + 2;
}

uint8_t const* common::parse16(uint8_t const* buffer, uint16_t& value)
{
    value =
        (static_cast<uint16_t>(buffer[0]) << 8) | (static_cast<uint16_t>(buffer[1]) << 0);
    return buffer + 2;
}

void common::parseReadRegisters(uint16_t* values, Frame const& frame, int length)
{
    // This is the packet length in bytes corresponding to the data
    // (2 bytes: High and Low): data_length (in bytes) + 1 (for `byte_count`)
    uint8_t data_length = (length * 2);
    validateByteCount(frame, data_length);
    uint8_t packet_length = data_length + 1;
    validateReply(frame, packet_length);

    for (int i = 0; i < length; ++i) {
        parse16(&frame.payload[1 + i * 2], values[i]);
    }
}

void common::parseReadDigitalInputs(std::vector<bool>& values,
    Frame const& frame,
    int length)
{
    // This is the length in bytes from the conversion bits to bytes 1 bit for each input:
    // data_length (in bytes) + 1 (for `byte_count`)
    uint8_t data_length = (length + 7) / 8;
    validateByteCount(frame, data_length);
    uint8_t packet_length = data_length + 1;
    validateReply(frame, packet_length);

    int i = 1;
    int shift = 0;
    for (; length > 0; --length, ++shift) {
        if (shift == 8) {
            i++;
            shift = 0;
        }
        values.push_back((frame.payload[i] >> shift) & 0x1);
    }
}

void common::validateReply(Frame const& frame, uint8_t expected_size)
{
    if (frame.payload.size() != expected_size) {
        throw UnexpectedReply("reply does not contain as many "
                              "bytes as was expected. payload size != expected size: " +
                              to_string(frame.payload.size()) +
                              " != " + to_string(expected_size));
    }
}

void common::validateByteCount(Frame const& frame, uint8_t expected_size)
{
    uint8_t byte_count = frame.payload[0];
    if (expected_size != byte_count) {
        throw UnexpectedReply(
            "reply byte count does not contain as many bytes as was expected");
    }
}