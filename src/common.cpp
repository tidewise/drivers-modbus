#include <modbus/Exceptions.hpp>
#include <modbus/common.hpp>
#include <cmath>


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
    uint8_t byte_count = frame.payload[0];
    validateReply(frame, (length * 2) + 1);
    if (frame.payload.size() != byte_count + 1u) {
        throw UnexpectedReply("reply's advertised byte count and frame payload "
                              "size differ (" +
                              to_string(byte_count + 1u) +
                              " != " + to_string(frame.payload.size()) + ")");
    }

    for (int i = 0; i < length; ++i) {
        parse16(&frame.payload[1 + i * 2], values[i]);
    }
}

void common::parseReadDigitalInputs(std::vector<bool>& values,
    Frame const& frame,
    int length)
{
    // This is the ammount of bytes needed to send values + 1 for `byte_count`
    uint8_t byte_length = std::ceil(length / 8.0) + 1;
    validateReply(frame, byte_length);
    uint16_t byte_count = frame.payload[0];
    if (frame.payload.size() != byte_count + 1u) {
        throw UnexpectedReply("RTU::praseReadDigitalInputs: reply's advertised byte "
                              "count and frame payload "
                              "size differ (" +
                              to_string(byte_count + 1u) +
                              " != " + to_string(frame.payload.size()) + ")");
    }

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
    if (frame.payload.empty()) {
        throw UnexpectedReply("empty reply");
    }

    if (frame.payload.size() != expected_size) {
        throw UnexpectedReply("reply does not contain as many "
                              "bytes as was expected. payload size != expected size: " +
                              to_string(frame.payload.size()) +
                              " != " + to_string(expected_size));
    }
}