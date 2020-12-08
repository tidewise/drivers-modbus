#include <modbus/common.hpp>
#include <modbus/Exceptions.hpp>

using namespace modbus;
using namespace std;

uint8_t* common::format16(uint8_t* buffer, uint16_t value) {
    buffer[0] = (value >> 8) & 0xFF;
    buffer[1] = (value >> 0) & 0xFF;
    return buffer + 2;
}

uint8_t const* common::parse16(uint8_t const* buffer, uint16_t& value) {
    value = (static_cast<uint16_t>(buffer[0]) << 8) |
            (static_cast<uint16_t>(buffer[1]) << 0);
    return buffer + 2;
}

void common::parseReadRegisters(uint16_t* values, Frame const& frame, int length) {
    uint8_t byte_count = frame.payload[0];
    if (frame.payload.size() != byte_count + 1u) {
        throw UnexpectedReply(
            "RTU::parseReadRegisters: reply's advertised byte count and frame payload "
            "size differ ("
            + to_string(byte_count + 1u) + " != "
            + to_string(frame.payload.size()) + ")"
        );
    }
    else if (byte_count != length * 2) {
        throw UnexpectedReply("RTU::parseReadRegisters: reply does not contain as many "
                              "registers as was expected");
    }

    for (int i = 0; i < length; ++i) {
        parse16(&frame.payload[1 + i * 2], values[i]);
    }
}

void common::parseReadDigitalInputs(
    std::vector<bool>& values, Frame const& frame, int length
) {
    if (frame.payload.empty()) {
        throw UnexpectedReply("RTU::parseReadDigitalInputs: empty reply");
    }
    uint16_t byte_count = frame.payload[0];
    if (frame.payload.size() != byte_count + 1u) {
        throw UnexpectedReply(
            "RTU::praseReadDigitalInputs: reply's advertised byte count and frame payload "
            "size differ ("
            + to_string(byte_count + 1u) + " != "
            + to_string(frame.payload.size()) + ")"
        );
    }
    if (byte_count * 8 < length) {
        throw UnexpectedReply("RTU::parseReadDigitalInputs: reply does not contain as many "
                              "coils/digital inputs as expected");
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
