#include <modbus/RTU.hpp>
#include <cstring>
#include <cmath>
#include <modbus/Exceptions.hpp>

using namespace std;
using namespace base;
using namespace modbus;

base::Time RTU::interframeDuration(int bitrate) {
    int duration_us = ceil(1000000.0 / static_cast<double>(bitrate) *
                           SERIAL_BITS_PER_CHAR * 3.5);
    return base::Time::fromMicroseconds(max(duration_us, 1750));
}

uint8_t* RTU::formatFrame(uint8_t* buffer, int address, int functionID,
                     std::vector<uint8_t> const& payload) {

    return RTU::formatFrame(buffer, address, functionID,
                            &(*payload.begin()), &(*payload.end()));
}
uint8_t* RTU::formatFrame(uint8_t* buffer, int address, int functionID,
                          uint8_t const* payloadStart, uint8_t const* payloadEnd) {

    int payloadSize = payloadEnd - payloadStart;
    if (payloadSize + FRAME_OVERHEAD_SIZE > FRAME_MAX_SIZE) {
        throw std::invalid_argument("RTU::formatFrame: frame size would be bigger "\
                                    "than maximum allowed of 256");
    }

    buffer[0] = address;
    buffer[1] = functionID;
    auto bufferPayload = buffer + FRAME_HEADER_SIZE;
    memcpy(bufferPayload, payloadStart, payloadSize);
    auto bufferCRC = crc(buffer, bufferPayload + payloadSize);
    *(bufferPayload + payloadSize) = bufferCRC[0];
    *(bufferPayload + payloadSize + 1) = bufferCRC[1];
    return bufferPayload + payloadSize + 2;
}

static void validateBufferSize(uint8_t const* start, uint8_t const* end,
                               char const* context) {
    if (end - start < RTU::FRAME_OVERHEAD_SIZE) {
        throw RTU::TooSmall(
            string(context) + ": "
            "expected at least " + to_string(RTU::FRAME_OVERHEAD_SIZE) + " bytes, "
            "but got " + to_string(end - start)
        );
    }
}

Frame RTU::parseFrame(uint8_t const* start, uint8_t const* end) {
    Frame result;
    parseFrame(result, start, end);
    return result;
}

void RTU::parseFrame(Frame& frame, uint8_t const* start, uint8_t const* end) {
    validateBufferSize(start, end, "RTU::parseFrame");
    if (!isCRCValid(start, end)) {
        throw InvalidCRC("RTU::parseFrame: CRC check failed");
    }

    frame.address  = start[0];
    frame.function = start[1];
    frame.payload.resize((end - start) - FRAME_OVERHEAD_SIZE);
    std::copy(start + FRAME_HEADER_SIZE, end - 2, frame.payload.begin());
}

array<uint8_t, 2> RTU::crc(uint8_t const* start, uint8_t const* end) {
    uint16_t crc = 0xFFFF;
    for (uint8_t const* it = start; it != end; ++it) {
        crc ^= (uint16_t)*it;

        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else {
                crc >>= 1;
            }
        }
    }

    array<uint8_t, 2> result;
    result[0] = crc & 0xFF;
    result[1] = (crc >> 8) & 0xFF;
    return result;
}

bool RTU::isCRCValid(uint8_t const* start, uint8_t const* end) {
    validateBufferSize(start, end, "RTU::isCRCValid");
    auto expected = crc(start, end - 2);
    return (end[-2] == expected[0]) && (end[-1] == expected[1]);
}

static uint8_t* format16(uint8_t* buffer, uint16_t value) {
    buffer[0] = (value >> 8) & 0xFF;
    buffer[1] = (value >> 0) & 0xFF;
    return buffer + 2;
}

static uint8_t const* parse16(uint8_t const* buffer, uint16_t& value) {
    value = (static_cast<uint16_t>(buffer[0]) << 8) |
            (static_cast<uint16_t>(buffer[1]) << 0);
    return buffer + 2;
}

uint8_t* RTU::formatReadRegisters(
    uint8_t* buffer, uint8_t address,
    bool input_registers, uint16_t start, uint8_t length) {
    if (length > 128) {
        throw std::invalid_argument(
            "RTU::formatReadRegisters: too many registers requested"
        );
    }
    else if (65535 - start < length) {
        throw std::invalid_argument(
            "RTU::formatReadRegisters: attempting to read beyond register 65536"
        );
    }

    uint8_t payload[4];
    format16(payload, start);
    format16(payload + 2, length);

    Functions function = input_registers ? FUNCTION_READ_INPUT_REGISTERS :
                                           FUNCTION_READ_HOLDING_REGISTERS;
    return formatFrame(buffer, address, function, payload, payload + 4);
}

void RTU::parseReadRegisters(uint16_t* values, Frame const& frame, int length) {
    uint8_t byte_count = frame.payload[0];
    if (frame.payload.size() != byte_count + 1u) {
        throw UnexpectedReply("RTU::parseReadRegisters: reply's advertised byte count "
                              "and frame payload size differ");
    }
    else if (byte_count != length * 2) {
        throw UnexpectedReply("RTU::parseReadRegisters: reply does not contain as many "
                              "registers as was expected");
    }

    for (int i = 0; i < length; ++i) {
        parse16(&frame.payload[1 + i * 2], values[i]);
    }
}
