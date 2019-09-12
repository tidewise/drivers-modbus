#include <modbus/RTU.hpp>
#include <cstring>
#include <cmath>

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
