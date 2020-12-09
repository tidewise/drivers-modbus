#include <modbus/RTU.hpp>
#include <cstring>
#include <cmath>
#include <modbus/common.hpp>
#include <modbus/Exceptions.hpp>

using namespace std;
using namespace base;
using namespace modbus;
using namespace modbus::common;

base::Time RTU::interframeDuration(int bitrate) {
    int duration_us = ceil(1000000.0 / static_cast<double>(bitrate) *
                           SERIAL_BITS_PER_CHAR * 3.5);
    return base::Time::fromMicroseconds(max(duration_us, 1750));
}

uint8_t* RTU::formatFrame(uint8_t* buffer, int address, int functionID,
                     std::vector<uint8_t> const& payload) {

    auto begin = &payload[0];
    return RTU::formatFrame(buffer, address, functionID, begin, begin + payload.size());
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

uint8_t* RTU::formatReadDigitalInputs(
    uint8_t* buffer, uint8_t address, bool coils, uint16_t register_id, int count
) {
    uint8_t payload[4];
    format16(payload, register_id);
    format16(payload + 2, count);
    auto function = coils ? FUNCTION_READ_COILS : FUNCTION_READ_DIGITAL_INPUTS;
    return formatFrame(buffer, address, function, payload, payload + 4);
}

uint8_t* RTU::formatWriteRegister(uint8_t* buffer, uint8_t address,
                                  uint16_t register_id, uint16_t value) {
    uint8_t payload[4];
    format16(payload, register_id);
    format16(payload + 2, value);
    return formatFrame(buffer, address, FUNCTION_WRITE_SINGLE_REGISTER,
                       payload, payload + 4);
}

uint8_t* RTU::formatWriteSingleCoil(uint8_t* buffer, uint8_t address,
                                    uint16_t register_id, bool value) {
    uint8_t payload[4];
    format16(payload, register_id);
    payload[2] = value ? 0xff : 0;
    payload[3] = 0;
    return formatFrame(buffer, address, FUNCTION_WRITE_SINGLE_COIL,
                       payload, payload + 4);
}