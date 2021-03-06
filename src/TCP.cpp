#include <modbus/TCP.hpp>

#include <cstring>
#include <cmath>
#include <modbus/common.hpp>
#include <modbus/Exceptions.hpp>

using namespace std;
using namespace modbus;
using namespace modbus::common;

uint16_t TCP::frameLength(uint8_t const* buffer) {
    uint16_t lengthField;
    parse16(buffer + 4, lengthField);
    return static_cast<uint32_t>(lengthField) + FRAME_OVERHEAD_SIZE - 2;
}

uint8_t* TCP::formatFrame(uint8_t* buffer,
                          uint16_t transactionID, int address, int functionID,
                          std::vector<uint8_t> const& payload) {
    auto begin = &payload[0];
    return TCP::formatFrame(
        buffer, transactionID, address, functionID, begin, begin + payload.size()
    );
}
uint8_t* TCP::formatFrame(uint8_t* buffer,
                          uint16_t transactionID, int address, int functionID,
                          uint8_t const* payloadStart, uint8_t const* payloadEnd) {

    int payloadSize = payloadEnd - payloadStart;
    if (payloadSize > MAX_PAYLOAD_SIZE) {
        throw std::invalid_argument("TCP::formatFrame: payload bigger than allowed");
    }

    buffer[0] = transactionID >> 8;
    buffer[1] = transactionID & 0xff;
    buffer[2] = 0;
    buffer[3] = 0;
    uint16_t length = payloadSize + 2;
    buffer[4] = length >> 8;
    buffer[5] = length & 0xff;
    buffer[6] = address;
    buffer[7] = functionID;
    memcpy(buffer + 8, payloadStart, payloadSize);
    return buffer + 8 + payloadSize;
}

static uint16_t validateBufferSize(uint8_t const* start, uint8_t const* end,
                                   char const* context) {
    uint32_t size = end - start;
    if (end - start < TCP::FRAME_OVERHEAD_SIZE) {
        throw TCP::TooSmall(
            string(context) + ": "
            "expected at least " + to_string(TCP::FRAME_OVERHEAD_SIZE) + " bytes, "
            "but got " + to_string(size)
        );
    }

    uint16_t lengthField;
    parse16(start + 4, lengthField);

    if (size != lengthField + 6u) {
        throw TCP::TooSmall(
            string(context) + ": "
            "length field announces a whole packet of " + to_string(lengthField + 5) +
            " bytes but got " + to_string(size)
        );
    }

    return lengthField - 2;
}

Frame TCP::parseFrame(uint16_t transactionID, uint8_t const* start, uint8_t const* end) {
    Frame result;
    parseFrame(result, transactionID, start, end);
    return result;
}

void TCP::parseFrame(Frame& frame, uint16_t transactionID,
                     uint8_t const* start, uint8_t const* end) {
    uint16_t payloadLength = validateBufferSize(start, end, "TCP::parseFrame");

    uint16_t msbTransactionID = start[0];
    uint16_t lsbTransactionID = start[1];
    uint16_t receivedTransactionID = msbTransactionID << 8 | lsbTransactionID;
    if (receivedTransactionID != transactionID) {
        throw TransactionIDMismatch("received and expected transaction IDs mismatch");
    }

    frame.address  = start[6];
    frame.function = start[7];
    frame.payload.resize(payloadLength);
    std::copy(start + 8, end, frame.payload.begin());
}

uint8_t* TCP::formatReadRegisters(
    uint8_t* buffer,
    uint16_t transactionID, uint8_t address,
    bool input_registers, uint16_t start, uint8_t length) {
    if (length > 128) {
        throw std::invalid_argument(
            "TCP::formatReadRegisters: too many registers requested"
        );
    }
    else if (65535 - start < length) {
        throw std::invalid_argument(
            "TCP::formatReadRegisters: attempting to read beyond register 65536"
        );
    }

    uint8_t payload[4];
    format16(payload, start);
    format16(payload + 2, length);

    Functions function = input_registers ? FUNCTION_READ_INPUT_REGISTERS :
                                           FUNCTION_READ_HOLDING_REGISTERS;
    return formatFrame(buffer, transactionID, address, function, payload, payload + 4);
}

uint8_t* TCP::formatWriteRegister(uint8_t* buffer, uint16_t transactionID, uint8_t address,
                                  uint16_t register_id, uint16_t value) {
    uint8_t payload[4];
    format16(payload, register_id);
    format16(payload + 2, value);
    return formatFrame(buffer, transactionID, address, FUNCTION_WRITE_SINGLE_REGISTER,
                       payload, payload + 4);
}

uint8_t* TCP::formatReadDigitalInputs(
    uint8_t* buffer, uint16_t transactionID, uint8_t address,
    bool coils, uint16_t register_id, int count
) {
    uint8_t payload[4];
    format16(payload, register_id);
    format16(payload + 2, count);
    auto function = coils ? FUNCTION_READ_COILS : FUNCTION_READ_DIGITAL_INPUTS;
    return formatFrame(buffer, transactionID, address, function, payload, payload + 4);
}

uint8_t* TCP::formatWriteSingleCoil(uint8_t* buffer, uint16_t transactionID, uint8_t address,
                                    uint16_t register_id, bool value) {
    uint8_t payload[4];
    format16(payload, register_id);
    payload[2] = value ? 0xff : 0;
    payload[3] = 0;
    return formatFrame(buffer, transactionID, address, FUNCTION_WRITE_SINGLE_COIL,
                       payload, payload + 4);
}