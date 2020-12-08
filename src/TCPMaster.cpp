#include <modbus/TCPMaster.hpp>
#include <modbus/TCP.hpp>
#include <modbus/Exceptions.hpp>

using namespace std;
using namespace base;
using namespace modbus;

int TCPMaster::extractPacket(uint8_t const* buffer, size_t bufferSize) const {
    if (bufferSize < 4) {
        return 0;
    }
    if (buffer[0] != m_transaction_id >> 8) {
        return -1;
    }
    if (buffer[1] != (m_transaction_id & 0xff)) {
        return -1;
    }
    if (buffer[2] != 0) {
        return -1;
    }
    if (buffer[3] != 0) {
        return -1;
    }
    if (bufferSize < TCP::FRAME_OVERHEAD_SIZE) {
        return 0;
    }
    uint16_t length = TCP::frameLength(buffer);
    if (length < bufferSize) {
        return 0;
    }

    return length;
}

TCPMaster::TCPMaster(uint16_t max_payload_size)
    : iodrivers_base::Driver((TCP::FRAME_OVERHEAD_SIZE + max_payload_size) * 10) {
    setReadTimeout(base::Time::fromSeconds(1));
    m_read_buffer.resize(MAX_PACKET_SIZE);
    m_write_buffer.resize(MAX_PACKET_SIZE);
    m_frame.payload.reserve(max_payload_size);
}

uint16_t TCPMaster::allocateTransactionID() {
    uint8_t lsb = m_transaction_id;
    ++lsb;
    return 0xAA00 | lsb;
}

Frame TCPMaster::readFrame() {
    Frame result;
    readFrame(result);
    return result;
}

void TCPMaster::readFrame(Frame& frame) {
    int c = readPacket(&m_read_buffer[0], m_read_buffer.size());
    TCP::parseFrame(frame, m_transaction_id, &m_read_buffer[0], &m_read_buffer[c]);
}

Frame const& TCPMaster::request(int address, int function, vector<uint8_t> const& payload) {
    uint8_t* start = &m_write_buffer[0];
    m_transaction_id = allocateTransactionID();
    uint8_t const* end = TCP::formatFrame(
        start, m_transaction_id, address, function, payload
    );
    writePacket(&m_write_buffer[0], end - start);
    readReply(m_frame, function);
    return m_frame;
}

Frame TCPMaster::readReply(int function) {
    Frame frame;
    readReply(frame, function);
    return frame;
}

void TCPMaster::readReply(Frame& frame, int function) {
    readFrame(frame);
    if (frame.function == function) {
        return;
    }

    if (frame.function == FUNCTION_CODE_EXCEPTION + function) {
        int exception_code = 0;
        if (frame.payload.size()) {
            exception_code = frame.payload[0];
        }
        throw RequestException(function, exception_code, "request failed");
    }
    else {
        throw UnexpectedReply("received reply's function does not match request");
    }
}

vector<uint16_t> TCPMaster::readRegisters(
    int address, bool input_registers, int start, int length) {
    vector<uint16_t> registers;
    registers.resize(length);
    readRegisters(&registers[0], address, input_registers, start, length);
    return registers;
}

void TCPMaster::readRegisters(
    uint16_t* values, int address, bool input_registers, int start, int length) {
    uint8_t* buffer_start = &m_write_buffer[0];
    m_transaction_id = allocateTransactionID();
    uint8_t const* buffer_end = TCP::formatReadRegisters(
        buffer_start, m_transaction_id, address, input_registers, start, length
    );
    writePacket(buffer_start, buffer_end - buffer_start);
    readReply(m_frame, input_registers ? FUNCTION_READ_INPUT_REGISTERS :
                                         FUNCTION_READ_HOLDING_REGISTERS);

    TCP::parseReadRegisters(values, m_frame, length);
}

uint16_t TCPMaster::readSingleRegister(int address, bool input_registers, int register_id) {
    uint16_t value;
    readRegisters(&value, address, input_registers, register_id, 1);
    return value;
}

void TCPMaster::writeSingleRegister(int address, uint16_t register_id, uint16_t value) {
    uint8_t* buffer_start = &m_write_buffer[0];
    m_transaction_id = allocateTransactionID();
    uint8_t const* buffer_end = TCP::formatWriteRegister(
        buffer_start, m_transaction_id, address, register_id, value
    );
    writePacket(buffer_start, buffer_end - buffer_start);
    readReply(m_frame, FUNCTION_WRITE_SINGLE_REGISTER);
}
