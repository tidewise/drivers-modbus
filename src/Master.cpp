#include <modbus/Master.hpp>
#include <modbus/RTU.hpp>
#include <modbus/Exceptions.hpp>

using namespace std;
using namespace base;
using namespace modbus;

int Master::extractPacket(uint8_t const* buffer, size_t bufferSize) const {
    throw std::logic_error("modbus::Master should be read only using readRaw");
}

Master::Master()
    : iodrivers_base::Driver(RTU::FRAME_MAX_SIZE * 10) {
    m_read_buffer.resize(MAX_PACKET_SIZE);
    m_write_buffer.resize(MAX_PACKET_SIZE);
    m_frame.payload.reserve(RTU::FRAME_MAX_SIZE);
}

void Master::setInterframeDelay(base::Time const& delay) {
    m_interframe_delay = delay;
}

base::Time Master::getInterframeDelay() const {
    return m_interframe_delay;
}

Frame Master::readFrame() {
    Frame result;
    readFrame(result);
    return result;
}

void Master::readFrame(Frame& frame) {
    int c = readRaw(&m_read_buffer[0], m_read_buffer.size(),
                    getReadTimeout(), getReadTimeout(), m_interframe_delay);

    RTU::parseFrame(frame, &m_read_buffer[0], &m_read_buffer[c]);
}

Frame const& Master::request(int address, int function, vector<uint8_t> const& payload) {
    uint8_t* start = &m_write_buffer[0];
    uint8_t const* end = RTU::formatFrame(start, address, function, payload);
    writePacket(&m_write_buffer[0], end - start);

    readReply(m_frame, function);
    return m_frame;
}

void Master::broadcast(int function, vector<uint8_t> const& payload) {
    uint8_t* start = &m_write_buffer[0];
    uint8_t const* end = RTU::formatFrame(start, RTU::BROADCAST, function, payload);
    writePacket(&m_write_buffer[0], end - start);
}

Frame Master::readReply(int function) {
    Frame frame;
    readReply(frame, function);
    return frame;
}

void Master::readReply(Frame& frame, int function) {
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

vector<uint16_t> Master::readRegisters(int address, bool input_registers,
                                       int start, int length) {
    vector<uint16_t> registers;
    registers.resize(length);
    readRegisters(&registers[0], address, input_registers, start, length);
    return registers;
}

void Master::readRegisters(uint16_t* values, int address,
                           bool input_registers, int start, int length) {
    uint8_t* buffer_start = &m_write_buffer[0];
    uint8_t const* buffer_end = RTU::formatReadRegisters(
        buffer_start, address, input_registers, start, length
    );
    writePacket(buffer_start, buffer_end - buffer_start);
    readReply(m_frame, input_registers ? RTU::FUNCTION_READ_INPUT_REGISTERS :
                                         RTU::FUNCTION_READ_HOLDING_REGISTERS);

    RTU::parseReadRegisters(values, m_frame, length);
}

void Master::writeRegister(int address, uint16_t register_id, uint16_t value) {
    uint8_t* buffer_start = &m_write_buffer[0];
    uint8_t const* buffer_end = RTU::formatWriteRegister(
        buffer_start, address, register_id, value
    );
    writePacket(buffer_start, buffer_end - buffer_start);
    readReply(m_frame, RTU::FUNCTION_WRITE_SINGLE_REGISTER);
}