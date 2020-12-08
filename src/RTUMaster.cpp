#include <modbus/RTUMaster.hpp>

#include <modbus/common.hpp>
#include <modbus/Exceptions.hpp>
#include <modbus/RTU.hpp>

using namespace std;
using namespace base;
using namespace modbus;

int RTUMaster::extractPacket(uint8_t const* buffer, size_t bufferSize) const {
    throw std::logic_error("modbus::RTUMaster should be read only using readRaw");
}

RTUMaster::RTUMaster()
    : iodrivers_base::Driver(RTU::FRAME_MAX_SIZE * 10) {
    setReadTimeout(base::Time::fromSeconds(1));
    m_read_buffer.resize(MAX_PACKET_SIZE);
    m_write_buffer.resize(MAX_PACKET_SIZE);
    m_frame.payload.reserve(RTU::FRAME_MAX_SIZE);
}

void RTUMaster::setInterframeDelay(base::Time const& delay) {
    m_interframe_delay = delay;
}

base::Time RTUMaster::getInterframeDelay() const {
    return m_interframe_delay;
}

Frame RTUMaster::readFrame() {
    Frame result;
    readFrame(result);
    return result;
}

void RTUMaster::readFrame(Frame& frame) {
    int c = readRaw(&m_read_buffer[0], m_read_buffer.size(),
                    getReadTimeout(), getReadTimeout(), m_interframe_delay);

    RTU::parseFrame(frame, &m_read_buffer[0], &m_read_buffer[c]);
}

Frame const& RTUMaster::request(int address, int function, vector<uint8_t> const& payload) {
    uint8_t* start = &m_write_buffer[0];
    uint8_t const* end = RTU::formatFrame(start, address, function, payload);
    writePacket(&m_write_buffer[0], end - start);

    readReply(m_frame, function);
    return m_frame;
}

void RTUMaster::broadcast(int function, vector<uint8_t> const& payload) {
    uint8_t* start = &m_write_buffer[0];
    uint8_t const* end = RTU::formatFrame(start, RTU::BROADCAST, function, payload);
    writePacket(&m_write_buffer[0], end - start);
}

Frame RTUMaster::readReply(int function) {
    Frame frame;
    readReply(frame, function);
    return frame;
}

void RTUMaster::readReply(Frame& frame, int function) {
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

vector<uint16_t> RTUMaster::readRegisters(int address, bool input_registers,
                                       int start, int length) {
    vector<uint16_t> registers;
    registers.resize(length);
    readRegisters(&registers[0], address, input_registers, start, length);
    return registers;
}

void RTUMaster::readRegisters(uint16_t* values, int address,
                           bool input_registers, int start, int length) {
    uint8_t* buffer_start = &m_write_buffer[0];
    uint8_t const* buffer_end = RTU::formatReadRegisters(
        buffer_start, address, input_registers, start, length
    );
    writePacket(buffer_start, buffer_end - buffer_start);
    readReply(m_frame, input_registers ? FUNCTION_READ_INPUT_REGISTERS :
                                         FUNCTION_READ_HOLDING_REGISTERS);

    common::parseReadRegisters(values, m_frame, length);
}

uint16_t RTUMaster::readSingleRegister(int address, bool input_registers,
                                    int register_id) {
    uint16_t value;
    readRegisters(&value, address, input_registers, register_id, 1);
    return value;
}

void RTUMaster::writeSingleRegister(int address, uint16_t register_id, uint16_t value) {
    uint8_t* buffer_start = &m_write_buffer[0];
    uint8_t const* buffer_end = RTU::formatWriteRegister(
        buffer_start, address, register_id, value
    );
    writePacket(buffer_start, buffer_end - buffer_start);
    readReply(m_frame, FUNCTION_WRITE_SINGLE_REGISTER);
}

void RTUMaster::writeSingleCoil(int address, uint16_t register_id, bool value) {
    uint8_t* buffer_start = &m_write_buffer[0];
    uint8_t const* buffer_end = RTU::formatWriteSingleCoil(
        buffer_start, address, register_id, value
    );
    writePacket(buffer_start, buffer_end - buffer_start);
    readReply(m_frame, FUNCTION_WRITE_SINGLE_COIL);
}

std::vector<bool> RTUMaster::readDigitalInputs(int address, bool coils, uint16_t register_id, uint16_t count) {
    uint8_t* buffer_start = &m_write_buffer[0];
    uint8_t const* buffer_end = RTU::formatReadDigitalInputs(
        buffer_start, address, coils, register_id, count
    );
    writePacket(buffer_start, buffer_end - buffer_start);

    auto function = coils ? FUNCTION_READ_COILS : FUNCTION_READ_DIGITAL_INPUTS;
    readReply(m_frame, function);

    std::vector<bool> values;
    common::parseReadDigitalInputs(values, m_frame, count);
    return values;
}
