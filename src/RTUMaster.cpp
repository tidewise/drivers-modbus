#include <modbus/RTUMaster.hpp>

#include <cmath>
#include <modbus/Exceptions.hpp>
#include <modbus/RTU.hpp>
#include <modbus/common.hpp>

using namespace std;
using namespace base;
using namespace modbus;

int RTUMaster::extractPacket(uint8_t const* buffer, size_t bufferSize) const
{
    throw std::logic_error("modbus::RTUMaster should be read only using readRaw");
}

RTUMaster::RTUMaster(uint8_t error_threshold, uint8_t error_increment)
    : iodrivers_base::Driver(RTU::FRAME_MAX_SIZE * 10)
{
    setReadTimeout(base::Time::fromSeconds(1));
    m_read_buffer.resize(MAX_PACKET_SIZE);
    m_write_buffer.resize(MAX_PACKET_SIZE);
    m_frame.payload.reserve(RTU::FRAME_MAX_SIZE);

    m_error_increment = error_increment;
    m_error_threshold = error_threshold;
}

void RTUMaster::setInterframeDelay(base::Time const& delay)
{
    m_interframe_delay = delay;
}

base::Time RTUMaster::getInterframeDelay() const
{
    return m_interframe_delay;
}

void RTUMaster::setErrorThreshold(uint8_t const& threshold)
{
    m_error_threshold = threshold;
}

void RTUMaster::setErrorIncrement(uint8_t const& increment)
{
    m_error_increment = increment;
}

RTUStatistics RTUMaster::getRTUStats() const
{
    return m_statistics;
}

Frame RTUMaster::readFrame()
{
    Frame result;
    readFrame(result);
    return result;
}

void RTUMaster::readFrame(Frame& frame)
{
    int c = readRaw(&m_read_buffer[0],
        m_read_buffer.size(),
        getReadTimeout(),
        getReadTimeout(),
        m_interframe_delay);

    try {
        RTU::parseFrame(frame, &m_read_buffer[0], &m_read_buffer[c]);
    }
    catch (...) {
        m_stats.bad_rx += c;
        throw;
    }
}

void RTUMaster::broadcast(int function, vector<uint8_t> const& payload)
{
    uint8_t* start = &m_write_buffer[0];
    uint8_t const* end = RTU::formatFrame(start, RTU::BROADCAST, function, payload);
    writePacket(&m_write_buffer[0], end - start);
}

Frame RTUMaster::readReply(int function)
{
    Frame frame;
    readReply(frame, function);
    return frame;
}

void RTUMaster::readReply(Frame& frame, int function)
{
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

vector<uint16_t> RTUMaster::readRegisters(int address,
    bool input_registers,
    int start,
    int length)
{
    vector<uint16_t> registers;
    registers.resize(length);
    readRegisters(&registers[0], address, input_registers, start, length);
    return registers;
}

void RTUMaster::writePacketAndReadReply(uint8_t const* buffer,
    int bufsize,
    Frame& frame,
    int function,
    uint8_t expected_length)
{
    do {
        try {
            writePacket(buffer, bufsize);
            readReply(m_frame, function);
            common::validateReply(m_frame, expected_length);
            decreaseErrorCount();
            return;
        }
        catch (modbus::RTU::InvalidCRC const&) {
            m_statistics.total_CRC_error_count++;
            if (increaseErrorCountAndValidate()) {
                throw;
            }
        }
        catch (UnexpectedReply const&) {
            m_statistics.total_unexpected_reply_error_count++;
            if (increaseErrorCountAndValidate()) {
                throw;
            }
        }
    } while (true);
}

bool RTUMaster::increaseErrorCountAndValidate()
{
    m_statistics.error_count += m_error_increment;
    m_statistics.timestamp = base::Time::now();

    return (m_statistics.error_count >= m_error_threshold);
}

void RTUMaster::decreaseErrorCount()
{
    m_statistics.total_sucess_count++;
    m_statistics.timestamp = base::Time::now();

    if (m_statistics.error_count > 0) {
        m_statistics.error_count--;
    }
}

void RTUMaster::readRegisters(uint16_t* values,
    int address,
    bool input_registers,
    int start,
    int length)
{
    uint8_t* buffer_start = &m_write_buffer[0];
    uint8_t const* buffer_end =
        RTU::formatReadRegisters(buffer_start, address, input_registers, start, length);

    // read registers have an extra byte equal to byte_count
    auto expected_length = (length * 2) + 1;
    writePacketAndReadReply(buffer_start,
        buffer_end - buffer_start,
        m_frame,
        input_registers ? FUNCTION_READ_INPUT_REGISTERS : FUNCTION_READ_HOLDING_REGISTERS,
        expected_length);

    common::parseReadRegisters(values, m_frame, length);
}

uint16_t RTUMaster::readSingleRegister(int address, bool input_registers, int register_id)
{
    uint16_t value;
    readRegisters(&value, address, input_registers, register_id, 1);
    return value;
}

void RTUMaster::writeSingleRegister(int address, uint16_t register_id, uint16_t value)
{
    uint8_t* buffer_start = &m_write_buffer[0];
    uint8_t const* buffer_end =
        RTU::formatWriteRegister(buffer_start, address, register_id, value);
    writePacketAndReadReply(buffer_start,
        buffer_end - buffer_start,
        m_frame,
        FUNCTION_WRITE_SINGLE_REGISTER,
        2);
}

void RTUMaster::writeSingleCoil(int address, uint16_t register_id, bool value)
{
    uint8_t* buffer_start = &m_write_buffer[0];
    uint8_t const* buffer_end =
        RTU::formatWriteSingleCoil(buffer_start, address, register_id, value);
    writePacketAndReadReply(buffer_start,
        buffer_end - buffer_start,
        m_frame,
        FUNCTION_WRITE_SINGLE_COIL,
        4);
}

std::vector<bool> RTUMaster::readDigitalInputs(int address,
    bool coils,
    uint16_t register_id,
    uint16_t count)
{
    uint8_t* buffer_start = &m_write_buffer[0];
    uint8_t const* buffer_end =
        RTU::formatReadDigitalInputs(buffer_start, address, coils, register_id, count);
    auto function = coils ? FUNCTION_READ_COILS : FUNCTION_READ_DIGITAL_INPUTS;

    uint8_t byte_length = (count + 7) / 8.0 + 1;
    writePacketAndReadReply(buffer_start,
        buffer_end - buffer_start,
        m_frame,
        function,
        byte_length);

    std::vector<bool> values;
    common::parseReadDigitalInputs(values, m_frame, count);
    return values;
}
