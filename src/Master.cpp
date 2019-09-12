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

    readFrame(m_frame);

    if (m_frame.function == function) {
        return m_frame;
    }
    else if (m_frame.function == FUNCTION_CODE_EXCEPTION + function) {
        int exception_code = 0;
        if (m_frame.payload.size()) {
            exception_code = m_frame.payload[0];
        }
        throw RequestException(function, exception_code, "request failed");
    }
    else {
        throw UnexpectedReply();
    }
}

void Master::broadcast(int function, vector<uint8_t> const& payload) {
    uint8_t* start = &m_write_buffer[0];
    uint8_t const* end = RTU::formatFrame(start, RTU::BROADCAST, function, payload);
    writePacket(&m_write_buffer[0], end - start);
}
