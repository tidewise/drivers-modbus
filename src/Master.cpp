#include <modbus/Master.hpp>
#include <modbus/RTU.hpp>

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
}

void Master::setInterframeDelay(base::Time const& delay) {
    m_interframe_delay = delay;
}

base::Time Master::getInterframeDelay() const {
    return m_interframe_delay;
}

Frame Master::readFrame() {
    int c = readRaw(&m_read_buffer[0], m_read_buffer.size(),
                    getReadTimeout(), getReadTimeout(), m_interframe_delay);

    return RTU::parseFrame(&m_read_buffer[0], &m_read_buffer[c]);
}

Frame Master::request(int address, int function, vector<uint8_t> const& payload) {
    uint8_t* start = &m_write_buffer[0];
    uint8_t const* end = RTU::formatFrame(start, address, function, payload);
    writePacket(&m_write_buffer[0], end - start);

    return readFrame();
}

void Master::broadcast(int function, vector<uint8_t> const& payload) {
    uint8_t* start = &m_write_buffer[0];
    uint8_t const* end = RTU::formatFrame(start, RTU::BROADCAST, function, payload);
    writePacket(&m_write_buffer[0], end - start);
}
