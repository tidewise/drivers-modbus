#ifndef MODBUS_MASTER_HPP
#define MODBUS_MASTER_HPP

#include <iodrivers_base/Driver.hpp>
#include <modbus/Master.hpp>
#include <modbus/Frame.hpp>

namespace modbus {
    /**
     * Driver implementing a Modbus master
     *
     * Modbus uses timeouts to determine the end of a frame. Because of this,
     * iodrivers_base's read timeout will only be used for the first byte
     * timeout (time to receive a first byte). Once the first byte is received,
     * a modbus-specified interframe delay is used instead.
     *
     * When on serial lines, the interframe delay is specified by modbus as
     * begin 3.5 times the transmission delay of a character (remember that
     * there are 11 bits for a character on a serial line), with a minimum
     * boundary of 1.750 ms for bitrates above 19200 bauds. This driver defaults
     * to this value, you will have to set it explicitely for transmission lines
     * at lower speeds.
     *
     * Since modbus uses time to define where packets start and end, this is
     * not meant to be used as a "standard" iodrivers_base Driver. Internally,
     * it uses readRaw. Using the packet-based interface (readPacket) will
     * throw.
     */
    class Master : public iodrivers_base::Driver {
        /** Modbus packet extraction is time-based
         *
         * This method just throws
         */
        int extractPacket(uint8_t const* buffer, size_t bufferSize) const;

        /*
         * Default interframe is spec'd delay for bitrate > 19200 (1.750ms)
         * with 5ms margin
         */
        base::Time m_interframe_delay = base::Time::fromMilliseconds(7);

        /** Internal read buffer */
        std::vector<uint8_t> m_read_buffer;

        /** Internal write buffer */
        std::vector<uint8_t> m_write_buffer;

    public:
        Master();

        /** Change the expected interframe delay
         *
         * When on serial lines, the interframe delay is specified by modbus as
         * begin 3.5 times the transmission delay of a character (remember that
         * there are 11 bits for a character on a serial line), with a minimum
         * boundary of 1.750 ms for bitrates above 19200 bauds. This driver
         * defaults to this value, you will have to set it explicitely for
         * transmission lines at lower speeds.
         */
        void setInterframeDelay(base::Time const& delay);

        /** Get the expected interframe delay
         */
        base::Time getInterframeDelay() const;

        /** Wait for one frame on the bus and read it
         */
        Frame readFrame();

        /** Send a request and wait for the slave's reply */
        Frame request(int address, int function, std::vector<uint8_t> const& payload);

        /** Send a broadcast
         *
         * Does not wait for the turnaround delay
         */
        void broadcast(int function, std::vector<uint8_t> const& payload);
    };
}

#endif