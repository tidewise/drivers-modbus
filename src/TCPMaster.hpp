#ifndef MODBUS_TCP_MASTER_HPP
#define MODBUS_TCP_MASTER_HPP

#include <iodrivers_base/Driver.hpp>
#include <modbus/Frame.hpp>
#include <modbus/Exceptions.hpp>

namespace modbus {
    /**
     * Driver implementing a Modbus TCP master
     */
    class TCPMaster : public iodrivers_base::Driver {
        /** Modbus packet extraction is time-based
         *
         * This method just throws
         */
        int extractPacket(uint8_t const* buffer, size_t bufferSize) const;

        /** Transaction ID of the last sent request */
        uint16_t m_transaction_id = 0;

        /** Allocate a new transaction ID */
        uint16_t allocateTransactionID();

        /** Internal read buffer */
        std::vector<uint8_t> m_read_buffer;

        /** Internal write buffer */
        std::vector<uint8_t> m_write_buffer;

        /** Internal frame object
         *
         * This is used to avoid unnecessary memory allocation
         */
        Frame m_frame;

        static const int FUNCTION_CODE_EXCEPTION = 0x80;

    public:
        TCPMaster(uint16_t max_payload_size);

        /** Wait for one frame on the bus and read it
         */
        Frame readFrame();

        /** Wait for one frame on the bus and read it
         */
        void readFrame(Frame& frame);

        /** Wait for the reply for the given request
         */
        Frame readReply(int function);

        /** Wait for the reply for the given request
         */
        void readReply(Frame& frame, int function);

        /** Send a request and wait for the slave's reply */
        Frame const& request(
            int address, int function, std::vector<uint8_t> const& payload
        );

        /** Read a set of registers */
        std::vector<uint16_t> readRegisters(
            int address, bool input_registers, int start, int length
        );

        /** Read a set of registers */
        void readRegisters(
            uint16_t* values,
            int address, bool input_registers, int start, int length
        );

        uint16_t readSingleRegister(int address, bool input_registers, int register_id);

        void writeSingleRegister(int address, uint16_t register_id, uint16_t value);
    };
}

#endif
