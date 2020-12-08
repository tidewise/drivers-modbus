#ifndef MODBUS_TCP_HPP
#define MODBUS_TCP_HPP

#include <cstdint>
#include <stdexcept>
#include <vector>

#include <modbus/Frame.hpp>
#include <modbus/Functions.hpp>

namespace modbus {
    namespace TCP {
        /** TCP maximum total frame size
         */
        static const int MAX_PAYLOAD_SIZE = 65535l - 2;

        /** TCP minimum frame size
         */
        static const int FRAME_OVERHEAD_SIZE = 8;

        /** Return the frame length encoded in the (frame-aligned) given buffer */
        uint16_t frameLength(uint8_t const* buffer);

        /** @overload
         */
        uint8_t* formatFrame(uint8_t* buffer,
                             uint16_t transactionID, int address, int functionID,
                             std::vector<uint8_t> const& payload);

        /** Fill a byte buffer with a valid RTU frame (incl. CRC)
         */
        uint8_t* formatFrame(uint8_t* buffer,
                             uint16_t transactionID, int address, int functionID,
                             uint8_t const* payloadStart, uint8_t const* payloadEnd);

        /**
         * Exception thrown when the transaction ID received in a frame does not
         * match the expected one
         */
        struct TransactionIDMismatch : public std::runtime_error {
            using std::runtime_error::runtime_error;
        };

        /**
         * Exception thrown when the given strings of bytes are too small to
         * contain a modbus TCP frame
         */
        struct TooSmall : public std::runtime_error {
            using std::runtime_error::runtime_error;
        };

        /** Validates a set of bytes and converts it into a Frame
         *
         * @throw InvalidCRC if the buffer does not contain a valid frame
         */
        Frame parseFrame(uint16_t transactionID,
                         uint8_t const* start, uint8_t const* end);

        /** @overload parseFrame version that allows the use of a preallocated
         *      payload
         */
        void parseFrame(Frame& frame,
                        uint16_t transactionID, uint8_t const* start, uint8_t const* end);

        /** Fill a byte buffer with a request to read registers
         *
         * @arg whether input registers or holding registers should be read
         * @arg the start register
         * @arg length the number of registers to read
         */
        uint8_t* formatReadRegisters(
            uint8_t* buffer,
            uint16_t transactionID, uint8_t address,
            bool input_registers, uint16_t start, uint8_t length
        );

        /** Parse a read registers reply */
        void parseReadRegisters(
            uint16_t* values, Frame const& frame, int length
        );

        /** Fill a byte buffer with a request to write a single register
         *
         * @arg the register
         * @arg the register value
         */
        uint8_t* formatWriteRegister(
            uint8_t* buffer, uint16_t transactionID, uint8_t address,
            uint16_t register_id, uint16_t value
        );
    }
}

#endif