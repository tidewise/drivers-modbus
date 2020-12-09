#ifndef MODBUS_RTU_HPP
#define MODBUS_RTU_HPP

#include <array>

#include <base/Time.hpp>
#include <modbus/Frame.hpp>
#include <modbus/Functions.hpp>
#include <modbus/RTU.hpp>

namespace modbus {
    /**
     * Implementation of the Modbus RTU (binary) protocol
     */
    namespace RTU {
        /** Modbus broadcast address */
        static const int BROADCAST = 0;

        /** Size in bits of a char on the serial bus. This is needed to compute
         * inter frame duration
         *
         * Note that this size is guaranteed, as the spec requires two stop
         * bits if parity is disabled
         */
        static const int SERIAL_BITS_PER_CHAR = 11;

        /** RTU maximum total frame size
         */
        static const int FRAME_MAX_SIZE = 256;

        /** Number of bytes in a RTU frame on top of the frame payload itself
         */
        static const int FRAME_OVERHEAD_SIZE = 4;

        /** Number of bytes in a RTU frame header */
        static const int FRAME_HEADER_SIZE = 2;

        /** Computes the interframe duration specified by the Modbus-on-serial
         * specification
         */
        base::Time interframeDuration(int bitrate);

        /** @overload
         */
        uint8_t* formatFrame(uint8_t* buffer, int address, int functionID,
                             std::vector<uint8_t> const& payload);

        /** Fill a byte buffer with a valid RTU frame (incl. CRC)
         */
        uint8_t* formatFrame(uint8_t* buffer, int address, int functionID,
                             uint8_t const* payloadStart, uint8_t const* payloadEnd);

        /**
         * Exception thrown when the given strings of bytes are too small to
         * contain a modbus RTU frame
         */
        struct TooSmall : public std::runtime_error {
            using std::runtime_error::runtime_error;
        };

        /** Exception thrown by parseFrame if the given buffer does not contain
         * a valid frame
         */
        struct InvalidCRC : public std::runtime_error {
            using std::runtime_error::runtime_error;
        };

        /** Validates a set of bytes and converts it into a Frame
         *
         * @throw InvalidCRC if the buffer does not contain a valid frame
         */
        Frame parseFrame(uint8_t const* start, uint8_t const* end);

        /** @overload parseFrame version that allows the use of a preallocated
         *      payload
         */
        void parseFrame(Frame& frame, uint8_t const* start, uint8_t const* end);

        /** Computes the Modbus CRC of a string of bytes */
        std::array<uint8_t, 2> crc(uint8_t const* start, uint8_t const* end);

        /** Validates the CRC contained at the end of a string of bytes
         *
         * The CRC is expected to be formatted as specified by the Modbus RTU
         * spec, that is: one-before-last byte is LSB and last byte is MSB.
         */
        bool isCRCValid(uint8_t const* start, uint8_t const* end);

        /** Fill a byte buffer with a request to read registers
         *
         * @arg whether input registers or holding registers should be read
         * @arg the start register
         * @arg length the number of registers to read
         */
        uint8_t* formatReadRegisters(
            uint8_t* buffer,
            uint8_t address, bool input_registers, uint16_t start, uint8_t length
        );

        /** Fill a byte buffer with a request to read multiple coils or digital inputs */
        uint8_t* formatReadDigitalInputs(
            uint8_t* buffer, uint8_t address,
            bool coils, uint16_t register_id, int count
        );

        /** Fill a byte buffer with a request to write a single register
         *
         * @arg the register
         * @arg the register value
         */
        uint8_t* formatWriteRegister(
            uint8_t* buffer, uint8_t address, uint16_t register_id, uint16_t value
        );

        /** Fill a byte buffer with a request to write a coil
         *
         * @arg the register
         * @arg the coil value
         */
        uint8_t* formatWriteSingleCoil(
            uint8_t* buffer, uint8_t address, uint16_t register_id, bool value
        );
    }
}

#endif