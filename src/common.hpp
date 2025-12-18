#ifndef MODBUS_COMMON_HPP
#define MODBUS_COMMON_HPP

#include <modbus/Frame.hpp>

namespace modbus {
    /** Parts common between the RTU and TCP protocols
     */
    namespace common {
        uint8_t* format16(uint8_t* buffer, uint16_t value);

        uint8_t const* parse16(uint8_t const* buffer, uint16_t& value);

        /** Parse a read registers reply */
        void parseReadRegisters(uint16_t* values, Frame const& frame, int length);

        /** Parse a coil/digital input reply */
        void parseReadDigitalInputs(std::vector<bool>& values,
            Frame const& frame,
            int length);

        /** Validates that the data payload size matches its expected size */
        void validateReply(Frame const& frame, uint8_t expected_size);

        /** Validates that the value of the byte count enclapsulated on the data payload
         *  matches the expected size */
        void validateByteCount(Frame const& frame, uint8_t expected_size);
    };
}

#endif