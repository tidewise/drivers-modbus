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
        void parseReadRegisters(
            uint16_t* values, Frame const& frame, int length
        );

        /** Parse a coil/digital input reply */
        void parseReadDigitalInputs(
            std::vector<bool>& values, Frame const& frame, int length
        );

    };
}

#endif