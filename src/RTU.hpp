#ifndef MODBUS_RTU_HPP
#define MODBUS_RTU_HPP

#include <base/Time.hpp>
#include <array>

namespace modbus {
    /**
     * Implementation of the Modbus RTU (binary) protocol
     */
    namespace RTU {
        static const int BROADCAST = 0;
        // Size in bits of a char on the serial bus. This is needed to compute
        // inter frame duration
        //
        // Note that this size is guaranteed, as the spec requires two stop bits
        // if parity is disabled
        static const int BITS_PER_CHAR = 11;
        static const int FRAME_MAX_SIZE = 256;
        static const int FRAME_OVERHEAD_SIZE = 4;
        static const int FRAME_HEADER_SIZE = 2;

        base::Time interframeDuration(int bitrate);

        uint8_t* formatFrame(uint8_t* buffer, int address, int functionID,
                             uint8_t const* payloadStart, uint8_t const* payloadEnd);

        std::array<uint8_t, 2> crc(uint8_t const* start, uint8_t const* end);

        bool isCRCValid(uint8_t const* start, uint8_t const* end);
    }
}

#endif