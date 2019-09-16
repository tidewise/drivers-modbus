#ifndef MODBUS_FRAME_HPP
#define MODBUS_FRAME_HPP

#include <vector>
#include <cstdint>

namespace modbus {
    /**
     * A modbus frame
     */
    struct Frame {
        uint8_t address;
        uint8_t function;
        std::vector<uint8_t> payload;
    };
}

#endif