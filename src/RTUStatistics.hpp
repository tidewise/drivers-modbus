#ifndef MODBUS_RTUSTATISTICS_HPP
#define MODBUS_RTUSTATISTICS_HPP

#include <base/Time.hpp>
#include <stdexcept>
#include <string>

namespace modbus {
    struct RTUStatistics {
        /** CRC Error count*/
        double total_CRC_error_count = 0;

        /** Unexpected Reply */
        double total_reply_error_count = 0;

        /** Unexpected Reply Error count */
        double total_sucess_count = 0;

        /** Error balance */
        uint8_t error_count = 0;

        /** Time of the last update*/
        base::Time timestamp;
    };
}

#endif