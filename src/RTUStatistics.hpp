#ifndef MODBUS_RTUSTATISTICS_HPP
#define MODBUS_RTUSTATISTICS_HPP

#include <base/Time.hpp>
#include <stdexcept>
#include <string>

namespace modbus {
    struct RTUStatistics {
        /** Total InvalidCRC error count */
        int total_CRC_error_count = 0;

        /** Total UnexpectedReply error count */
        int total_unexpected_reply_error_count = 0;

        /** Total Success count */
        int total_sucess_count = 0;

        /** Error balance
         *
         * Each sucess decreases 1 and each failure increases by the defined
         * increment per error */
        uint8_t error_count = 0;

        /** Time of the last update*/
        base::Time timestamp;
    };
}

#endif