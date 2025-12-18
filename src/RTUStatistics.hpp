#ifndef MODBUS_RTUSTATISTICS_HPP
#define MODBUS_RTUSTATISTICS_HPP

#include <base/Time.hpp>
#include <stdexcept>
#include <string>

namespace modbus {
    struct RTUStatistics {
        /** Time of the last update
         *
         * @meta role logical_time
         */
        base::Time time;

        // Total InvalidCRC error count
        uint32_t total_crc_error_count = 0;

        // Total UnexpectedReply error count
        uint32_t total_unexpected_reply_error_count = 0;

        // Total Success count
        uint64_t total_success_count = 0;

        /** Error balance
         *
         * Each success decreases 1 and each failure increases by the defined
         * increment per error */
        uint8_t error_count = 0;
    };
}

#endif