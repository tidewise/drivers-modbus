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

        /** Total amount of replies expected written values are incoerent,
         *  rising values could indicate degraded bus communication.
         *
         *  The CRC are 2 bytes added at the end of each message
         *  and are calculated using a polynomial expression using
         *  all the bytes on the message. This represents how many
         *  times the expected value did not match the actual value.
         */
        uint32_t total_crc_error_count = 0;

        /** Total amount of replies where written information does not match expected,
         *  rising values may indicate transmission or timing problems.
         *
         *  Every Modbus RTU request has a response that should match
         *  a given pattern which is better described here:
         *  https://www.modbustools.com/modbus.html
         *  Here are stored all the times the length of the data in response
         *  payload did not match the expected size or function code.
         */
        uint32_t total_unexpected_reply_error_count = 0;

        /** Total amount of replies where the frame received was too small */
        uint32_t total_too_small_error_count = 0;

        /** Total amount of messages that passed validation */
        uint64_t total_success_count = 0;

        /** Error balance
         *
         * Each success decreases 1 and each failure increases by the defined
         * increment per error */
        uint8_t error_count = 0;
    };
}

#endif