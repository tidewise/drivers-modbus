#ifndef MODBUS_EXCEPTIONS_HPP
#define MODBUS_EXCEPTIONS_HPP

#include <string>
#include <stdexcept>

namespace modbus {
    /** Exception thrown when a slave replies to a request with an exception
     */
    struct RequestException : public std::runtime_error {
        int function_code;
        int exception_code;

        RequestException(
            int function_code, int exception_code,
            std::string const& message = "received exception in reply to a request"
        );
    };

    /** Exception thrown when a reply does not have the expected function code
     */
    struct UnexpectedReply : public std::runtime_error {
        UnexpectedReply(std::string const& message = "received an unexpected reply");
    };
}

#endif