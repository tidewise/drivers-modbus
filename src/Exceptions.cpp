#include <modbus/Exceptions.hpp>

using namespace std;
using namespace modbus;

RequestException::RequestException(int function_code, int exception_code,
                                   string const& message)
    : runtime_error(message)
    , function_code(function_code)
    , exception_code(exception_code) {
}

UnexpectedReply::UnexpectedReply(std::string const& message)
    : std::runtime_error(message) {
}