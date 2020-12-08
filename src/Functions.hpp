#ifndef MODBUS_FUNCTIONS_HPP
#define MODBUS_FUNCTIONS_HPP

namespace modbus {
    enum Functions {
        FUNCTION_READ_HOLDING_REGISTERS = 0x03,
        FUNCTION_READ_INPUT_REGISTERS = 0x04,
        FUNCTION_WRITE_SINGLE_REGISTER = 0x06
    };
}

#endif