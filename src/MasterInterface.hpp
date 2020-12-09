#ifndef MODBUS_MASTERINTERFACE_HPP
#define MODBUS_MASTERINTERFACE_HPP

#include <modbus/Frame.hpp>

namespace modbus {
    /** Common interface between the RTU and TCP implementations
     */
    class MasterInterface {
    public:
        virtual ~MasterInterface() {}

        /** Wait for one frame on the bus and read it
         */
        virtual Frame readFrame() = 0;

        /** Wait for one frame on the bus and read it
         */
        virtual void readFrame(Frame& frame) = 0;

        /** Wait for the reply for the given request
         */
        virtual Frame readReply(int function) = 0;

        /** Wait for the reply for the given request
         */
        virtual void readReply(Frame& frame, int function) = 0;

        /** Send a request and wait for the slave's reply */
        virtual Frame const& request(
            int address, int function, std::vector<uint8_t> const& payload
        ) = 0;

        /** Read a set of registers */
        virtual std::vector<uint16_t> readRegisters(
            int address, bool input_registers, int start, int length) = 0;

        /** Read a set of registers */
        virtual void readRegisters(
            uint16_t* values,
            int address, bool input_registers, int start, int length
        ) = 0;

        virtual uint16_t readSingleRegister(
            int address, bool input_registers, int register_id
        ) = 0;

        virtual void writeSingleRegister(
            int address, uint16_t register_id, uint16_t value
        ) = 0;

        virtual void writeSingleCoil(int address, uint16_t register_id, bool value) = 0;

        virtual std::vector<bool> readDigitalInputs(
            int address, bool coils, uint16_t register_id, uint16_t count
        ) = 0;
    };
}

#endif