#include <iostream>
#include <list>
#include <memory>
#include <modbus/RTUMaster.hpp>
#include <modbus/TCPMaster.hpp>

using namespace std;
using namespace modbus;

void usage(ostream& stream) {
    stream << "usage: modbus_ctl URI [PROTOCOL] CMD\n"
           << "where URI is a iodrivers_base URI\n"
           << "      PROTOCOL is either rtu or tcp. It may be omitted, in which\n"
           << "               case rtu is used by default\n"
           << "\n"
           << "Available Commands\n"
           << "  read-holding ID REG (LENGTH): read holding registers\n"
           << "  read-input ID REG (LENGTH): read input registers\n"
           << "  read-coil ID REG (LENGTH): read coils\n"
           << "  read-din ID REG (LENGTH): read digital inputs\n"
           << "  write-coil ID REG VALUE: write a single coil\n"
           << endl;
}

int main(int argc, char** argv)
{
    list<string> args(argv + 1, argv + argc);
    if (args.size() < 2) {
        bool error = !args.empty();
        usage(error ? cerr : cout);
        return error;
    }

    string uri = args.front();
    args.pop_front();

    string protocol = "rtu";
    if (args.front() == "tcp" || args.front() == "rtu") {
        protocol = args.front();
        args.pop_front();
    }

    if (args.empty()) {
        usage(cerr);
        return 1;
    }

    string cmd = args.front();
    args.pop_front();

    std::unique_ptr<modbus::MasterInterface> modbus_master;
    if (protocol == "tcp") {
        auto* master = new modbus::TCPMaster(256);
        modbus_master.reset(master);
        master->openURI(uri);
    }
    else {
        auto* master = new modbus::RTUMaster();
        modbus_master.reset(master);
        master->openURI(uri);
    }

    if (cmd == "read-holding" || cmd == "read-input") {
        if (args.size() < 2) {
            cerr << "missing register to read\n\n";
            usage(cerr);
            return 1;
        }
        else if (args.size() > 3) {
            cerr << "too many arguments\n\n";
            usage(cerr);
            return 1;
        }

        int address = std::stoi(args.front());
        args.pop_front();
        int start_register = std::stoi(args.front());
        args.pop_front();

        size_t length = args.empty() ? std::stoi(args.front()) : 1;
        bool input = cmd == "read-input";
        auto result = modbus_master->readRegisters(
            address, input, start_register, length
        );

        for (size_t i = 0; i < length; ++i) {
            std::cout << dec << start_register + i << ": " << result[i] << std::endl;
        }
    }
    else if (cmd == "read-coil" || cmd == "read-din") {
        if (args.size() < 2) {
            cerr << "missing coil to read\n\n";
            usage(cerr);
            return 1;
        }
        else if (args.size() > 3) {
            cerr << "too many arguments\n\n";
            usage(cerr);
            return 1;
        }

        int address = std::stoi(args.front());
        args.pop_front();
        int start_input = std::stoi(args.front());
        args.pop_front();

        size_t length = args.empty() ? std::stoi(args.front()) : 1;
        bool coil = cmd == "read-coil";
        auto result = modbus_master->readDigitalInputs(
            address, coil, start_input, length
        );

        for (size_t i = 0; i < length; ++i) {
            std::cout << dec << start_input + i << ": " << result[i] << std::endl;
        }
    }
    else if (cmd == "write-coil") {
        if (args.size() != 3) {
            cerr << "wrong number of arguments\n\n";
            usage(cerr);
            return 1;
        }

        int address = std::stoi(args.front());
        args.pop_front();
        int input = std::stoi(args.front());
        args.pop_front();
        bool value = std::stoi(args.front());
        args.pop_front();

        modbus_master->writeSingleCoil(address, input, value);
    }
    else {
        cerr << "unknown command '" << cmd << "'\n\n";
        usage(cerr);
        return 1;
    }
    return 0;
}
