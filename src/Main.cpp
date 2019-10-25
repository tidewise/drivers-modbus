#include <iostream>
#include <modbus/Master.hpp>

using namespace std;
using namespace modbus;

void usage(ostream& stream) {
    stream << "usage: modbus_ctl URI CMD\n"
           << "\n"
           << "Available Commands\n"
           << "  read-holding ID REG (LENGTH): read holding registers\n"
           << "  read-input ID REG (LENGTH): read input registers\n"
           << endl;
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        bool error = argc == 1 ? 0 : 1;
        usage(error ? cerr : cout);
        return error;
    }

    string uri = argv[1];
    string cmd = argv[2];

    if (cmd == "read-holding" || cmd == "read-input") {
        if (argc < 5) {
            cerr << "missing register to read\n\n";
            usage(cerr);
            return 1;
        }
        else if (argc > 6) {
            cerr << "too many arguments\n\n";
            usage(cerr);
            return 1;
        }

        modbus::Master modbus_master;
        modbus_master.openURI(uri);
        int address = std::atoi(argv[3]);
        int start_register = std::atoi(argv[4]);
        size_t length = (argc > 6) ? std::atoi(argv[5]) : 1;
        bool input = cmd == "read-input";
        auto result = modbus_master.readRegisters(
            address, input, start_register, length
        );

        if (result.size() != length) {
            std::cerr << "queried " << length << " registers, but received only "
                      << result.size() << std::endl;
        }

        for (size_t i = 0; i < length; ++i) {
            std::cout << dec << start_register + i << ": " << result[i] << std::endl;
        }
    }
    else {
        cerr << "unknown command '" << cmd << "'\n\n";
        usage(cerr);
        return 1;
    }
    return 0;
}
