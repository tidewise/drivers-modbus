#include <iostream>
#include <modbus/Dummy.hpp>

int main(int argc, char** argv)
{
    modbus::DummyClass dummyClass;
    dummyClass.welcome();

    return 0;
}
