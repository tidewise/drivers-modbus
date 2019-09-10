rock_library(modbus
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(modbus_bin Main.cpp
    DEPS modbus)

