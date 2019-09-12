rock_library(modbus
    SOURCES RTU.cpp Master.cpp
    HEADERS RTU.hpp Frame.hpp Master.hpp
    DEPS_PKGCONFIG base-types iodrivers_base)

rock_executable(modbus_bin Main.cpp
    DEPS modbus)

