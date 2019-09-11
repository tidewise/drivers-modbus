rock_library(modbus
    SOURCES RTU.cpp
    HEADERS RTU.hpp
    DEPS_PKGCONFIG base-types)

rock_executable(modbus_bin Main.cpp
    DEPS modbus)

