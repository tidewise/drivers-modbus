rock_library(modbus
    SOURCES RTU.cpp Master.cpp Exceptions.cpp
    HEADERS RTU.hpp Frame.hpp Master.hpp Exceptions.hpp
    DEPS_PKGCONFIG base-types iodrivers_base)

rock_executable(modbus_ctl Main.cpp
    DEPS modbus)

