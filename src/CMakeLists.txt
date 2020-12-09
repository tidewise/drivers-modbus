rock_library(modbus
    SOURCES TCP.cpp TCPMaster.cpp RTU.cpp RTUMaster.cpp
        Exceptions.cpp common.cpp
    HEADERS TCP.hpp TCPMaster.hpp RTU.hpp RTUMaster.hpp
        Master.hpp Frame.hpp Exceptions.hpp common.hpp
        MasterInterface.hpp Functions.hpp
    DEPS_PKGCONFIG base-types iodrivers_base)

rock_executable(modbus_ctl Main.cpp
    DEPS modbus)

