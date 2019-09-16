# drivers/modbus

Implementation of the Modbus protocol

It implements only the protocol. This is meant to be used by other packages
that have to communicate with devices through Modbus

Frame handling is implemented in the RTU namespace (in `modbus/RTU.hpp`)
while the Master class (in `modbus/Master.hpp`) implements modbus
request/reply mechanisms.

## Reference Documents

- Modbus over serial line (modbus.org)
- Modbus application protocol v1.1b (modbus.org)

License
-------
BSD 3-clause

Installation
------------
The easiest way to build and install this package is to use Rock's build system.
See [this page](http://rock-robotics.org/stable/documentation/installation.html)
on how to install Rock.

However, if you feel that it's too heavy for your needs, Rock aims at having
most of its "library" packages (such as this one) to follow best practices. See
[this page](http://rock-robotics.org/stable/documentation/packages/outside_of_rock.html)
for installation instructions outside of Rock.
