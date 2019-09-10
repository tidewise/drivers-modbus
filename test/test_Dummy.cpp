#include <boost/test/unit_test.hpp>
#include <modbus/Dummy.hpp>

using namespace modbus;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    modbus::DummyClass dummy;
    dummy.welcome();
}
