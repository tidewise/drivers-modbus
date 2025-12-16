#include <fcntl.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <iodrivers_base/FixtureGTest.hpp>
#include <modbus/Exceptions.hpp>
#include <modbus/RTU.hpp>
#include <modbus/RTUMaster.hpp>
#include <thread>

using namespace std;
using base::Time;
using testing::ElementsAreArray;
using namespace modbus;

struct RTUMasterTest : public ::testing::Test, iodrivers_base::Fixture<RTUMaster> {
    int pipeTX = -1;

    RTUMasterTest()
    {
    }

    ~RTUMasterTest()
    {
        if (pipeTX != -1) {
            close(pipeTX);
        }
    }

    void openPipe()
    {
        int pipes[2];
        ASSERT_EQ(pipe(pipes), 0);
        int rx = pipes[0];
        int tx = pipes[1];

        long fd_flags = fcntl(rx, F_GETFL);
        fcntl(rx, F_SETFL, fd_flags | O_NONBLOCK);

        driver.setFileDescriptor(rx, true);
        driver.setErrorIncrement(1);
        driver.setErrorThreshold(2);
        pipeTX = tx;
    }

    void writeToPipe(uint8_t const* bytes, int size)
    {
        ASSERT_EQ(write(pipeTX, bytes, size), 1);
    }
};

TEST_F(RTUMasterTest, it_throws_if_calling_readPacket)
{
    driver.openURI("test://");
    // push one byte to get into extractPacket
    uint8_t buffer[1];
    pushDataToDriver(buffer, buffer + 1);
    ASSERT_THROW(driver.readPacket(buffer, 1024), std::logic_error);
}

TEST_F(RTUMasterTest, it_uses_the_interframe_delay_to_determine_the_end_of_a_frame)
{
    openPipe();

    uint8_t bytes[] = {0x02, 0x10, 1, 2, 3, 4, 5, 0x34, 0xEB};
    thread writeThread([this, &bytes] {
        for (uint8_t i = 0; i < 9; ++i) {
            writeToPipe(bytes + i, 1);
            usleep(1000);
        }
    });
    driver.setInterframeDelay(Time::fromMilliseconds(10));
    driver.setReadTimeout(Time::fromSeconds(1));

    Time start = Time::now();
    Frame f = driver.readFrame();
    ASSERT_EQ(0x02, f.address);
    ASSERT_EQ(0x10, f.function);
    uint8_t payload[5] = {1, 2, 3, 4, 5};
    ASSERT_THAT(f.payload, ElementsAreArray(payload));
    ASSERT_LE(Time::now() - start, Time::fromMilliseconds(50));

    writeThread.join();
}

TEST_F(RTUMasterTest, it_does_a_modbus_broadcast)
{
    driver.openURI("test://");

    driver.broadcast(0x10, vector<uint8_t>{6, 7, 8, 9});
    auto bytes = readDataFromDriver();

    uint8_t expected[] = {0x00, 0x10, 6, 7, 8, 9, 0xB7, 0x57};
    ASSERT_THAT(bytes, ElementsAreArray(expected));
}

TEST_F(RTUMasterTest, it_retries_on_CRC_error)
{
    driver.openURI("test://");
    driver.setErrorIncrement(1);
    driver.setErrorThreshold(4);

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x07});
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x07});
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x07});
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x06});

    ASSERT_EQ((vector<uint16_t>{0x1234, 0x5678}),
        driver.readRegisters(0x10, false, 0xabcd, 2));
    auto statistics = driver.getRTUStats();
    // It tries thrice (+3) and then it succeeds once (-1)
    ASSERT_EQ(2, statistics.error_count);
    ASSERT_EQ(3, statistics.total_CRC_error_count);
    ASSERT_EQ(0, statistics.total_unexpected_reply_error_count);
    ASSERT_EQ(1, statistics.total_sucess_count);
}

TEST_F(RTUMasterTest, it_retries_on_unexpected_reply_function)
{
    driver.openURI("test://");
    driver.setErrorIncrement(3);
    driver.setErrorThreshold(12);

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x04, 0x4, 0x12, 0x34, 0x56, 0x78, 0x81, 0xb1});
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x04, 0x4, 0x12, 0x34, 0x56, 0x78, 0x81, 0xb1});
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x04, 0x4, 0x12, 0x34, 0x56, 0x78, 0x81, 0xb1});
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x06});

    ASSERT_EQ((vector<uint16_t>{0x1234, 0x5678}),
        driver.readRegisters(0x10, false, 0xabcd, 2));
    auto statistics = driver.getRTUStats();
    // It tries thrice (+9) and then it succeeds once (-1)
    ASSERT_EQ(8, statistics.error_count);
    ASSERT_EQ(0, statistics.total_CRC_error_count);
    ASSERT_EQ(3, statistics.total_unexpected_reply_error_count);
    ASSERT_EQ(1, statistics.total_sucess_count);
}

TEST_F(RTUMasterTest, it_retries_on_unexpected_reply_length)
{
    driver.openURI("test://");
    driver.setErrorIncrement(3);
    driver.setErrorThreshold(12);

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x77, 0x47, 0x86});
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x77, 0x47, 0x86});
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x77, 0x47, 0x86});
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x06});

    ASSERT_EQ((vector<uint16_t>{0x1234, 0x5678}),
        driver.readRegisters(0x10, false, 0xabcd, 2));
    auto statistics = driver.getRTUStats();
    // It tries thrice (+9) and then it succeeds once (-1)
    ASSERT_EQ(8, statistics.error_count);
    ASSERT_EQ(0, statistics.total_CRC_error_count);
    ASSERT_EQ(3, statistics.total_unexpected_reply_error_count);
    ASSERT_EQ(1, statistics.total_sucess_count);
}

TEST_F(RTUMasterTest, it_accounts_for_invalid_data)
{
    driver.openURI("test://");
    driver.setErrorIncrement(1);
    driver.setErrorThreshold(2);

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x07});
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x06});
    driver.readRegisters(0x10, false, 0xabcd, 2);
    ASSERT_EQ(9, driver.getStatus().bad_rx);
}

TEST_F(RTUMasterTest, it_throws_if_the_error_count_reaches_the_max_value)
{
    driver.openURI("test://");
    driver.setErrorIncrement(1);
    driver.setErrorThreshold(3);

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x07});
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x07});
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x07});

    ASSERT_THROW(driver.readRegisters(0x10, false, 0xabcd, 2), modbus::RTU::InvalidCRC);
}

TEST_F(RTUMasterTest, it_does_a_holding_register_read_request)
{
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91},
        vector<uint8_t>{0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x06});
    ASSERT_EQ((vector<uint16_t>{0x1234, 0x5678}),
        driver.readRegisters(0x10, false, 0xabcd, 2));
}

TEST_F(RTUMasterTest, it_does_a_single_holding_register_read_request)
{
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x03, 0xab, 0xcd, 0x00, 0x01, 0x36, 0x90},
        vector<uint8_t>{0x10, 0x03, 0x2, 0x12, 0x34, 0x49, 0x30});
    ASSERT_EQ(0x1234, driver.readSingleRegister(0x10, false, 0xabcd));
}

TEST_F(RTUMasterTest, it_does_a_register_write_request)
{
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x06, 0xab, 0xcd, 0x12, 0x34, 0x36, 0x27},
        vector<uint8_t>{0x10, 0x06, 0xab, 0xcd, 0x12, 0x34, 0x36, 0x27});
    driver.writeSingleRegister(0x10, 0xabcd, 0x1234);
}

TEST_F(RTUMasterTest, it_reads_multiple_coils)
{
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    // CRC Computed with https://www.lammertbies.nl/comm/info/crc-calculation.html
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x01, 0x12, 0x34, 0x00, 0x09, 0xbb, 0xfb},
        vector<uint8_t>{0x10, 0x01, 0x02, 0xab, 0xcd, 0xfb, 0x5a});
    auto values = driver.readDigitalInputs(0x10, true, 0x1234, 9);

    bool expected[9] = {true, true, false, true, false, true, false, true, true};
    ASSERT_THAT(values, ElementsAreArray(expected));
}

TEST_F(RTUMasterTest, it_reads_multiple_digital_inputs)
{
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    // CRC Computed with https://www.lammertbies.nl/comm/info/crc-calculation.html
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x02, 0x12, 0x34, 0x00, 0x09, 0xff, 0xfb},
        vector<uint8_t>{0x10, 0x02, 0x02, 0xab, 0xcd, 0xfb, 0x1e});
    auto values = driver.readDigitalInputs(0x10, false, 0x1234, 9);

    bool expected[9] = {true, true, false, true, false, true, false, true, true};
    ASSERT_THAT(values, ElementsAreArray(expected));
}

TEST_F(RTUMasterTest, it_does_a_coil_ON_write_request)
{
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x05, 0x12, 0x34, 0xff, 0x00, 0xcb, 0xcd},
        vector<uint8_t>{0x10, 0x05, 0x12, 0x34, 0xff, 0x00, 0xcb, 0xcd});
    driver.writeSingleCoil(0x10, 0x1234, true);
}

TEST_F(RTUMasterTest, it_does_a_coil_OFF_write_request)
{
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(vector<uint8_t>{0x10, 0x05, 0x12, 0x34, 0x00, 0x00, 0x8a, 0x3d},
        vector<uint8_t>{0x10, 0x05, 0x12, 0x34, 0x00, 0x00, 0x8a, 0x3d});
    driver.writeSingleCoil(0x10, 0x1234, false);
}