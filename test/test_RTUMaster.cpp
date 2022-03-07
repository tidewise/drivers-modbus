#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <modbus/RTU.hpp>
#include <modbus/RTUMaster.hpp>
#include <modbus/Exceptions.hpp>
#include <iodrivers_base/FixtureGTest.hpp>
#include <fcntl.h>
#include <thread>

using namespace std;
using testing::ElementsAreArray;
using base::Time;
using namespace modbus;

struct RTUMasterTest : public ::testing::Test, iodrivers_base::Fixture<RTUMaster> {
    int pipeTX = -1;

    RTUMasterTest() {
    }

    ~RTUMasterTest() {
        if (pipeTX != -1) {
            close(pipeTX);
        }
    }

    void openPipe() {
        int pipes[2];
        ASSERT_EQ(pipe(pipes), 0);
        int rx = pipes[0];
        int tx = pipes[1];

        long fd_flags = fcntl(rx, F_GETFL);
        fcntl(rx, F_SETFL, fd_flags | O_NONBLOCK);

        driver.setFileDescriptor(rx, true);
        pipeTX = tx;
    }

    void writeToPipe(uint8_t const* bytes, int size) {
        ASSERT_EQ(write(pipeTX, bytes, size), 1);
    }
};

TEST_F(RTUMasterTest, it_throws_if_calling_readPacket) {
    driver.openURI("test://");
    // push one byte to get into extractPacket
    uint8_t buffer[1];
    pushDataToDriver(buffer, buffer + 1);
    ASSERT_THROW(driver.readPacket(buffer, 1024), std::logic_error);
}

TEST_F(RTUMasterTest, it_uses_the_interframe_delay_to_determine_the_end_of_a_frame) {
    openPipe();

    uint8_t bytes[] = { 0x02, 0x10, 1, 2, 3, 4, 5, 0x34, 0xEB };
    thread writeThread([this,&bytes]{
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
    uint8_t payload[5] = { 1, 2, 3, 4, 5 };
    ASSERT_THAT(f.payload, ElementsAreArray(payload));
    ASSERT_LE(Time::now() - start, Time::fromMilliseconds(50));

    writeThread.join();
}

TEST_F(RTUMasterTest, it_does_a_modbus_request) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    uint8_t request[] = { 0x02, 0x10, 1, 2, 3, 4, 5, 0x34, 0xEB };
    uint8_t reply[] = { 0x02, 0x10, 6, 7, 8, 9, 0xB6, 0xB5 };
    EXPECT_REPLY(vector<uint8_t>(request, request + 9),
                 vector<uint8_t>(reply, reply + 8));

    Frame f = driver.request(0x02, 0x10, vector<uint8_t>{1, 2, 3, 4, 5});
    ASSERT_EQ(0x02, f.address);
    ASSERT_EQ(0x10, f.function);
    uint8_t expected[] = { 6, 7, 8, 9 };
    ASSERT_THAT(f.payload, ElementsAreArray(expected));
}

TEST_F(RTUMasterTest, it_throws_if_receiving_an_unexpected_function_code_in_reply) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    uint8_t request[] = { 0x02, 0x10, 1, 2, 3, 4, 5, 0x34, 0xEB };
    uint8_t reply[] = { 0x02, 0x00, 0x01, 0x11, 0xC0 };
    EXPECT_REPLY(vector<uint8_t>(request, request + 9),
                 vector<uint8_t>(reply, reply + 5));

    ASSERT_THROW(
        driver.request(0x02, 0x10, vector<uint8_t>{1, 2, 3, 4, 5}),
        UnexpectedReply);
}

TEST_F(RTUMasterTest, it_throws_if_receiving_an_exception_function_code_in_reply) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    uint8_t request[] = { 0x02, 0x10, 1, 2, 3, 4, 5, 0x34, 0xEB };
    uint8_t reply[] = { 0x02, 0x90, 0x01, 0x7D, 0xC0 };
    EXPECT_REPLY(vector<uint8_t>(request, request + 9),
                 vector<uint8_t>(reply, reply + 5));

    ASSERT_THROW(
        driver.request(0x02, 0x10, vector<uint8_t>{1, 2, 3, 4, 5}),
        RequestException);
}

TEST_F(RTUMasterTest, it_handles_a_malformed_exception_reply_that_is_lacking_the_exception_code) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    uint8_t request[] = { 0x02, 0x10, 1, 2, 3, 4, 5, 0x34, 0xEB };
    uint8_t reply[] = { 0x02, 0x90, 0x00, 0xBC };
    EXPECT_REPLY(vector<uint8_t>(request, request + 9),
                 vector<uint8_t>(reply, reply + 4));

    ASSERT_THROW(
        driver.request(0x02, 0x10, vector<uint8_t>{1, 2, 3, 4, 5}),
        RequestException);
}

TEST_F(RTUMasterTest, it_does_a_modbus_broadcast) {
    driver.openURI("test://");

    driver.broadcast(0x10, vector<uint8_t>{6, 7, 8, 9});
    auto bytes = readDataFromDriver();

    uint8_t expected[] = { 0x00, 0x10, 6, 7, 8, 9, 0xB7, 0x57 };
    ASSERT_THAT(bytes, ElementsAreArray(expected));
}

TEST_F(RTUMasterTest, it_retries_on_CRC_error) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(
        vector<uint8_t>{ 0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91 },
        vector<uint8_t>{ 0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x07 }
    );
    EXPECT_REPLY(
        vector<uint8_t>{ 0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91 },
        vector<uint8_t>{ 0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x06 }
    );
    ASSERT_EQ((vector<uint16_t>{ 0x1234, 0x5678 }),
              driver.readRegisters(0x10, false, 0xabcd, 2));
}

TEST_F(RTUMasterTest, it_accounts_for_invalid_data) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(
        vector<uint8_t>{ 0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91 },
        vector<uint8_t>{ 0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x07 }
    );
    EXPECT_REPLY(
        vector<uint8_t>{ 0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91 },
        vector<uint8_t>{ 0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x06 }
    );
    driver.readRegisters(0x10, false, 0xabcd, 2);
    ASSERT_EQ(9, driver.getStatus().bad_rx);
}

TEST_F(RTUMasterTest, it_does_timeout_if_the_CRC_error_is_permanent) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(
        vector<uint8_t>{ 0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91 },
        vector<uint8_t>{ 0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x07 }
    );
    driver.setReadTimeout(Time());
    ASSERT_THROW(driver.readRegisters(0x10, false, 0xabcd, 2), modbus::RTU::InvalidCRC);
}

TEST_F(RTUMasterTest, it_does_a_holding_register_read_request) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(
        vector<uint8_t>{ 0x10, 0x03, 0xab, 0xcd, 0x00, 0x02, 0x76, 0x91 },
        vector<uint8_t>{ 0x10, 0x03, 0x4, 0x12, 0x34, 0x56, 0x78, 0x80, 0x06 }
    );
    ASSERT_EQ((vector<uint16_t>{ 0x1234, 0x5678 }),
              driver.readRegisters(0x10, false, 0xabcd, 2));
}

TEST_F(RTUMasterTest, it_does_a_single_holding_register_read_request) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(
        vector<uint8_t>{ 0x10, 0x03, 0xab, 0xcd, 0x00, 0x01, 0x36, 0x90 },
        vector<uint8_t>{ 0x10, 0x03, 0x2, 0x12, 0x34, 0x49, 0x30 }
    );
    ASSERT_EQ(0x1234, driver.readSingleRegister(0x10, false, 0xabcd));
}

TEST_F(RTUMasterTest, it_does_a_register_write_request) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(
        vector<uint8_t>{ 0x10, 0x06, 0xab, 0xcd, 0x12, 0x34, 0x36, 0x27 },
        vector<uint8_t>{ 0x10, 0x06, 0xab, 0xcd, 0x5a, 0x40 }
    );
    driver.writeSingleRegister(0x10, 0xabcd, 0x1234);
}

TEST_F(RTUMasterTest, it_reads_multiple_coils) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    // CRC Computed with https://www.lammertbies.nl/comm/info/crc-calculation.html
    EXPECT_REPLY(
        vector<uint8_t>{ 0x10, 0x01, 0x12, 0x34, 0x00, 0x09, 0xbb, 0xfb },
        vector<uint8_t>{ 0x10, 0x01, 0x02, 0xab, 0xcd, 0xfb, 0x5a }
    );
    auto values = driver.readDigitalInputs(0x10, true, 0x1234, 9);

    bool expected[9] = { true, true, false, true, false, true, false, true, true };
    ASSERT_THAT(values, ElementsAreArray(expected));
}

TEST_F(RTUMasterTest, it_reads_multiple_digital_inputs) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    // CRC Computed with https://www.lammertbies.nl/comm/info/crc-calculation.html
    EXPECT_REPLY(
        vector<uint8_t>{ 0x10, 0x02, 0x12, 0x34, 0x00, 0x09, 0xff, 0xfb },
        vector<uint8_t>{ 0x10, 0x02, 0x02, 0xab, 0xcd, 0xfb, 0x1e }
    );
    auto values = driver.readDigitalInputs(0x10, false, 0x1234, 9);

    bool expected[9] = { true, true, false, true, false, true, false, true, true };
    ASSERT_THAT(values, ElementsAreArray(expected));
}

TEST_F(RTUMasterTest, it_does_a_coil_ON_write_request) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(
        vector<uint8_t>{ 0x10, 0x05, 0x12, 0x34, 0xff, 0x00, 0xcb, 0xcd },
        vector<uint8_t>{ 0x10, 0x05, 0x12, 0x34, 0xff, 0x00, 0xcb, 0xcd }
    );
    driver.writeSingleCoil(0x10, 0x1234, true);
}

TEST_F(RTUMasterTest, it_does_a_coil_OFF_write_request) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(
        vector<uint8_t>{ 0x10, 0x05, 0x12, 0x34, 0x00, 0x00, 0x8a, 0x3d },
        vector<uint8_t>{ 0x10, 0x05, 0x12, 0x34, 0x00, 0x00, 0x8a, 0x3d }
    );
    driver.writeSingleCoil(0x10, 0x1234, false);
}