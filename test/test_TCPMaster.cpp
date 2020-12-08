#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <modbus/TCPMaster.hpp>
#include <iodrivers_base/FixtureGTest.hpp>
#include <fcntl.h>
#include <thread>

using namespace modbus;

using namespace std;
using testing::ElementsAreArray;
using base::Time;
using namespace modbus;

struct PreconfiguredDriver : public TCPMaster {
    PreconfiguredDriver()
        : TCPMaster(256) {
    }
};

struct TCPMasterTest : public ::testing::Test, iodrivers_base::Fixture<PreconfiguredDriver> {
    int pipeTX = -1;

    TCPMasterTest() {
    }

    ~TCPMasterTest() {
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

TEST_F(TCPMasterTest, it_does_a_modbus_request) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    uint8_t request[] = { 0xaa, 0x01, 0, 0, 0, 7, 0x10, 0x02, 1, 2, 3, 4, 5 };
    uint8_t reply[] = { 0xaa, 0x01, 0, 0, 0, 6, 0x10, 0x02, 6, 7, 8, 9 };
    EXPECT_REPLY(vector<uint8_t>(request, request + sizeof(request)),
                 vector<uint8_t>(reply, reply + sizeof(reply)));

    Frame f = driver.request(0x10, 0x02, vector<uint8_t>{1, 2, 3, 4, 5});
    ASSERT_EQ(0x10, f.address);
    ASSERT_EQ(0x02, f.function);
    uint8_t expected[4] = { 6, 7, 8, 9 };
    ASSERT_THAT(f.payload, ElementsAreArray(expected));
}

TEST_F(TCPMasterTest, it_throws_if_receiving_an_unexpected_function_code_in_reply) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    uint8_t request[] = { 0xaa, 0x01, 0, 0, 0, 7, 0x10, 0x02, 1, 2, 3, 4, 5 };
    uint8_t reply[] = { 0xaa, 0x01, 0, 0, 0, 6, 0x10, 0x03, 6, 7, 8, 9 };
    EXPECT_REPLY(vector<uint8_t>(request, request + sizeof(request)),
                 vector<uint8_t>(reply, reply + sizeof(reply)));

    ASSERT_THROW(
        driver.request(0x10, 0x02, vector<uint8_t>{1, 2, 3, 4, 5}),
        UnexpectedReply);
}

TEST_F(TCPMasterTest, it_throws_if_receiving_an_exception_function_code_in_reply) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    uint8_t request[] = { 0xaa, 0x01, 0, 0, 0, 7, 0x10, 0x02, 1, 2, 3, 4, 5 };
    uint8_t reply[] = { 0xaa, 0x01, 0, 0, 0, 6, 0x10, 0x82, 6, 7, 8, 9 };
    EXPECT_REPLY(vector<uint8_t>(request, request + sizeof(request)),
                 vector<uint8_t>(reply, reply + sizeof(reply)));

    ASSERT_THROW(
        driver.request(0x10, 0x02, vector<uint8_t>{1, 2, 3, 4, 5}),
        RequestException);
}

TEST_F(TCPMasterTest, it_handles_a_malformed_exception_reply_that_is_lacking_the_exception_code) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    uint8_t request[] = { 0xaa, 0x01, 0, 0, 0, 7, 0x10, 0x02, 1, 2, 3, 4, 5 };
    uint8_t reply[] = { 0xaa, 0x01, 0, 0, 0, 2, 0x10, 0x82 };
    EXPECT_REPLY(vector<uint8_t>(request, request + sizeof(request)),
                 vector<uint8_t>(reply, reply + sizeof(reply)));

    ASSERT_THROW(
        driver.request(0x10, 0x02, vector<uint8_t>{1, 2, 3, 4, 5}),
        RequestException);
}

TEST_F(TCPMasterTest, it_does_a_holding_register_read_request) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(
        vector<uint8_t>{ 0xaa, 0x01, 0, 0, 0, 6, 0x10, 0x03, 0xab, 0xcd, 0, 2 },
        vector<uint8_t>{ 0xaa, 0x01, 0, 0, 0, 7, 0x10, 0x03, 4, 0xa, 2, 0xa, 3 }
    );

    ASSERT_EQ((vector<uint16_t>{ 0xa02, 0xa03 }),
              driver.readRegisters(0x10, false, 0xabcd, 2));
}

TEST_F(TCPMasterTest, it_does_a_single_holding_register_read_request) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(
        vector<uint8_t>{ 0xaa, 0x01, 0, 0, 0, 6, 0x10, 0x03, 0xab, 0xcd, 0, 1 },
        vector<uint8_t>{ 0xaa, 0x01, 0, 0, 0, 5, 0x10, 0x03, 2, 0x12, 0x34 }
    );

    ASSERT_EQ(0x1234, driver.readSingleRegister(0x10, false, 0xabcd));
}

TEST_F(TCPMasterTest, it_does_a_register_write_request) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(
        vector<uint8_t>{ 0xaa, 0x01, 0, 0, 0, 6, 0x10, 0x06, 0xab, 0xcd, 0x12, 0x34 },
        vector<uint8_t>{ 0xaa, 0x01, 0, 0, 0, 2, 0x10, 0x06 }
    );
    driver.writeSingleRegister(0x10, 0xabcd, 0x1234);
}

