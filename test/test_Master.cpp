#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <modbus/Master.hpp>
#include <iodrivers_base/FixtureGTest.hpp>
#include <fcntl.h>
#include <thread>

using namespace std;
using testing::ElementsAreArray;
using base::Time;
using namespace modbus;

struct MasterTest : public ::testing::Test, iodrivers_base::Fixture<Master> {
    int pipeTX = -1;

    MasterTest() {
    }

    ~MasterTest() {
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

TEST_F(MasterTest, it_throws_if_calling_readPacket) {
    driver.openURI("test://");
    // push one byte to get into extractPacket
    uint8_t buffer[1];
    pushDataToDriver(buffer, buffer + 1);
    ASSERT_THROW(driver.readPacket(buffer, 1024), std::logic_error);
}

TEST_F(MasterTest, it_uses_the_interframe_delay_to_determine_the_end_of_a_frame) {
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

TEST_F(MasterTest, it_does_a_modbus_request) {
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

TEST_F(MasterTest, it_does_a_modbus_broadcast) {
    driver.openURI("test://");

    driver.broadcast(0x10, vector<uint8_t>{6, 7, 8, 9});
    auto bytes = readDataFromDriver();

    uint8_t expected[] = { 0x00, 0x10, 6, 7, 8, 9, 0xB7, 0x57 };
    ASSERT_THAT(bytes, ElementsAreArray(expected));
}
