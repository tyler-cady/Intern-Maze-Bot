#include <gtest/gtest.h>
#include "mouse.h"

// Helper functions to simulate resource tracking
void simulateCpuUsage() {
    // Simulate some CPU usage
}

void simulateRamUsage() {
    // Simulate some RAM usage
}

void simulateDiskUsage() {
    // Simulate some Disk usage
}

void simulateRuntime() {
    // Simulate some Runtime usage
}

// Mock Serial class to capture the print statements
class MockSerial {
public:
    void begin(int baudRate) {}
    void print(const char* str) {
        output += str;
    }
    void print(int value) {
        output += std::to_string(value);
    }
    void println(const char* str) {
        output += str;
        output += "\n";
    }
    void println(int value) {
        output += std::to_string(value);
        output += "\n";
    }
    std::string output;
};

MockSerial Serial;

class MouseTest : public ::testing::Test {
protected:
    Mouse mouse;

    void SetUp() override {
        mouse.setup();
    }

    void TearDown() override {
        // Cleanup code here
    }
};

TEST_F(MouseTest, CpuUsageTest) {
    simulateCpuUsage();
    mouse.readEncoders();
    EXPECT_NE(Serial.output.find("Encoder1:"), std::string::npos);
    EXPECT_NE(Serial.output.find("Encoder2:"), std::string::npos);
}

TEST_F(MouseTest, RamUsageTest) {
    simulateRamUsage();
    mouse.readEncoders();
    EXPECT_NE(Serial.output.find("Encoder1:"), std::string::npos);
    EXPECT_NE(Serial.output.find("Encoder2:"), std::string::npos);
}

TEST_F(MouseTest, DiskUsageTest) {
    simulateDiskUsage();
    mouse.readEncoders();
    EXPECT_NE(Serial.output.find("Encoder1:"), std::string::npos);
    EXPECT_NE(Serial.output.find("Encoder2:"), std::string::npos);
}

TEST_F(MouseTest, RuntimeTest) {
    simulateRuntime();
    mouse.readEncoders();
    EXPECT_NE(Serial.output.find("Encoder1:"), std::string::npos);
    EXPECT_NE(Serial.output.find("Encoder2:"), std::string::npos);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}