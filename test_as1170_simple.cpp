/**
 * @file test_as1170_simple.cpp
 * @brief Simple AS1170 I2C detection test
 */

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

int main() {
    std::cout << "=== Simple AS1170 I2C Communication Test ===" << std::endl;
    std::cout << "Testing I2C Bus 1, Address 0x30" << std::endl;

    // Open I2C bus
    int file = open("/dev/i2c-1", O_RDWR);
    if (file < 0) {
        std::cout << "ERROR: Could not open /dev/i2c-1" << std::endl;
        std::cout << "Make sure I2C bus 1 is enabled in config.txt" << std::endl;
        return 1;
    }

    // Set I2C address
    if (ioctl(file, I2C_SLAVE, 0x30) < 0) {
        std::cout << "ERROR: Could not set I2C address 0x30" << std::endl;
        close(file);
        return 1;
    }

    // Try to read from device (AS1170 status register)
    char buffer[1] = {0};
    int result = read(file, buffer, 1);

    if (result == 1) {
        std::cout << "SUCCESS: AS1170 detected at address 0x30 on bus 1!" << std::endl;
        std::cout << "Status byte: 0x" << std::hex << (int)buffer[0] << std::dec << std::endl;
    } else {
        std::cout << "WARNING: No response from AS1170 at address 0x30" << std::endl;
        std::cout << "Check hardware connections:" << std::endl;
        std::cout << "  - AS1170 power supply" << std::endl;
        std::cout << "  - I2C SDA/SCL connections" << std::endl;
        std::cout << "  - Correct I2C address (0x30)" << std::endl;
    }

    close(file);
    return 0;
}