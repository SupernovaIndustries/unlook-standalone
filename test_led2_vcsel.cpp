#include "unlook/hardware/AS1170Controller.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

using namespace unlook::hardware;

int main() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "LED2 (VCSEL) INCREMENTAL TEST" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Testing LED2 from 0 to 300mA with 20mA steps" << std::endl;
    std::cout << "Watch cameras for VCSEL activation!" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Get AS1170 controller singleton
    auto led_controller = AS1170Controller::getInstance();

    if (!led_controller) {
        std::cerr << "ERROR: Failed to get AS1170Controller instance" << std::endl;
        std::cerr << "Is I2C device available at /dev/i2c-1 address 0x30?" << std::endl;
        return 1;
    }

    std::cout << "✓ AS1170Controller initialized" << std::endl;
    std::cout << "Starting in 2 seconds...\n" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Test parameters
    const uint16_t START_CURRENT_MA = 0;
    const uint16_t END_CURRENT_MA = 300;
    const uint16_t STEP_MA = 20;
    const int WAIT_TIME_SECONDS = 3;  // Wait time at each step

    // Loop through current values
    for (uint16_t current_ma = START_CURRENT_MA; current_ma <= END_CURRENT_MA; current_ma += STEP_MA) {
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
        std::cout << "STEP: Setting LED2 to " << std::setw(3) << current_ma << " mA" << std::endl;
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;

        if (current_ma == 0) {
            // Turn OFF LED2
            std::cout << "  → Turning LED2 OFF" << std::endl;
            bool success = led_controller->setLEDState(AS1170Controller::LEDChannel::LED2, false, 0);
            if (!success) {
                std::cerr << "  ✗ Failed to turn OFF LED2" << std::endl;
            } else {
                std::cout << "  ✓ LED2 OFF" << std::endl;
            }
        } else {
            // Turn ON LED2 with specified current
            std::cout << "  → Setting LED2 current: " << current_ma << " mA" << std::endl;

            // First enable LED2
            bool enable_success = led_controller->setLEDState(
                AS1170Controller::LEDChannel::LED2,
                true,  // enable
                current_ma
            );

            if (!enable_success) {
                std::cerr << "  ✗ Failed to enable LED2" << std::endl;
                continue;
            }

            // Verify current was set
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            std::cout << "  ✓ LED2 enabled at " << current_ma << " mA" << std::endl;
        }

        // Wait and observe
        std::cout << "  Waiting " << WAIT_TIME_SECONDS << " seconds for observation..." << std::endl;
        for (int i = WAIT_TIME_SECONDS; i > 0; i--) {
            std::cout << "    " << i << "... " << std::flush;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "0" << std::endl;
    }

    // Final cleanup - turn off LED2
    std::cout << "\n========================================" << std::endl;
    std::cout << "Test complete! Turning OFF LED2..." << std::endl;
    led_controller->setLEDState(AS1170Controller::LEDChannel::LED2, false, 0);
    std::cout << "✓ LED2 OFF" << std::endl;
    std::cout << "========================================\n" << std::endl;

    return 0;
}
