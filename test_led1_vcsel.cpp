#include "unlook/hardware/AS1170Controller.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

using namespace unlook::hardware;

int main() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "LED1 (VCSEL DOT PROJECTOR) TEST" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Testing LED1 from 0 to 300mA with 20mA steps" << std::endl;
    std::cout << "LED1 should be VCSEL (dot pattern projector)" << std::endl;
    std::cout << "Watch cameras for dot pattern!" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // Get AS1170 controller singleton
    auto led_controller = AS1170Controller::getInstance();

    if (!led_controller) {
        std::cerr << "ERROR: Failed to get AS1170Controller instance" << std::endl;
        return 1;
    }

    std::cout << "✓ AS1170Controller initialized" << std::endl;
    std::cout << "Starting in 2 seconds...\n" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Test parameters
    const uint16_t START_CURRENT_MA = 0;
    const uint16_t END_CURRENT_MA = 300;
    const uint16_t STEP_MA = 20;
    const int WAIT_TIME_SECONDS = 3;

    // Loop through current values
    for (uint16_t current_ma = START_CURRENT_MA; current_ma <= END_CURRENT_MA; current_ma += STEP_MA) {
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
        std::cout << "STEP: Setting LED1 to " << std::setw(3) << current_ma << " mA" << std::endl;
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;

        if (current_ma == 0) {
            std::cout << "  → Turning LED1 OFF" << std::endl;
            bool success = led_controller->setLEDState(AS1170Controller::LEDChannel::LED1, false, 0);
            if (!success) {
                std::cerr << "  ✗ Failed to turn OFF LED1" << std::endl;
            } else {
                std::cout << "  ✓ LED1 OFF" << std::endl;
            }
        } else {
            std::cout << "  → Setting LED1 current: " << current_ma << " mA" << std::endl;

            bool enable_success = led_controller->setLEDState(
                AS1170Controller::LEDChannel::LED1,
                true,
                current_ma
            );

            if (!enable_success) {
                std::cerr << "  ✗ Failed to enable LED1" << std::endl;
                continue;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::cout << "  ✓ LED1 enabled at " << current_ma << " mA" << std::endl;
        }

        // Wait and observe
        std::cout << "  Waiting " << WAIT_TIME_SECONDS << " seconds..." << std::endl;
        for (int i = WAIT_TIME_SECONDS; i > 0; i--) {
            std::cout << "    " << i << "... " << std::flush;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "0" << std::endl;
    }

    // Final cleanup
    std::cout << "\n========================================" << std::endl;
    std::cout << "Test complete! Turning OFF LED1..." << std::endl;
    led_controller->setLEDState(AS1170Controller::LEDChannel::LED1, false, 0);
    std::cout << "✓ LED1 OFF" << std::endl;
    std::cout << "========================================\n" << std::endl;

    return 0;
}
