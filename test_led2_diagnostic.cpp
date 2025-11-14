#include "unlook/hardware/AS1170Controller.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

using namespace unlook::hardware;

int main() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "LED2 (FLOOD ILLUMINATOR) DIAGNOSTIC TEST" << std::endl;
    std::cout << "========================================\n" << std::endl;

    auto led_controller = AS1170Controller::getInstance();

    if (!led_controller) {
        std::cerr << "ERROR: Failed to get AS1170Controller instance" << std::endl;
        return 1;
    }

    std::cout << "✓ AS1170Controller initialized\n" << std::endl;

    // Test incrementalmente con diagnostica completa
    std::vector<uint16_t> test_currents = {50, 100, 150, 200, 250, 300};

    for (uint16_t current_ma : test_currents) {
        std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
        std::cout << "Testing LED2 at " << current_ma << " mA" << std::endl;
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;

        // Enable LED2
        std::cout << "  → Enabling LED2 at " << current_ma << " mA..." << std::endl;
        bool success = led_controller->setLEDState(
            AS1170Controller::LEDChannel::LED2,
            true,
            current_ma
        );

        if (!success) {
            std::cerr << "  ✗ FAILED to enable LED2" << std::endl;
            std::cerr << "  Possible causes:" << std::endl;
            std::cerr << "    - LED2 not connected" << std::endl;
            std::cerr << "    - LED2 polarity reversed" << std::endl;
            std::cerr << "    - AS1170 protection activated" << std::endl;
            break;
        }

        std::cout << "  ✓ LED2 command sent successfully" << std::endl;

        // Wait for LED to stabilize
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Read back actual current (if available)
        std::cout << "  Checking LED2 status..." << std::endl;

        // Wait for observation
        std::cout << "  → Waiting 5 seconds for visual observation..." << std::endl;
        for (int i = 5; i > 0; i--) {
            std::cout << "    " << i << "... " << std::flush;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "0\n" << std::endl;

        // Turn off before next test
        std::cout << "  → Turning OFF LED2..." << std::endl;
        led_controller->setLEDState(AS1170Controller::LEDChannel::LED2, false, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "Diagnostic test complete!" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "\nDid you see any change in the cameras?" << std::endl;
    std::cout << "If NO:" << std::endl;
    std::cout << "  1. Check LED2 physical connection" << std::endl;
    std::cout << "  2. Verify LED2 polarity (+ and -)" << std::endl;
    std::cout << "  3. Check if LED2 is the correct LED type" << std::endl;
    std::cout << "\nIf YES, note at which current (mA) you first saw light" << std::endl;
    std::cout << "========================================\n" << std::endl;

    return 0;
}
