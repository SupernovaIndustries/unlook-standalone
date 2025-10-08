/**
 * VCSEL Diagnostic Tool
 * Tests both VCSEL channels on AS1170 controller
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <unlook/hardware/AS1170Controller.hpp>
#include <unlook/core/Logger.hpp>

using namespace unlook;
using namespace std::chrono_literals;

void printStatus(const hardware::AS1170Controller::AS1170Status& status) {
    std::cout << "\n=== AS1170 Status ===" << std::endl;
    std::cout << "Initialized: " << (status.initialized ? "YES" : "NO") << std::endl;
    std::cout << "I2C Connected: " << (status.i2c_connected ? "YES" : "NO") << std::endl;
    std::cout << "GPIO Configured: " << (status.gpio_configured ? "YES" : "NO") << std::endl;
    std::cout << "LED1 Current: " << status.led1_current_ma << " mA" << std::endl;
    std::cout << "LED2 Current: " << status.led2_current_ma << " mA" << std::endl;
    std::cout << "Temperature: " << std::fixed << std::setprecision(1) << status.temperature_c << "°C" << std::endl;
    if (!status.error_message.empty()) {
        std::cout << "Error: " << status.error_message << std::endl;
    }
    std::cout << "==================\n" << std::endl;
}

int main() {
    std::cout << "\n=== VCSEL Diagnostic Tool ===" << std::endl;
    std::cout << "Testing AS1170 dual VCSEL configuration\n" << std::endl;

    // Initialize logger
    auto& logger = core::Logger::getInstance();
    logger.initialize(core::LogLevel::DEBUG, true, false);

    // Get AS1170 controller
    auto as1170 = hardware::AS1170Controller::getInstance();

    // Initialize controller
    std::cout << "Initializing AS1170 controller..." << std::endl;
    if (!as1170->initialize()) {
        std::cerr << "Failed to initialize AS1170 controller!" << std::endl;
        std::cerr << "Check: I2C bus 1, address 0x30, GPIO 19" << std::endl;
        return -1;
    }

    std::cout << "AS1170 initialized successfully!\n" << std::endl;

    // Get initial status
    auto status = as1170->getStatus();
    printStatus(status);

    // Test sequence
    const int test_current = 200;  // mA
    const int on_duration = 1000;  // ms
    const int off_duration = 500;  // ms

    std::cout << "\n=== Starting VCSEL Test Sequence ===" << std::endl;
    std::cout << "Current: " << test_current << " mA per VCSEL" << std::endl;
    std::cout << "Duration: " << on_duration << " ms ON, " << off_duration << " ms OFF\n" << std::endl;

    for (int cycle = 1; cycle <= 3; cycle++) {
        std::cout << "\n--- Cycle " << cycle << "/3 ---" << std::endl;

        // Test LED1 (VCSEL 1 - Upper)
        std::cout << "[" << cycle << ".1] Activating LED1 (VCSEL1 - Upper)..." << std::endl;
        if (as1170->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, true, test_current)) {
            std::cout << "       LED1 ON at " << test_current << " mA ✓" << std::endl;
        } else {
            std::cerr << "       LED1 activation FAILED ✗" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(on_duration));

        std::cout << "       Turning OFF LED1..." << std::endl;
        as1170->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(off_duration));

        // Test LED2 (VCSEL 2 - Lower)
        std::cout << "[" << cycle << ".2] Activating LED2 (VCSEL2 - Lower)..." << std::endl;
        if (as1170->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, true, test_current)) {
            std::cout << "       LED2 ON at " << test_current << " mA ✓" << std::endl;
        } else {
            std::cerr << "       LED2 activation FAILED ✗" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(on_duration));

        std::cout << "       Turning OFF LED2..." << std::endl;
        as1170->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, false, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(off_duration));

        // Test BOTH (Dual VCSEL)
        std::cout << "[" << cycle << ".3] Activating BOTH VCSELs..." << std::endl;
        if (as1170->setLEDState(hardware::AS1170Controller::LEDChannel::BOTH, true, test_current)) {
            std::cout << "       BOTH VCSELs ON at " << test_current << " mA each ✓" << std::endl;
        } else {
            std::cerr << "       BOTH VCSELs activation FAILED ✗" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(on_duration));

        std::cout << "       Turning OFF BOTH..." << std::endl;
        as1170->setLEDState(hardware::AS1170Controller::LEDChannel::BOTH, false, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(off_duration));

        // Get status after cycle
        status = as1170->getStatus();
        std::cout << "\nStatus after cycle " << cycle << ":" << std::endl;
        std::cout << "  Temperature: " << std::fixed << std::setprecision(1)
                  << status.temperature_c << "°C" << std::endl;
        std::cout << "  LED1: " << status.led1_current_ma << " mA, "
                  << "LED2: " << status.led2_current_ma << " mA" << std::endl;
    }

    // Final status
    std::cout << "\n=== Test Complete ===" << std::endl;
    printStatus(as1170->getStatus());

    std::cout << "\n=== Diagnostics Summary ===" << std::endl;
    std::cout << "• If LED1 works but LED2 doesn't:" << std::endl;
    std::cout << "  - Check VCSEL2 connection to LED2 pins" << std::endl;
    std::cout << "  - Verify LED2 current limit in AS1170 config" << std::endl;
    std::cout << "  - Ensure LED2 is configured for TORCH mode" << std::endl;
    std::cout << "\n• If neither works:" << std::endl;
    std::cout << "  - Check I2C connection (bus 1, address 0x30)" << std::endl;
    std::cout << "  - Verify GPIO 19 for strobe control" << std::endl;
    std::cout << "  - Check power supply to AS1170" << std::endl;
    std::cout << "\n• If both work:" << std::endl;
    std::cout << "  - VCSELs are properly connected and configured! ✓" << std::endl;

    as1170->shutdown();
    return 0;
}