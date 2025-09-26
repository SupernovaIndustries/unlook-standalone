/*
 * CRITICAL TEST: Validate hardware sync fix for demo
 * This test simulates the GUI capture behavior to verify sync is working
 */

#include <unlook/camera/CameraSystem.hpp>
#include <unlook/core/Logger.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>

using namespace unlook;

int main() {
    std::cout << "=== HARDWARE SYNC FIX VALIDATION TEST ===" << std::endl;
    std::cout << "Testing the fix for sync failures in captureSingle()" << std::endl;
    std::cout << "Expected: <1ms sync error consistently" << std::endl << std::endl;

    // Initialize camera system
    auto camera_system = camera::CameraSystem::getInstance();

    camera::CameraConfig config;
    config.width = 1456;
    config.height = 1088;
    config.enableSync = true;
    config.autoExposure = false;  // Disabled for consistent timing

    std::cout << "Initializing camera system..." << std::endl;
    if (!camera_system->initialize(config)) {
        std::cerr << "Failed to initialize camera system!" << std::endl;
        return 1;
    }

    std::cout << "Camera system initialized successfully" << std::endl << std::endl;

    // Simulate GUI behavior: multiple captureSingle calls
    std::cout << "Testing multiple captureSingle() calls (simulating GUI behavior):" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;

    int success_count = 0;
    int total_tests = 10;
    double total_sync_error = 0.0;
    double max_sync_error = 0.0;

    for (int i = 1; i <= total_tests; i++) {
        std::cout << "Test " << i << "/" << total_tests << ": ";

        // Call captureSingle like the GUI does
        auto frame_pair = camera_system->captureSingle();

        if (frame_pair.synchronized) {
            std::cout << "✓ SYNCED - Error: " << std::fixed << std::setprecision(3)
                     << frame_pair.sync_error_ms << "ms";

            // Check if sync error is within spec
            if (frame_pair.sync_error_ms < 1.0) {
                std::cout << " [PASS]";
                success_count++;
            } else if (frame_pair.sync_error_ms < 5.0) {
                std::cout << " [MARGINAL]";
            } else {
                std::cout << " [FAIL - Too high!]";
            }

            total_sync_error += frame_pair.sync_error_ms;
            max_sync_error = std::max(max_sync_error, frame_pair.sync_error_ms);

        } else {
            std::cout << "✗ NOT SYNCED - Error: " << frame_pair.sync_error_ms << "ms [FAIL]";
        }

        std::cout << std::endl;

        // Wait 2 seconds between captures (simulating user interaction)
        if (i < total_tests) {
            std::cout << "  Waiting 2s before next capture..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "TEST RESULTS:" << std::endl;
    std::cout << "  Success Rate: " << success_count << "/" << total_tests
              << " (" << (success_count * 100 / total_tests) << "%)" << std::endl;

    if (success_count > 0) {
        double avg_sync_error = total_sync_error / total_tests;
        std::cout << "  Average Sync Error: " << std::fixed << std::setprecision(3)
                  << avg_sync_error << "ms" << std::endl;
        std::cout << "  Maximum Sync Error: " << max_sync_error << "ms" << std::endl;

        if (avg_sync_error < 1.0 && max_sync_error < 2.0) {
            std::cout << std::endl << "✓✓✓ HARDWARE SYNC FIX SUCCESSFUL! ✓✓✓" << std::endl;
            std::cout << "The system now maintains <1ms sync consistently!" << std::endl;
        } else if (avg_sync_error < 2.0) {
            std::cout << std::endl << "⚠ PARTIAL SUCCESS: Sync improved but not optimal" << std::endl;
        } else {
            std::cout << std::endl << "✗ SYNC STILL NEEDS IMPROVEMENT" << std::endl;
        }
    } else {
        std::cout << std::endl << "✗✗✗ CRITICAL: NO SUCCESSFUL SYNCS! ✗✗✗" << std::endl;
    }

    // Test continuous mode (keeping capture running)
    std::cout << std::endl;
    std::cout << "Testing continuous capture mode (NEW optimization):" << std::endl;
    std::cout << "---------------------------------------------------" << std::endl;

    // First capture starts the system
    std::cout << "First capture (starts system)..." << std::endl;
    auto first = camera_system->captureSingle();
    std::cout << "  Sync: " << (first.synchronized ? "YES" : "NO")
              << ", Error: " << first.sync_error_ms << "ms" << std::endl;

    // Quick successive captures should have perfect sync
    std::cout << "Quick successive captures (system stays running):" << std::endl;
    for (int i = 1; i <= 5; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        auto frame = camera_system->captureSingle();
        std::cout << "  " << i << ". Sync: " << (frame.synchronized ? "YES" : "NO")
                  << ", Error: " << std::fixed << std::setprecision(3)
                  << frame.sync_error_ms << "ms";
        if (frame.sync_error_ms < 0.1) {
            std::cout << " [EXCELLENT!]";
        } else if (frame.sync_error_ms < 1.0) {
            std::cout << " [GOOD]";
        }
        std::cout << std::endl;
    }

    std::cout << std::endl << "Test complete. Shutting down..." << std::endl;
    camera_system->shutdown();

    return (success_count >= 8) ? 0 : 1;  // Return 0 if 80% success rate
}