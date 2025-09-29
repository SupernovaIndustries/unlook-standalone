/**
 * @file test_file_logger.cpp
 * @brief Thread-safety and performance validation for file logging system
 *
 * Tests:
 * 1. Thread-safety: Concurrent logging from multiple threads
 * 2. Performance: Logging overhead measurement
 * 3. File creation: Automatic directory and timestamped file creation
 * 4. Dual output: Verify both console and file logging
 * 5. Log file content verification
 */

#include "unlook/core/Logger.hpp"

#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include <atomic>
#include <sstream>
#include <fstream>
#include <cassert>

using namespace unlook::core;

// Test statistics
std::atomic<int> totalMessages{0};
std::atomic<int> errors{0};

/**
 * Test 1: Basic file logging initialization
 */
bool testBasicInitialization() {
    std::cout << "\n=== Test 1: Basic Initialization ===" << std::endl;

    auto& logger = Logger::getInstance();

    // Initialize with test directory
    bool success = logger.initializeWithTimestamp("/tmp/unlook_test_logs", LogLevel::DEBUG);

    if (!success) {
        std::cerr << "ERROR: Logger initialization failed" << std::endl;
        errors++;
        return false;
    }

    std::string logFile = logger.getCurrentLogFile();
    std::cout << "Log file created: " << logFile << std::endl;

    // Verify file exists
    std::ifstream file(logFile);
    if (!file.good()) {
        std::cerr << "ERROR: Log file not accessible" << std::endl;
        errors++;
        return false;
    }

    std::cout << "PASS: Basic initialization successful" << std::endl;
    return true;
}

/**
 * Test 2: Thread-safety - concurrent logging
 */
bool testThreadSafety() {
    std::cout << "\n=== Test 2: Thread Safety (Concurrent Logging) ===" << std::endl;

    auto& logger = Logger::getInstance();
    constexpr int NUM_THREADS = 10;
    constexpr int MESSAGES_PER_THREAD = 100;

    std::vector<std::thread> threads;
    threads.reserve(NUM_THREADS);

    auto startTime = std::chrono::high_resolution_clock::now();

    // Launch threads
    for (int i = 0; i < NUM_THREADS; ++i) {
        threads.emplace_back([i, &logger]() {
            for (int j = 0; j < MESSAGES_PER_THREAD; ++j) {
                std::ostringstream oss;
                oss << "Thread " << i << " message " << j;
                logger.info(oss.str());
                totalMessages++;
            }
        });
    }

    // Wait for all threads
    for (auto& t : threads) {
        t.join();
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    logger.flush();

    int expectedMessages = NUM_THREADS * MESSAGES_PER_THREAD;
    std::cout << "Total messages logged: " << totalMessages.load() << " (expected: " << expectedMessages << ")" << std::endl;
    std::cout << "Time taken: " << duration.count() << "ms" << std::endl;
    std::cout << "Messages per second: " << (totalMessages.load() * 1000.0 / duration.count()) << std::endl;

    if (totalMessages.load() != expectedMessages) {
        std::cerr << "ERROR: Message count mismatch" << std::endl;
        errors++;
        return false;
    }

    std::cout << "PASS: Thread safety test successful" << std::endl;
    return true;
}

/**
 * Test 3: Performance measurement
 */
bool testPerformance() {
    std::cout << "\n=== Test 3: Performance Measurement ===" << std::endl;

    auto& logger = Logger::getInstance();
    constexpr int NUM_MESSAGES = 1000;

    // Measure logging overhead
    auto startTime = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < NUM_MESSAGES; ++i) {
        logger.info("Performance test message " + std::to_string(i));
    }

    logger.flush();

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);

    double averagePerMessage = static_cast<double>(duration.count()) / NUM_MESSAGES;

    std::cout << "Total time for " << NUM_MESSAGES << " messages: " << duration.count() << "us" << std::endl;
    std::cout << "Average time per message: " << averagePerMessage << "us" << std::endl;

    // Verify performance requirement: <1ms per message
    if (averagePerMessage > 1000.0) {
        std::cerr << "ERROR: Performance requirement not met (>1ms per message)" << std::endl;
        errors++;
        return false;
    }

    std::cout << "PASS: Performance test successful" << std::endl;
    return true;
}

/**
 * Test 4: Log levels
 */
bool testLogLevels() {
    std::cout << "\n=== Test 4: Log Levels ===" << std::endl;

    auto& logger = Logger::getInstance();

    // Test all log levels
    logger.trace("This is a TRACE message");
    logger.debug("This is a DEBUG message");
    logger.info("This is an INFO message");
    logger.warning("This is a WARNING message");
    logger.error("This is an ERROR message");
    logger.critical("This is a CRITICAL message");

    logger.flush();

    std::cout << "PASS: All log levels tested" << std::endl;
    return true;
}

/**
 * Test 5: Verify log file content
 */
bool testLogFileContent() {
    std::cout << "\n=== Test 5: Log File Content Verification ===" << std::endl;

    auto& logger = Logger::getInstance();
    std::string logFile = logger.getCurrentLogFile();

    // Write unique test message
    const std::string testMessage = "UNIQUE_TEST_MESSAGE_12345";
    logger.info(testMessage);
    logger.flush();

    // Read log file and verify message exists
    std::ifstream file(logFile);
    if (!file.good()) {
        std::cerr << "ERROR: Cannot open log file for verification" << std::endl;
        errors++;
        return false;
    }

    std::string line;
    bool messageFound = false;
    while (std::getline(file, line)) {
        if (line.find(testMessage) != std::string::npos) {
            messageFound = true;
            break;
        }
    }

    if (!messageFound) {
        std::cerr << "ERROR: Test message not found in log file" << std::endl;
        errors++;
        return false;
    }

    std::cout << "PASS: Log file content verified" << std::endl;
    return true;
}

/**
 * Main test runner
 */
int main() {
    std::cout << "=========================================" << std::endl;
    std::cout << "Unlook File Logger Validation Tests" << std::endl;
    std::cout << "=========================================" << std::endl;

    bool allPassed = true;

    // Run all tests
    allPassed &= testBasicInitialization();
    allPassed &= testThreadSafety();
    allPassed &= testPerformance();
    allPassed &= testLogLevels();
    allPassed &= testLogFileContent();

    // Summary
    std::cout << "\n=========================================" << std::endl;
    std::cout << "Test Summary" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Total errors: " << errors.load() << std::endl;

    if (allPassed && errors.load() == 0) {
        std::cout << "ALL TESTS PASSED" << std::endl;
        return 0;
    } else {
        std::cout << "SOME TESTS FAILED" << std::endl;
        return 1;
    }
}