#include <QApplication>
#include <QStyleFactory>
#include <QDir>
#include <QDebug>
#include <QMessageBox>

#include "unlook/gui/main_window.hpp"
#include "unlook/gui/styles/supernova_style.hpp"
#include "unlook/core/Logger.hpp"
#include "unlook/core/QtLogHandler.hpp"

#include <iostream>
#include <exception>

using namespace unlook::gui;
using namespace unlook::gui::styles;

int main(int argc, char *argv[])
{
    // Create Qt application
    QApplication app(argc, argv);

    // Set application properties
    app.setApplicationName("Unlook 3D Scanner");
    app.setApplicationVersion("1.0");
    app.setOrganizationName("Unlook Project");
    app.setOrganizationDomain("unlook-scanner.org");

    // Set application icon
    app.setWindowIcon(QIcon(":/icons/unlook_icon.png"));

    // ========================================
    // CRITICAL: Initialize file logging system FIRST
    // ========================================
    auto& logger = unlook::core::Logger::getInstance();

    // Initialize with timestamped log file in /home/unlook/log
    // Falls back to console-only if directory creation fails
    bool logInitSuccess = logger.initializeWithTimestamp("/home/unlook/log", unlook::core::LogLevel::DEBUG);

    if (logInitSuccess) {
        std::cout << "[LOGGING] File logging initialized: " << logger.getCurrentLogFile() << std::endl;
        logger.info("=== Unlook 3D Scanner File Logging Initialized ===");
        logger.info("Log file: " + logger.getCurrentLogFile());
    } else {
        std::cerr << "[LOGGING] Warning: File logging initialization failed, using console only" << std::endl;
    }

    // Install Qt message handler to capture qDebug/qWarning/qCritical
    unlook::core::QtLogHandler::install();
    logger.info("Qt message handler installed - all Qt logging redirected to file");

    try {
        // Log application startup
        logger.info("=== Unlook 3D Scanner v1.0 ===");
        logger.info("Professional Modular Open Source 3D Scanner");
        logger.info("Target Precision: 0.005mm");
        logger.info("Starting application...");

        // Maintain console output for visibility
        std::cout << "=== Unlook 3D Scanner v1.0 ===" << std::endl;
        std::cout << "Professional Modular Open Source 3D Scanner" << std::endl;
        std::cout << "Target Precision: 0.005mm" << std::endl;
        std::cout << "Starting application..." << std::endl;
        
        // Apply global Supernova style
        app.setStyleSheet(SupernovaStyle::getApplicationStyleSheet());
        
        // Create and show main window
        UnlookMainWindow main_window;
        
        // Apply Supernova styling to main window
        SupernovaStyle::applyStyle(&main_window);

        logger.info("Main window created, starting fullscreen...");
        std::cout << "Main window created, starting fullscreen..." << std::endl;

        // Show the main window
        main_window.show();

        logger.info("Application started successfully");
        logger.info("Press ESC to toggle fullscreen/windowed mode");
        std::cout << "Application started successfully" << std::endl;
        std::cout << "Press ESC to toggle fullscreen/windowed mode" << std::endl;

        // Run the application event loop
        int result = app.exec();

        // Shutdown logging system
        logger.info("=== Application Shutdown ===");
        logger.info("Exit code: " + std::to_string(result));
        logger.flush();

        std::cout << "Application shutting down..." << std::endl;

        // Uninstall Qt message handler
        unlook::core::QtLogHandler::uninstall();

        return result;
        
    } catch (const std::exception& e) {
        // Log the fatal error
        logger.critical("FATAL ERROR: " + std::string(e.what()));
        logger.flush();

        std::cerr << "FATAL ERROR: " << e.what() << std::endl;

        QMessageBox error_dialog;
        error_dialog.setIcon(QMessageBox::Critical);
        error_dialog.setWindowTitle("Fatal Error");
        error_dialog.setText("A critical error occurred during startup:");
        error_dialog.setDetailedText(QString::fromStdString(e.what()));
        error_dialog.setStandardButtons(QMessageBox::Ok);

        // Apply styling to error dialog
        SupernovaStyle::applyStyle(&error_dialog);

        error_dialog.exec();

        // Cleanup
        unlook::core::QtLogHandler::uninstall();
        return 1;

    } catch (...) {
        // Log unknown fatal error
        logger.critical("FATAL ERROR: Unknown exception occurred");
        logger.flush();

        std::cerr << "FATAL ERROR: Unknown exception occurred" << std::endl;

        QMessageBox error_dialog;
        error_dialog.setIcon(QMessageBox::Critical);
        error_dialog.setWindowTitle("Fatal Error");
        error_dialog.setText("An unknown critical error occurred during startup.");
        error_dialog.setStandardButtons(QMessageBox::Ok);

        // Apply styling to error dialog
        SupernovaStyle::applyStyle(&error_dialog);

        error_dialog.exec();

        // Cleanup
        unlook::core::QtLogHandler::uninstall();
        return 2;
    }
}