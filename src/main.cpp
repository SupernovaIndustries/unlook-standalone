#include <QApplication>
#include <QStyleFactory>
#include <QDir>
#include <QDebug>
#include <QMessageBox>

#include "unlook/gui/main_window.hpp"
#include "unlook/gui/styles/supernova_style.hpp"

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
    
    try {
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
        
        std::cout << "Main window created, starting fullscreen..." << std::endl;
        
        // Show the main window
        main_window.show();
        
        std::cout << "Application started successfully" << std::endl;
        std::cout << "Press ESC to toggle fullscreen/windowed mode" << std::endl;
        
        // Run the application event loop
        int result = app.exec();
        
        std::cout << "Application shutting down..." << std::endl;
        return result;
        
    } catch (const std::exception& e) {
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
        return 1;
        
    } catch (...) {
        std::cerr << "FATAL ERROR: Unknown exception occurred" << std::endl;
        
        QMessageBox error_dialog;
        error_dialog.setIcon(QMessageBox::Critical);
        error_dialog.setWindowTitle("Fatal Error");
        error_dialog.setText("An unknown critical error occurred during startup.");
        error_dialog.setStandardButtons(QMessageBox::Ok);
        
        // Apply styling to error dialog
        SupernovaStyle::applyStyle(&error_dialog);
        
        error_dialog.exec();
        return 2;
    }
}