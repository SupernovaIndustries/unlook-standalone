#pragma once

#include <QWidget>
#include <QLabel>
#include <QProgressBar>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QPropertyAnimation>

namespace unlook {
namespace gui {
namespace widgets {

/**
 * @brief Industrial-grade status display with animated indicators
 * 
 * Features:
 * - Real-time status updates
 * - Color-coded state indicators
 * - Progress bar support
 * - Animated status changes
 * - Touch-friendly layout
 */
class StatusDisplay : public QWidget {
    Q_OBJECT
    
public:
    enum class StatusType {
        INFO,       // Information status (blue/teal)
        SUCCESS,    // Success status (green)
        WARNING,    // Warning status (orange)
        ERROR,      // Error status (red)
        PROCESSING  // Processing status (animated)
    };
    
    /**
     * @brief Constructor
     * @param title Status title
     * @param parent Parent widget
     */
    explicit StatusDisplay(const QString& title = "", QWidget* parent = nullptr);
    
    /**
     * @brief Set status message with type
     */
    void setStatus(const QString& message, StatusType type = StatusType::INFO);
    
    /**
     * @brief Set status title
     */
    void setTitle(const QString& title);
    
    /**
     * @brief Show progress bar with value (0-100)
     */
    void setProgress(int value, const QString& message = "");
    
    /**
     * @brief Hide progress bar
     */
    void hideProgress();
    
    /**
     * @brief Set additional info text
     */
    void setInfoText(const QString& info);
    
    /**
     * @brief Clear all status information
     */
    void clear();
    
    /**
     * @brief Set compact mode (smaller layout)
     */
    void setCompactMode(bool compact);
    
    /**
     * @brief Enable/disable status animations
     */
    void setAnimationEnabled(bool enabled) { animation_enabled_ = enabled; }
    
    /**
     * @brief Start pulsing animation (for processing states)
     */
    void startPulsing();
    
    /**
     * @brief Stop pulsing animation
     */
    void stopPulsing();

public slots:
    /**
     * @brief Update status with timestamp
     */
    void updateStatus(const QString& message, StatusType type = StatusType::INFO);
    
    /**
     * @brief Show temporary message (auto-hide after timeout)
     */
    void showTemporaryMessage(const QString& message, 
                             StatusType type = StatusType::INFO, 
                             int timeout_ms = 3000);

signals:
    /**
     * @brief Emitted when status changes
     */
    void statusChanged(const QString& message, StatusType type);
    
    /**
     * @brief Emitted when clicked (for interactive status displays)
     */
    void clicked();

protected:
    /**
     * @brief Override mouse press event for click detection
     */
    void mousePressEvent(QMouseEvent* event) override;
    
    /**
     * @brief Override paint event for custom styling
     */
    void paintEvent(QPaintEvent* event) override;

private slots:
    /**
     * @brief Handle temporary message timeout
     */
    void onTemporaryMessageTimeout();
    
    /**
     * @brief Handle pulse animation step
     */
    void onPulseAnimationStep();

private:
    /**
     * @brief Initialize the widget layout and styling
     */
    void initializeWidget();
    
    /**
     * @brief Update styling based on current status type
     */
    void updateStyling(StatusType type);
    
    /**
     * @brief Get color for status type
     */
    QColor getStatusColor(StatusType type) const;
    
    /**
     * @brief Animate status change
     */
    void animateStatusChange(StatusType old_type, StatusType new_type);
    
    // UI Components
    QVBoxLayout* main_layout_;
    QHBoxLayout* header_layout_;
    QLabel* title_label_;
    QLabel* status_label_;
    QLabel* info_label_;
    QLabel* timestamp_label_;
    QProgressBar* progress_bar_;
    QLabel* status_indicator_; // Colored indicator circle
    
    // State
    StatusType current_status_type_;
    bool compact_mode_;
    bool animation_enabled_;
    bool show_timestamp_;
    
    // Animation
    QPropertyAnimation* status_animation_;
    QTimer* pulse_timer_;
    QTimer* temporary_message_timer_;
    double pulse_opacity_;
    bool pulse_direction_up_;
    
    // Previous status for restore after temporary messages
    QString previous_message_;
    StatusType previous_type_;
};

} // namespace widgets
} // namespace gui
} // namespace unlook