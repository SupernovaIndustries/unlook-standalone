/**
 * @file swipe_gesture_detector.hpp
 * @brief Swipe gesture detection system for smartphone-like navigation
 *
 * Provides intuitive touch gestures for navigation without physical back buttons:
 * - Swipe right (from left edge): Navigate back
 * - Customizable gesture zones and sensitivity
 * - Visual feedback during gesture execution
 *
 * @author Unlook Team
 * @date 2025-11-05
 */

#ifndef UNLOOK_GUI_SWIPE_GESTURE_DETECTOR_HPP
#define UNLOOK_GUI_SWIPE_GESTURE_DETECTOR_HPP

#include <QObject>
#include <QWidget>
#include <QTouchEvent>
#include <QMouseEvent>
#include <QPoint>
#include <QTimer>
#include <functional>
#include <memory>

namespace unlook {
namespace gui {

/**
 * @brief Edge-based swipe gesture detector for touch-friendly navigation
 *
 * This class implements smartphone-like swipe gestures for navigation:
 * - Detects swipes starting from screen edges
 * - Provides visual feedback during gesture
 * - Configurable sensitivity and dead zones
 * - Supports both touch and mouse events (for development)
 *
 * Usage:
 * @code
 * auto* detector = new SwipeGestureDetector(widget);
 * detector->setSwipeBackCallback([this]() {
 *     navigateBack();
 * });
 * detector->setEnabled(true);
 * @endcode
 */
class SwipeGestureDetector : public QObject {
    Q_OBJECT

public:
    /**
     * @brief Swipe direction enumeration
     */
    enum class SwipeDirection {
        None,
        Right,      ///< Swipe from left edge to right (back navigation)
        Left,       ///< Swipe from right edge to left
        Up,         ///< Swipe from bottom edge to top
        Down        ///< Swipe from top edge to bottom
    };

    /**
     * @brief Gesture state for visual feedback
     */
    enum class GestureState {
        Idle,           ///< No gesture in progress
        Started,        ///< Gesture started, still within dead zone
        Active,         ///< Gesture is active and being tracked
        Completed,      ///< Gesture completed successfully
        Cancelled       ///< Gesture cancelled (moved wrong direction, etc.)
    };

    /**
     * @brief Constructor
     * @param parent Parent widget to monitor for gestures
     */
    explicit SwipeGestureDetector(QWidget* parent = nullptr);

    /**
     * @brief Destructor
     */
    ~SwipeGestureDetector() override = default;

    // Configuration methods

    /**
     * @brief Enable or disable gesture detection
     * @param enabled True to enable, false to disable
     */
    void setEnabled(bool enabled);

    /**
     * @brief Check if gesture detection is enabled
     * @return True if enabled
     */
    bool isEnabled() const { return m_enabled; }

    /**
     * @brief Set edge zone width for gesture start detection
     * @param width Width in pixels from screen edge (default: 30px)
     */
    void setEdgeZoneWidth(int width);

    /**
     * @brief Get current edge zone width
     * @return Edge zone width in pixels
     */
    int edgeZoneWidth() const { return m_edgeZoneWidth; }

    /**
     * @brief Set minimum swipe distance to trigger gesture
     * @param distance Minimum distance in pixels (default: 80px)
     */
    void setMinimumSwipeDistance(int distance);

    /**
     * @brief Get minimum swipe distance
     * @return Minimum swipe distance in pixels
     */
    int minimumSwipeDistance() const { return m_minSwipeDistance; }

    /**
     * @brief Set maximum vertical deviation for horizontal swipe
     * @param deviation Maximum vertical movement in pixels (default: 50px)
     */
    void setMaxVerticalDeviation(int deviation);

    /**
     * @brief Get maximum vertical deviation
     * @return Maximum vertical deviation in pixels
     */
    int maxVerticalDeviation() const { return m_maxVerticalDeviation; }

    /**
     * @brief Enable visual feedback overlay during gesture
     * @param enabled True to show visual feedback
     */
    void setVisualFeedbackEnabled(bool enabled);

    /**
     * @brief Check if visual feedback is enabled
     * @return True if enabled
     */
    bool isVisualFeedbackEnabled() const { return m_visualFeedback; }

    /**
     * @brief Set callback for swipe-back gesture (right swipe from left edge)
     * @param callback Function to call when swipe back is completed
     */
    void setSwipeBackCallback(std::function<void()> callback);

    /**
     * @brief Set callback for any swipe direction
     * @param callback Function to call with swipe direction when completed
     */
    void setSwipeCallback(std::function<void(SwipeDirection)> callback);

    /**
     * @brief Get current gesture state
     * @return Current gesture state
     */
    GestureState gestureState() const { return m_gestureState; }

signals:
    /**
     * @brief Emitted when swipe back gesture is detected
     */
    void swipeBackDetected();

    /**
     * @brief Emitted when any swipe is detected
     * @param direction Direction of the swipe
     */
    void swipeDetected(SwipeDirection direction);

    /**
     * @brief Emitted when gesture state changes (for visual feedback)
     * @param state New gesture state
     * @param progress Progress of gesture (0.0 to 1.0)
     */
    void gestureStateChanged(GestureState state, float progress);

protected:
    /**
     * @brief Event filter to intercept touch and mouse events
     * @param obj Object that received the event
     * @param event Event to filter
     * @return True if event was handled
     */
    bool eventFilter(QObject* obj, QEvent* event) override;

private:
    // Event handlers
    void handleTouchBegin(const QTouchEvent* event);
    void handleTouchUpdate(const QTouchEvent* event);
    void handleTouchEnd(const QTouchEvent* event);
    void handleMousePress(const QMouseEvent* event);
    void handleMouseMove(const QMouseEvent* event);
    void handleMouseRelease(const QMouseEvent* event);

    // Gesture detection logic
    bool isPointInEdgeZone(const QPoint& point) const;
    SwipeDirection calculateSwipeDirection(const QPoint& start, const QPoint& end) const;
    bool isValidSwipe(const QPoint& start, const QPoint& end, SwipeDirection direction) const;
    float calculateGestureProgress(const QPoint& current) const;
    void updateGestureState(GestureState newState, float progress = 0.0f);
    void completeGesture();
    void cancelGesture();
    void resetGesture();

    // Configuration
    bool m_enabled{false};
    int m_edgeZoneWidth{80};          ///< Width of edge zone in pixels (more permissive)
    int m_minSwipeDistance{50};        ///< Minimum swipe distance to trigger (easier to trigger)
    int m_maxVerticalDeviation{120};    ///< Maximum vertical movement for horizontal swipe (more tolerance)
    bool m_visualFeedback{true};       ///< Show visual feedback overlay

    // State tracking
    GestureState m_gestureState{GestureState::Idle};
    QPoint m_gestureStartPoint;        ///< Point where gesture started
    QPoint m_gestureCurrentPoint;      ///< Current point during gesture
    bool m_gestureActive{false};       ///< Is a gesture currently being tracked
    bool m_touchActive{false};         ///< Is touch/mouse currently active
    qint64 m_gestureStartTime{0};      ///< Timestamp when gesture started

    // Callbacks
    std::function<void()> m_swipeBackCallback;
    std::function<void(SwipeDirection)> m_swipeCallback;

    // Parent widget
    QWidget* m_parentWidget{nullptr};
};

} // namespace gui
} // namespace unlook

#endif // UNLOOK_GUI_SWIPE_GESTURE_DETECTOR_HPP
