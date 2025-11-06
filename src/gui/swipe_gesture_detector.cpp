/**
 * @file swipe_gesture_detector.cpp
 * @brief Implementation of swipe gesture detection system
 */

#include "unlook/gui/swipe_gesture_detector.hpp"
#include <QTouchEvent>
#include <QMouseEvent>
#include <QDateTime>
#include <cmath>

namespace unlook {
namespace gui {

SwipeGestureDetector::SwipeGestureDetector(QWidget* parent)
    : QObject(parent), m_parentWidget(parent) {
    if (m_parentWidget) {
        m_parentWidget->installEventFilter(this);
        // Enable touch events on the parent widget
        m_parentWidget->setAttribute(Qt::WA_AcceptTouchEvents, true);
    }
}

void SwipeGestureDetector::setEnabled(bool enabled) {
    m_enabled = enabled;
    if (!enabled) {
        resetGesture();
    }
}

void SwipeGestureDetector::setEdgeZoneWidth(int width) {
    m_edgeZoneWidth = qMax(10, width);  // Minimum 10px
}

void SwipeGestureDetector::setMinimumSwipeDistance(int distance) {
    m_minSwipeDistance = qMax(20, distance);  // Minimum 20px
}

void SwipeGestureDetector::setMaxVerticalDeviation(int deviation) {
    m_maxVerticalDeviation = qMax(10, deviation);  // Minimum 10px
}

void SwipeGestureDetector::setVisualFeedbackEnabled(bool enabled) {
    m_visualFeedback = enabled;
}

void SwipeGestureDetector::setSwipeBackCallback(std::function<void()> callback) {
    m_swipeBackCallback = std::move(callback);
}

void SwipeGestureDetector::setSwipeCallback(std::function<void(SwipeDirection)> callback) {
    m_swipeCallback = std::move(callback);
}

bool SwipeGestureDetector::eventFilter(QObject* obj, QEvent* event) {
    if (!m_enabled || obj != m_parentWidget) {
        return QObject::eventFilter(obj, event);
    }

    switch (event->type()) {
        case QEvent::TouchBegin:
            handleTouchBegin(static_cast<QTouchEvent*>(event));
            return false;  // Don't consume the event

        case QEvent::TouchUpdate:
            handleTouchUpdate(static_cast<QTouchEvent*>(event));
            return m_gestureActive;  // Consume if gesture is active

        case QEvent::TouchEnd:
        case QEvent::TouchCancel:
            handleTouchEnd(static_cast<QTouchEvent*>(event));
            return m_gestureActive;  // Consume if gesture was active

        case QEvent::MouseButtonPress:
            // Support mouse for development/testing
            handleMousePress(static_cast<QMouseEvent*>(event));
            return false;

        case QEvent::MouseMove:
            handleMouseMove(static_cast<QMouseEvent*>(event));
            return m_gestureActive;

        case QEvent::MouseButtonRelease:
            handleMouseRelease(static_cast<QMouseEvent*>(event));
            return m_gestureActive;

        default:
            break;
    }

    return QObject::eventFilter(obj, event);
}

void SwipeGestureDetector::handleTouchBegin(const QTouchEvent* event) {
    if (event->touchPoints().isEmpty()) {
        return;
    }

    const QTouchEvent::TouchPoint& touchPoint = event->touchPoints().first();
    const QPoint pos = touchPoint.pos().toPoint();

    if (isPointInEdgeZone(pos)) {
        m_gestureStartPoint = pos;
        m_gestureCurrentPoint = pos;
        m_gestureActive = true;
        m_touchActive = true;
        m_gestureStartTime = QDateTime::currentMSecsSinceEpoch();
        updateGestureState(GestureState::Started, 0.0f);
    }
}

void SwipeGestureDetector::handleTouchUpdate(const QTouchEvent* event) {
    if (!m_gestureActive || event->touchPoints().isEmpty()) {
        return;
    }

    const QTouchEvent::TouchPoint& touchPoint = event->touchPoints().first();
    m_gestureCurrentPoint = touchPoint.pos().toPoint();

    // Calculate progress
    float progress = calculateGestureProgress(m_gestureCurrentPoint);

    // Check if gesture is still valid
    SwipeDirection direction = calculateSwipeDirection(m_gestureStartPoint, m_gestureCurrentPoint);

    if (direction != SwipeDirection::None &&
        isValidSwipe(m_gestureStartPoint, m_gestureCurrentPoint, direction)) {
        updateGestureState(GestureState::Active, progress);
    } else if (progress < 0.1f) {
        // Still in starting zone
        updateGestureState(GestureState::Started, progress);
    } else {
        // Invalid gesture (e.g., moved wrong direction)
        cancelGesture();
    }
}

void SwipeGestureDetector::handleTouchEnd(const QTouchEvent* event) {
    if (!m_gestureActive) {
        return;
    }

    m_touchActive = false;

    // Check if gesture should be completed
    SwipeDirection direction = calculateSwipeDirection(m_gestureStartPoint, m_gestureCurrentPoint);

    if (direction != SwipeDirection::None &&
        isValidSwipe(m_gestureStartPoint, m_gestureCurrentPoint, direction)) {

        float progress = calculateGestureProgress(m_gestureCurrentPoint);

        // Require at least 50% progress to complete gesture
        if (progress >= 0.5f) {
            updateGestureState(GestureState::Completed, 1.0f);
            completeGesture();
        } else {
            cancelGesture();
        }
    } else {
        cancelGesture();
    }

    resetGesture();
}

void SwipeGestureDetector::handleMousePress(const QMouseEvent* event) {
    if (event->button() != Qt::LeftButton) {
        return;
    }

    const QPoint pos = event->pos();

    if (isPointInEdgeZone(pos)) {
        m_gestureStartPoint = pos;
        m_gestureCurrentPoint = pos;
        m_gestureActive = true;
        m_touchActive = true;
        m_gestureStartTime = QDateTime::currentMSecsSinceEpoch();
        updateGestureState(GestureState::Started, 0.0f);
    }
}

void SwipeGestureDetector::handleMouseMove(const QMouseEvent* event) {
    if (!m_gestureActive || !m_touchActive) {
        return;
    }

    m_gestureCurrentPoint = event->pos();

    // Calculate progress
    float progress = calculateGestureProgress(m_gestureCurrentPoint);

    // Check if gesture is still valid
    SwipeDirection direction = calculateSwipeDirection(m_gestureStartPoint, m_gestureCurrentPoint);

    if (direction != SwipeDirection::None &&
        isValidSwipe(m_gestureStartPoint, m_gestureCurrentPoint, direction)) {
        updateGestureState(GestureState::Active, progress);
    } else if (progress < 0.1f) {
        updateGestureState(GestureState::Started, progress);
    } else {
        cancelGesture();
    }
}

void SwipeGestureDetector::handleMouseRelease(const QMouseEvent* event) {
    if (!m_gestureActive || event->button() != Qt::LeftButton) {
        return;
    }

    m_touchActive = false;

    // Check if gesture should be completed
    SwipeDirection direction = calculateSwipeDirection(m_gestureStartPoint, m_gestureCurrentPoint);

    if (direction != SwipeDirection::None &&
        isValidSwipe(m_gestureStartPoint, m_gestureCurrentPoint, direction)) {

        float progress = calculateGestureProgress(m_gestureCurrentPoint);

        if (progress >= 0.5f) {
            updateGestureState(GestureState::Completed, 1.0f);
            completeGesture();
        } else {
            cancelGesture();
        }
    } else {
        cancelGesture();
    }

    resetGesture();
}

bool SwipeGestureDetector::isPointInEdgeZone(const QPoint& point) const {
    if (!m_parentWidget) {
        return false;
    }

    // Left edge zone for swipe right (back navigation)
    return point.x() <= m_edgeZoneWidth;
}

SwipeGestureDetector::SwipeDirection SwipeGestureDetector::calculateSwipeDirection(
    const QPoint& start, const QPoint& end) const {

    const int dx = end.x() - start.x();
    const int dy = end.y() - start.y();

    const int absDx = std::abs(dx);
    const int absDy = std::abs(dy);

    // Must be primarily horizontal or vertical
    if (absDx < m_minSwipeDistance && absDy < m_minSwipeDistance) {
        return SwipeDirection::None;
    }

    // Horizontal swipe
    if (absDx > absDy) {
        if (dx > 0) {
            return SwipeDirection::Right;  // Swipe to right (back)
        } else {
            return SwipeDirection::Left;
        }
    }
    // Vertical swipe
    else {
        if (dy > 0) {
            return SwipeDirection::Down;
        } else {
            return SwipeDirection::Up;
        }
    }
}

bool SwipeGestureDetector::isValidSwipe(
    const QPoint& start, const QPoint& end, SwipeDirection direction) const {

    const int dx = end.x() - start.x();
    const int dy = end.y() - start.y();

    const int absDx = std::abs(dx);
    const int absDy = std::abs(dy);

    switch (direction) {
        case SwipeDirection::Right:
        case SwipeDirection::Left:
            // For horizontal swipe, vertical deviation must be small
            return absDy <= m_maxVerticalDeviation && dx > 0;  // Only right swipes for back

        case SwipeDirection::Up:
        case SwipeDirection::Down:
            // For vertical swipe, horizontal deviation must be small
            return absDx <= m_maxVerticalDeviation;

        default:
            return false;
    }
}

float SwipeGestureDetector::calculateGestureProgress(const QPoint& current) const {
    if (!m_parentWidget) {
        return 0.0f;
    }

    const int dx = current.x() - m_gestureStartPoint.x();

    // Progress based on how far across the screen we've swiped
    // Full progress at 1/3 of screen width
    const float fullProgressDistance = m_parentWidget->width() / 3.0f;

    float progress = static_cast<float>(dx) / fullProgressDistance;
    return qBound(0.0f, progress, 1.0f);
}

void SwipeGestureDetector::updateGestureState(GestureState newState, float progress) {
    if (m_gestureState != newState) {
        m_gestureState = newState;

        if (m_visualFeedback) {
            emit gestureStateChanged(newState, progress);
        }
    }
}

void SwipeGestureDetector::completeGesture() {
    SwipeDirection direction = calculateSwipeDirection(m_gestureStartPoint, m_gestureCurrentPoint);

    // Emit general swipe signal
    emit swipeDetected(direction);

    if (m_swipeCallback) {
        m_swipeCallback(direction);
    }

    // Emit specific back gesture signal
    if (direction == SwipeDirection::Right) {
        emit swipeBackDetected();

        if (m_swipeBackCallback) {
            m_swipeBackCallback();
        }
    }
}

void SwipeGestureDetector::cancelGesture() {
    updateGestureState(GestureState::Cancelled, 0.0f);
}

void SwipeGestureDetector::resetGesture() {
    m_gestureActive = false;
    m_touchActive = false;
    m_gestureStartPoint = QPoint();
    m_gestureCurrentPoint = QPoint();
    m_gestureStartTime = 0;
    updateGestureState(GestureState::Idle, 0.0f);
}

} // namespace gui
} // namespace unlook
