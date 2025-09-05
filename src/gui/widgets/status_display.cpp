#include "unlook/gui/widgets/status_display.hpp"
#include "unlook/gui/styles/supernova_style.hpp"
#include <QDateTime>
#include <QMouseEvent>
#include <QPainter>
#include <QProgressBar>
#include <QSpacerItem>

using namespace unlook::gui::styles;

namespace unlook {
namespace gui {
namespace widgets {

StatusDisplay::StatusDisplay(const QString& title, QWidget* parent)
    : QWidget(parent)
    , current_status_type_(StatusType::INFO)
    , compact_mode_(false)
    , animation_enabled_(true)
    , show_timestamp_(false)
    , status_animation_(nullptr)
    , pulse_timer_(nullptr)
    , temporary_message_timer_(nullptr)
    , pulse_opacity_(1.0)
    , pulse_direction_up_(false)
{
    initializeWidget();
    setTitle(title);
}

void StatusDisplay::setStatus(const QString& message, StatusType type) {
    StatusType old_type = current_status_type_;
    current_status_type_ = type;
    
    status_label_->setText(message);
    
    if (show_timestamp_) {
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
        timestamp_label_->setVisible(true);
    }
    
    updateStyling(type);
    
    if (animation_enabled_) {
        animateStatusChange(old_type, type);
    }
    
    emit statusChanged(message, type);
}

void StatusDisplay::setTitle(const QString& title) {
    title_label_->setText(title);
    title_label_->setVisible(!title.isEmpty());
}

void StatusDisplay::setProgress(int value, const QString& message) {
    progress_bar_->setValue(value);
    progress_bar_->setVisible(true);
    
    if (!message.isEmpty()) {
        status_label_->setText(message);
    }
}

void StatusDisplay::hideProgress() {
    progress_bar_->setVisible(false);
}

void StatusDisplay::setInfoText(const QString& info) {
    info_label_->setText(info);
    info_label_->setVisible(!info.isEmpty());
}

void StatusDisplay::clear() {
    status_label_->clear();
    info_label_->clear();
    timestamp_label_->clear();
    hideProgress();
    current_status_type_ = StatusType::INFO;
    updateStyling(StatusType::INFO);
}

void StatusDisplay::setCompactMode(bool compact) {
    compact_mode_ = compact;
    
    if (compact) {
        title_label_->setVisible(false);
        timestamp_label_->setVisible(false);
        info_label_->setVisible(false);
        setMaximumHeight(30);
    } else {
        title_label_->setVisible(!title_label_->text().isEmpty());
        setMaximumHeight(QWIDGETSIZE_MAX);
    }
}

void StatusDisplay::startPulsing() {
    if (!pulse_timer_) {
        pulse_timer_ = new QTimer(this);
        connect(pulse_timer_, &QTimer::timeout, this, &StatusDisplay::onPulseAnimationStep);
    }
    
    pulse_timer_->start(50); // 20 FPS pulsing
}

void StatusDisplay::stopPulsing() {
    if (pulse_timer_) {
        pulse_timer_->stop();
    }
    pulse_opacity_ = 1.0;
    update();
}

void StatusDisplay::updateStatus(const QString& message, StatusType type) {
    setStatus(message, type);
}

void StatusDisplay::showTemporaryMessage(const QString& message, StatusType type, int timeout_ms) {
    // Store current status
    previous_message_ = status_label_->text();
    previous_type_ = current_status_type_;
    
    // Show temporary message
    setStatus(message, type);
    
    // Setup timer to restore previous status
    if (!temporary_message_timer_) {
        temporary_message_timer_ = new QTimer(this);
        temporary_message_timer_->setSingleShot(true);
        connect(temporary_message_timer_, &QTimer::timeout, this, &StatusDisplay::onTemporaryMessageTimeout);
    }
    
    temporary_message_timer_->start(timeout_ms);
}

void StatusDisplay::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        emit clicked();
    }
    QWidget::mousePressEvent(event);
}

void StatusDisplay::paintEvent(QPaintEvent* event) {
    QWidget::paintEvent(event);
    
    // Custom painting for pulsing effect
    if (pulse_timer_ && pulse_timer_->isActive()) {
        QPainter painter(this);
        painter.setOpacity(pulse_opacity_);
        
        QColor status_color = getStatusColor(current_status_type_);
        painter.fillRect(rect(), status_color.darker(150));
    }
}

void StatusDisplay::onTemporaryMessageTimeout() {
    // Restore previous status
    setStatus(previous_message_, previous_type_);
}

void StatusDisplay::onPulseAnimationStep() {
    const double pulse_speed = 0.05;
    
    if (pulse_direction_up_) {
        pulse_opacity_ += pulse_speed;
        if (pulse_opacity_ >= 1.0) {
            pulse_opacity_ = 1.0;
            pulse_direction_up_ = false;
        }
    } else {
        pulse_opacity_ -= pulse_speed;
        if (pulse_opacity_ <= 0.3) {
            pulse_opacity_ = 0.3;
            pulse_direction_up_ = true;
        }
    }
    
    update();
}

void StatusDisplay::initializeWidget() {
    main_layout_ = new QVBoxLayout(this);
    main_layout_->setSpacing(SupernovaStyle::Spacing::PADDING_SMALL);
    main_layout_->setContentsMargins(SupernovaStyle::Spacing::PADDING_SMALL,
                                    SupernovaStyle::Spacing::PADDING_SMALL,
                                    SupernovaStyle::Spacing::PADDING_SMALL,
                                    SupernovaStyle::Spacing::PADDING_SMALL);
    
    // Header layout
    header_layout_ = new QHBoxLayout();
    
    // Status indicator (colored circle)
    status_indicator_ = new QLabel("●", this);
    status_indicator_->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::BODY));
    header_layout_->addWidget(status_indicator_);
    
    // Title
    title_label_ = new QLabel(this);
    title_label_->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SMALL, SupernovaStyle::FontWeight::BOLD));
    title_label_->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_PRIMARY)));
    header_layout_->addWidget(title_label_);
    
    // Spacer
    header_layout_->addSpacerItem(new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum));
    
    // Timestamp
    timestamp_label_ = new QLabel(this);
    timestamp_label_->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SMALL));
    timestamp_label_->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_SECONDARY)));
    timestamp_label_->setVisible(false);
    header_layout_->addWidget(timestamp_label_);
    
    main_layout_->addLayout(header_layout_);
    
    // Status message
    status_label_ = new QLabel(this);
    status_label_->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::BODY));
    status_label_->setWordWrap(true);
    main_layout_->addWidget(status_label_);
    
    // Progress bar
    progress_bar_ = new QProgressBar(this);
    progress_bar_->setRange(0, 100);
    progress_bar_->setVisible(false);
    main_layout_->addWidget(progress_bar_);
    
    // Info text
    info_label_ = new QLabel(this);
    info_label_->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SMALL));
    info_label_->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_SECONDARY)));
    info_label_->setWordWrap(true);
    info_label_->setVisible(false);
    main_layout_->addWidget(info_label_);
    
    // Apply initial styling
    setStyleSheet(SupernovaStyle::getStatusDisplayStyle());
    updateStyling(StatusType::INFO);
}

void StatusDisplay::updateStyling(StatusType type) {
    QColor status_color = getStatusColor(type);
    
    status_indicator_->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(status_color)));
    status_label_->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_PRIMARY)));
    
    // Update border color based on status
    setStyleSheet(SupernovaStyle::getStatusDisplayStyle(SupernovaStyle::NEBULA_SURFACE) + 
                  QString(" QWidget { border-left: 4px solid %1; }").arg(SupernovaStyle::colorToString(status_color)));
}

QColor StatusDisplay::getStatusColor(StatusType type) const {
    switch (type) {
        case StatusType::INFO:
        case StatusType::PROCESSING:
            return SupernovaStyle::ELECTRIC_PRIMARY;
        case StatusType::SUCCESS:
            return SupernovaStyle::SUCCESS_STATE;
        case StatusType::WARNING:
            return SupernovaStyle::WARNING_STATE;
        case StatusType::ERROR:
            return SupernovaStyle::ERROR_STATE;
        default:
            return SupernovaStyle::ELECTRIC_PRIMARY;
    }
}

void StatusDisplay::animateStatusChange(StatusType old_type, StatusType new_type) {
    // Simple fade animation
    if (status_animation_) {
        status_animation_->stop();
        status_animation_->deleteLater();
    }
    
    status_animation_ = new QPropertyAnimation(this, "windowOpacity", this);
    status_animation_->setDuration(SupernovaStyle::Animation::FAST_DURATION_MS);
    status_animation_->setStartValue(0.7);
    status_animation_->setEndValue(1.0);
    status_animation_->start();
}

} // namespace widgets
} // namespace gui
} // namespace unlook

// moc include removed - handled by CMake