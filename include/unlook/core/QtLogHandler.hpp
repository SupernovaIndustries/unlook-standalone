#pragma once

#include <QString>
#include <QMessageLogContext>

namespace unlook {
namespace core {

/**
 * Qt Message Handler for redirecting Qt logging to Unlook Logger
 *
 * This handler intercepts ALL Qt logging (qDebug, qWarning, qCritical, qInfo)
 * and redirects it to the Unlook file logging system while maintaining console output.
 *
 * Thread-safe: Qt guarantees message handler calls are synchronized
 */
class QtLogHandler {
public:
    /**
     * Install Qt message handler
     * Call this AFTER Logger::initializeWithTimestamp()
     */
    static void install();

    /**
     * Restore default Qt message handler
     */
    static void uninstall();

private:
    /**
     * Custom Qt message handler function
     * Captures all Qt logging and routes to Unlook Logger
     */
    static void messageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg);

    static QtMessageHandler previousHandler_;
};

} // namespace core
} // namespace unlook