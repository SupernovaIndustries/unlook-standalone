#pragma once

#include "unlook/face/FaceTypes.hpp"
#include "unlook/core/exception.h"
#include <stdexcept>
#include <string>
#include <vector>
#include <chrono>
#include <map>

namespace unlook {
namespace face {

/**
 * @brief Base exception class for facial recognition operations
 *
 * Comprehensive exception handling system for banking-grade facial
 * recognition with detailed error context and recovery suggestions.
 */
class FaceException : public core::UnlookException {
public:
    /**
     * @brief Constructor with error code and message
     * @param code Face-specific error code
     * @param message Human-readable error message
     */
    explicit FaceException(FaceResultCode code, const std::string& message = "");

    /**
     * @brief Constructor with error code, message, and context
     * @param code Face-specific error code
     * @param message Human-readable error message
     * @param context Additional error context
     */
    FaceException(FaceResultCode code,
                 const std::string& message,
                 const std::map<std::string, std::string>& context);

    /**
     * @brief Virtual destructor
     */
    virtual ~FaceException() noexcept = default;

    /**
     * @brief Get face-specific error code
     * @return Face result code
     */
    FaceResultCode getFaceErrorCode() const noexcept { return face_error_code_; }

    /**
     * @brief Get error context information
     * @return Error context map
     */
    const std::map<std::string, std::string>& getContext() const noexcept { return context_; }

    /**
     * @brief Get recovery suggestions
     * @return Vector of recovery suggestions
     */
    const std::vector<std::string>& getRecoverySuggestions() const noexcept { return recovery_suggestions_; }

    /**
     * @brief Get error severity level
     * @return Severity level string
     */
    const std::string& getSeverity() const noexcept { return severity_; }

    /**
     * @brief Check if error is recoverable
     * @return True if error is potentially recoverable
     */
    bool isRecoverable() const noexcept { return is_recoverable_; }

    /**
     * @brief Get error timestamp
     * @return Timestamp when error occurred
     */
    std::chrono::system_clock::time_point getTimestamp() const noexcept { return timestamp_; }

    /**
     * @brief Add additional context information
     * @param key Context key
     * @param value Context value
     */
    void addContext(const std::string& key, const std::string& value);

    /**
     * @brief Add recovery suggestion
     * @param suggestion Recovery suggestion text
     */
    void addRecoverySuggestion(const std::string& suggestion);

protected:
    FaceResultCode face_error_code_;
    std::map<std::string, std::string> context_;
    std::vector<std::string> recovery_suggestions_;
    std::string severity_;
    bool is_recoverable_;
    std::chrono::system_clock::time_point timestamp_;

private:
    void initializeFromErrorCode(FaceResultCode code);
};

/**
 * @brief Face detection specific exception
 */
class FaceDetectionException : public FaceException {
public:
    explicit FaceDetectionException(FaceResultCode code, const std::string& message = "");

    FaceDetectionException(FaceResultCode code,
                          const std::string& message,
                          const cv::Size& image_size,
                          int num_faces_detected = 0);
};

/**
 * @brief Landmark extraction specific exception
 */
class LandmarkExtractionException : public FaceException {
public:
    explicit LandmarkExtractionException(FaceResultCode code, const std::string& message = "");

    LandmarkExtractionException(FaceResultCode code,
                               const std::string& message,
                               int landmarks_detected,
                               int landmarks_expected,
                               float confidence_achieved);
};

/**
 * @brief 3D reconstruction specific exception
 */
class Face3DReconstructionException : public FaceException {
public:
    explicit Face3DReconstructionException(FaceResultCode code, const std::string& message = "");

    Face3DReconstructionException(FaceResultCode code,
                                 const std::string& message,
                                 float reconstruction_quality,
                                 float completeness_achieved);
};

/**
 * @brief Template encoding specific exception
 */
class TemplateEncodingException : public FaceException {
public:
    explicit TemplateEncodingException(FaceResultCode code, const std::string& message = "");

    TemplateEncodingException(FaceResultCode code,
                             const std::string& message,
                             size_t features_extracted,
                             float encoding_quality);
};

/**
 * @brief Face matching specific exception
 */
class FaceMatchingException : public FaceException {
public:
    explicit FaceMatchingException(FaceResultCode code, const std::string& message = "");

    FaceMatchingException(FaceResultCode code,
                         const std::string& message,
                         float similarity_achieved,
                         float threshold_required);
};

/**
 * @brief Liveness detection specific exception
 */
class LivenessDetectionException : public FaceException {
public:
    explicit LivenessDetectionException(FaceResultCode code, const std::string& message = "");

    LivenessDetectionException(FaceResultCode code,
                              const std::string& message,
                              float liveness_score,
                              const std::string& attack_type_detected = "");
};

/**
 * @brief Enrollment specific exception
 */
class EnrollmentException : public FaceException {
public:
    explicit EnrollmentException(FaceResultCode code, const std::string& message = "");

    EnrollmentException(FaceResultCode code,
                       const std::string& message,
                       EnrollmentState current_state,
                       int samples_collected,
                       float quality_achieved);
};

/**
 * @brief Supernova ML integration specific exception
 */
class SupernovaMLException : public FaceException {
public:
    explicit SupernovaMLException(FaceResultCode code, const std::string& message = "");

    SupernovaMLException(FaceResultCode code,
                        const std::string& message,
                        int http_status_code,
                        const std::string& api_response = "");
};

/**
 * @brief Banking compliance specific exception
 */
class BankingComplianceException : public FaceException {
public:
    explicit BankingComplianceException(FaceResultCode code, const std::string& message = "");

    BankingComplianceException(FaceResultCode code,
                              const std::string& message,
                              float far_achieved,
                              float frr_achieved,
                              const std::string& compliance_violation);
};

/**
 * @brief Security and encryption specific exception
 */
class SecurityException : public FaceException {
public:
    explicit SecurityException(FaceResultCode code, const std::string& message = "");

    SecurityException(FaceResultCode code,
                     const std::string& message,
                     const std::string& security_context,
                     const std::string& threat_type = "");
};

/**
 * @brief Exception factory for creating appropriate exception types
 */
class FaceExceptionFactory {
public:
    /**
     * @brief Create appropriate exception based on error code
     * @param code Face error code
     * @param message Error message
     * @param context Additional context
     * @return Unique pointer to created exception
     */
    static std::unique_ptr<FaceException> createException(
        FaceResultCode code,
        const std::string& message = "",
        const std::map<std::string, std::string>& context = {});

    /**
     * @brief Create exception with component context
     * @param code Face error code
     * @param message Error message
     * @param component Component name that generated error
     * @param operation Operation that failed
     * @return Unique pointer to created exception
     */
    static std::unique_ptr<FaceException> createExceptionWithComponent(
        FaceResultCode code,
        const std::string& message,
        const std::string& component,
        const std::string& operation);
};

/**
 * @brief Exception handler for centralized error processing
 */
class FaceExceptionHandler {
public:
    /**
     * @brief Handle exception with logging and recovery attempts
     * @param exception Exception to handle
     * @param component Component that caught the exception
     * @param attempt_recovery Whether to attempt automatic recovery
     * @return True if recovery was successful
     */
    static bool handleException(const FaceException& exception,
                               const std::string& component,
                               bool attempt_recovery = true);

    /**
     * @brief Log exception details for audit trail
     * @param exception Exception to log
     * @param component Component name
     * @param user_id Associated user ID (if applicable)
     */
    static void logException(const FaceException& exception,
                            const std::string& component,
                            const std::string& user_id = "");

    /**
     * @brief Generate user-friendly error message
     * @param exception Exception to process
     * @return User-friendly error message
     */
    static std::string generateUserMessage(const FaceException& exception);

    /**
     * @brief Generate technical error report
     * @param exception Exception to process
     * @return Detailed technical error report
     */
    static std::string generateTechnicalReport(const FaceException& exception);

    /**
     * @brief Attempt automatic error recovery
     * @param exception Exception to recover from
     * @param component Component attempting recovery
     * @return True if recovery was successful
     */
    static bool attemptRecovery(const FaceException& exception,
                               const std::string& component);
};

/**
 * @brief Utility macros for exception handling
 */

/**
 * @brief Throw face exception with automatic context
 */
#define THROW_FACE_EXCEPTION(code, message) \
    do { \
        auto exception = FaceExceptionFactory::createException(code, message); \
        exception->addContext("file", __FILE__); \
        exception->addContext("line", std::to_string(__LINE__)); \
        exception->addContext("function", __FUNCTION__); \
        throw *exception; \
    } while(0)

/**
 * @brief Throw face exception with component context
 */
#define THROW_FACE_EXCEPTION_WITH_COMPONENT(code, message, component, operation) \
    do { \
        auto exception = FaceExceptionFactory::createExceptionWithComponent(code, message, component, operation); \
        exception->addContext("file", __FILE__); \
        exception->addContext("line", std::to_string(__LINE__)); \
        exception->addContext("function", __FUNCTION__); \
        throw *exception; \
    } while(0)

/**
 * @brief Try-catch block with automatic exception handling
 */
#define FACE_TRY_CATCH_BLOCK(component, operation) \
    try { \
        operation; \
    } catch (const FaceException& ex) { \
        FaceExceptionHandler::handleException(ex, component); \
        throw; \
    } catch (const std::exception& ex) { \
        auto face_ex = FaceExceptionFactory::createException(FaceResultCode::ERROR_GENERIC, ex.what()); \
        face_ex->addContext("component", component); \
        FaceExceptionHandler::handleException(*face_ex, component); \
        throw *face_ex; \
    }

/**
 * @brief Banking-grade error validation
 */
#define VALIDATE_BANKING_GRADE_RESULT(result, far, frr, message) \
    do { \
        if (!result.isBankingGradeMatch() || far >= 0.001f || frr >= 0.03f) { \
            throw BankingComplianceException( \
                FaceResultCode::ERROR_AUTHENTICATION_FAILED, \
                message, \
                far, \
                frr, \
                "Banking grade requirements not met" \
            ); \
        } \
    } while(0)

/**
 * @brief Security validation macro
 */
#define VALIDATE_SECURITY_CONTEXT(condition, message, threat_type) \
    do { \
        if (!(condition)) { \
            throw SecurityException( \
                FaceResultCode::ERROR_AUTHORIZATION_DENIED, \
                message, \
                "Security validation failed", \
                threat_type \
            ); \
        } \
    } while(0)

} // namespace face
} // namespace unlook