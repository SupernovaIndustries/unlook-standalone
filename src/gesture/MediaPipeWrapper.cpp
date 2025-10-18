/**
 * @file MediaPipeWrapper.cpp
 * @brief Implementation of MediaPipe Python wrapper using pybind11
 */

#include "unlook/gesture/MediaPipeWrapper.hpp"
#include "unlook/core/Logger.hpp"
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <mutex>
#include <stdexcept>

namespace py = pybind11;
using namespace pybind11::literals;

namespace unlook {
namespace gesture {

/**
 * @brief PIMPL implementation class
 *
 * Contains all Python/pybind11 specific code to avoid polluting
 * the header with Python dependencies.
 */
class MediaPipeWrapper::Impl {
public:
    /**
     * @brief Python interpreter guard (singleton)
     *
     * Only one Python interpreter can exist per process.
     * This guard ensures the interpreter is initialized once
     * and destroyed at program exit.
     */
    static py::scoped_interpreter& getPythonInterpreter() {
        static py::scoped_interpreter guard{};
        return guard;
    }

    /**
     * @brief Initialize MediaPipe detector
     */
    Impl(const std::string& model_path, int num_hands,
         float min_detection_confidence, float min_tracking_confidence)
        : initialized_(false)
        , last_error_("")
        , last_inference_time_ms_(0.0f)
    {
        LOG_INFO("MediaPipeWrapper: Initializing...");

        try {
            // Ensure Python interpreter is initialized
            getPythonInterpreter();

            // Get Python system module
            py::module_ sys = py::module_::import("sys");

            // DEBUG: Print Python version and executable
            std::string py_version = sys.attr("version").cast<std::string>();
            std::string py_executable = sys.attr("executable").cast<std::string>();
            LOG_INFO("MediaPipeWrapper: Python version: " + py_version);
            LOG_INFO("MediaPipeWrapper: Python executable: " + py_executable);

            // Get current sys.path
            py::list path = sys.attr("path");

            // DEBUG: Print original sys.path
            LOG_INFO("MediaPipeWrapper: Original sys.path:");
            for (size_t i = 0; i < py::len(path); ++i) {
                std::string p = path[i].cast<std::string>();
                LOG_INFO("  [" + std::to_string(i) + "] " + p);
            }

            // Add user site-packages directory (where pip --user installs packages)
            std::string user_site_packages = "/home/alessandro/.local/lib/python3.11/site-packages";
            path.insert(0, user_site_packages);
            LOG_INFO("MediaPipeWrapper: Added user site-packages: " + user_site_packages);

            // Add absolute path to Python scripts
            std::string python_path = "/home/alessandro/unlook-gesture/python";
            path.append(python_path);
            LOG_INFO("MediaPipeWrapper: Added Python path: " + python_path);

            // DEBUG: Print modified sys.path
            LOG_INFO("MediaPipeWrapper: Modified sys.path:");
            for (size_t i = 0; i < py::len(path); ++i) {
                std::string p = path[i].cast<std::string>();
                LOG_INFO("  [" + std::to_string(i) + "] " + p);
            }

            // DEBUG: Try to import cv2 directly
            LOG_INFO("MediaPipeWrapper: Attempting to import cv2...");
            try {
                py::module_ cv2 = py::module_::import("cv2");
                std::string cv2_version = cv2.attr("__version__").cast<std::string>();
                LOG_INFO("MediaPipeWrapper: Successfully imported cv2 version " + cv2_version);
            } catch (const py::error_already_set& e) {
                LOG_ERROR("MediaPipeWrapper: FAILED to import cv2: " + std::string(e.what()));
                throw;
            }

            // DEBUG: Try to import mediapipe directly
            LOG_INFO("MediaPipeWrapper: Attempting to import mediapipe...");
            try {
                py::module_ mp = py::module_::import("mediapipe");
                std::string mp_version = mp.attr("__version__").cast<std::string>();
                LOG_INFO("MediaPipeWrapper: Successfully imported mediapipe version " + mp_version);
            } catch (const py::error_already_set& e) {
                LOG_ERROR("MediaPipeWrapper: FAILED to import mediapipe: " + std::string(e.what()));
                throw;
            }

            // Import mediapipe_gesture_detector module
            LOG_INFO("MediaPipeWrapper: Attempting to import mediapipe_gesture_detector module...");
            py::module_ mp_module = py::module_::import("mediapipe_gesture_detector");
            LOG_INFO("MediaPipeWrapper: Imported mediapipe_gesture_detector module");

            // Get MediaPipeGestureDetector class
            py::object MediaPipeGestureDetector = mp_module.attr("MediaPipeGestureDetector");

            // Create detector instance
            detector_ = MediaPipeGestureDetector(
                "model_path"_a = model_path,
                "num_hands"_a = num_hands,
                "min_detection_confidence"_a = min_detection_confidence,
                "min_tracking_confidence"_a = min_tracking_confidence
            );

            LOG_INFO("MediaPipeWrapper: Created MediaPipeGestureDetector instance");

            initialized_ = true;
            LOG_INFO("MediaPipeWrapper: Initialization complete");

        } catch (const py::error_already_set& e) {
            last_error_ = std::string("Python error: ") + e.what();
            LOG_ERROR("MediaPipeWrapper: " + last_error_);
            throw std::runtime_error(last_error_);
        } catch (const std::exception& e) {
            last_error_ = std::string("C++ error: ") + e.what();
            LOG_ERROR("MediaPipeWrapper: " + last_error_);
            throw std::runtime_error(last_error_);
        }
    }

    /**
     * @brief Destructor - cleanup Python resources
     */
    ~Impl() {
        try {
            if (initialized_ && !detector_.is_none()) {
                detector_.attr("close")();
                LOG_INFO("MediaPipeWrapper: Closed Python detector");
            }
        } catch (...) {
            // Ignore cleanup errors
            LOG_WARNING("MediaPipeWrapper: Error during cleanup (ignored)");
        }
    }

    /**
     * @brief Detect gesture from frame
     */
    bool detect(const cv::Mat& frame, std::string& gesture_name,
                std::vector<std::vector<float>>& landmarks, float& confidence) {
        if (!initialized_) {
            last_error_ = "MediaPipe wrapper not initialized";
            return false;
        }

        if (frame.empty()) {
            last_error_ = "Empty input frame";
            return false;
        }

        if (frame.channels() != 3) {
            last_error_ = "Frame must have 3 channels (BGR)";
            return false;
        }

        try {
            // Convert cv::Mat to numpy array (zero-copy if contiguous)
            // MediaPipe expects BGR format (OpenCV default)
            py::array_t<uint8_t> np_frame({frame.rows, frame.cols, frame.channels()},
                                          {frame.step[0], frame.step[1], sizeof(uint8_t)},
                                          frame.data,
                                          py::cast(frame)); // Keep cv::Mat alive

            // Call Python detect method
            py::tuple result = detector_.attr("detect")(np_frame);

            // Extract results
            py::object py_gesture = result[0];
            py::object py_landmarks = result[1];
            py::object py_confidence = result[2];

            // Convert gesture name
            gesture_name.clear();
            if (!py_gesture.is_none()) {
                gesture_name = py_gesture.cast<std::string>();
            }

            // Convert landmarks
            landmarks.clear();
            if (py::isinstance<py::list>(py_landmarks)) {
                py::list lm_list = py_landmarks.cast<py::list>();
                for (auto lm : lm_list) {
                    py::list point = lm.cast<py::list>();
                    if (point.size() >= 3) {
                        std::vector<float> pt = {
                            point[0].cast<float>(),
                            point[1].cast<float>(),
                            point[2].cast<float>()
                        };
                        landmarks.push_back(pt);
                    }
                }
            }

            // Convert confidence
            confidence = py_confidence.cast<float>();

            // Update performance stats
            updatePerformanceStats();

            return true;

        } catch (const py::error_already_set& e) {
            last_error_ = std::string("Python error in detect(): ") + e.what();
            LOG_ERROR("MediaPipeWrapper: " + last_error_);
            return false;
        } catch (const std::exception& e) {
            last_error_ = std::string("C++ error in detect(): ") + e.what();
            LOG_ERROR("MediaPipeWrapper: " + last_error_);
            return false;
        }
    }

    /**
     * @brief Get average inference time
     */
    float getAvgInferenceTimeMs() const {
        return last_inference_time_ms_;
    }

    /**
     * @brief Check if initialized
     */
    bool isInitialized() const {
        return initialized_;
    }

    /**
     * @brief Get last error
     */
    std::string getLastError() const {
        return last_error_;
    }

private:
    /**
     * @brief Update performance statistics from Python detector
     */
    void updatePerformanceStats() {
        try {
            if (initialized_ && !detector_.is_none()) {
                last_inference_time_ms_ = detector_.attr("get_avg_inference_time_ms")().cast<float>();
            }
        } catch (...) {
            // Ignore performance stat errors
        }
    }

    py::object detector_;           ///< Python MediaPipeGestureDetector instance
    bool initialized_;              ///< Initialization flag
    std::string last_error_;        ///< Last error message
    float last_inference_time_ms_;  ///< Last inference time
};

// ============================================================================
// MediaPipeWrapper Public API Implementation
// ============================================================================

MediaPipeWrapper::MediaPipeWrapper(const std::string& model_path, int num_hands,
                                   float min_detection_confidence,
                                   float min_tracking_confidence)
    : pImpl(std::make_unique<Impl>(model_path, num_hands,
                                    min_detection_confidence,
                                    min_tracking_confidence))
{
}

MediaPipeWrapper::~MediaPipeWrapper() = default;

bool MediaPipeWrapper::detect(const cv::Mat& frame, std::string& gesture_name,
                               std::vector<std::vector<float>>& landmarks,
                               float& confidence) {
    return pImpl->detect(frame, gesture_name, landmarks, confidence);
}

float MediaPipeWrapper::getAvgInferenceTimeMs() const {
    return pImpl->getAvgInferenceTimeMs();
}

bool MediaPipeWrapper::isInitialized() const {
    return pImpl->isInitialized();
}

std::string MediaPipeWrapper::getLastError() const {
    return pImpl->getLastError();
}

} // namespace gesture
} // namespace unlook
