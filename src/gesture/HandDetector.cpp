/**
 * @file HandDetector.cpp
 * @brief Implementation of hand detector using ONNX Runtime
 */

#include <unlook/gesture/HandDetector.hpp>
#include <unlook/core/Logger.hpp>

#include <onnxruntime_cxx_api.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utils/logger.hpp>

#include <chrono>
#include <algorithm>
#include <cmath>

namespace unlook {
namespace gesture {

// Helper function implementations
cv::Rect HandDetection::get_landmark_roi(float expansion_factor, const cv::Size& image_size) const {
    // Expand bounding box for landmark extraction
    int cx = bounding_box.x + bounding_box.width / 2;
    int cy = bounding_box.y + bounding_box.height / 2;
    int size = std::max(bounding_box.width, bounding_box.height);
    int expanded_size = static_cast<int>(size * expansion_factor);

    cv::Rect roi(
        cx - expanded_size / 2,
        cy - expanded_size / 2,
        expanded_size,
        expanded_size
    );

    // Clamp to image boundaries
    roi.x = std::max(0, roi.x);
    roi.y = std::max(0, roi.y);
    roi.width = std::min(roi.width, image_size.width - roi.x);
    roi.height = std::min(roi.height, image_size.height - roi.y);

    return roi;
}

// PIMPL implementation
class HandDetector::Impl {
public:
    HandDetectorConfig config;
    bool initialized = false;
    std::string last_error;
    double last_detection_time_ms = 0.0;

    // ONNX Runtime session
    std::unique_ptr<Ort::Env> ort_env;
    std::unique_ptr<Ort::Session> ort_session;
    Ort::MemoryInfo memory_info{nullptr};

    // Model I/O names
    std::vector<const char*> input_names;
    std::vector<const char*> output_names;
    std::vector<int64_t> input_shape;

    /**
     * @brief Preprocess image for palm detection model
     */
    cv::Mat preprocess_image(const cv::Mat& image) {
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(config.input_width, config.input_height));

        // Convert BGR to RGB if needed
        cv::Mat rgb;
        if (image.channels() == 3) {
            cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
        } else {
            rgb = resized.clone();
        }

        // Normalize to [0, 1]
        cv::Mat normalized;
        rgb.convertTo(normalized, CV_32FC3, 1.0 / 255.0);

        return normalized;
    }

    /**
     * @brief Non-maximum suppression for overlapping detections
     */
    std::vector<int> nms(const std::vector<cv::Rect>& boxes,
                         const std::vector<float>& scores,
                         float iou_threshold = 0.3f) {
        std::vector<int> indices;
        if (boxes.empty()) return indices;

        // Create index list sorted by score
        std::vector<std::pair<float, int>> score_index;
        for (size_t i = 0; i < scores.size(); ++i) {
            score_index.push_back({scores[i], i});
        }
        std::sort(score_index.rbegin(), score_index.rend());

        std::vector<bool> suppressed(boxes.size(), false);

        for (const auto& [score, idx] : score_index) {
            if (suppressed[idx]) continue;

            indices.push_back(idx);
            if (indices.size() >= static_cast<size_t>(config.max_num_hands)) {
                break;
            }

            // Suppress overlapping boxes
            const cv::Rect& box_i = boxes[idx];
            for (size_t j = 0; j < boxes.size(); ++j) {
                if (j == static_cast<size_t>(idx) || suppressed[j]) continue;

                const cv::Rect& box_j = boxes[j];
                cv::Rect intersection = box_i & box_j;
                float iou = static_cast<float>(intersection.area()) /
                           static_cast<float>((box_i | box_j).area());

                if (iou > iou_threshold) {
                    suppressed[j] = true;
                }
            }
        }

        return indices;
    }

    /**
     * @brief Parse model output to hand detections
     */
    std::vector<HandDetection> parse_detections(
        const std::vector<Ort::Value>& output_tensors,
        const cv::Size& original_size) {

        std::vector<HandDetection> detections;

        // Model output format: [num_detections, 18]
        // Each detection: [score, cx, cy, w, h, ... additional params]
        // Note: Actual format depends on the PINTO palm detection model

        try {
            // Get output tensor
            const float* output_data = output_tensors[0].GetTensorData<float>();
            auto shape_info = output_tensors[0].GetTensorTypeAndShapeInfo();
            auto shape = shape_info.GetShape();

            if (shape.size() < 2) {
                last_error = "Unexpected output tensor shape";
                return detections;
            }

            int num_detections = static_cast<int>(shape[0]);
            int num_features = static_cast<int>(shape[1]);

            // Scale factors for converting from model coordinates to image coordinates
            float scale_x = static_cast<float>(original_size.width) / config.input_width;
            float scale_y = static_cast<float>(original_size.height) / config.input_height;

            std::vector<cv::Rect> boxes;
            std::vector<float> scores;

            for (int i = 0; i < num_detections; ++i) {
                const float* detection = output_data + i * num_features;

                float score = detection[0];  // First element is score
                if (score < config.score_threshold) {
                    continue;
                }

                // Parse bounding box (format may vary, adjust based on actual model output)
                float cx = detection[1] * scale_x;
                float cy = detection[2] * scale_y;
                float w = detection[3] * scale_x;
                float h = detection[4] * scale_y;

                cv::Rect bbox(
                    static_cast<int>(cx - w / 2),
                    static_cast<int>(cy - h / 2),
                    static_cast<int>(w),
                    static_cast<int>(h)
                );

                boxes.push_back(bbox);
                scores.push_back(score);
            }

            // Apply NMS
            std::vector<int> keep_indices = nms(boxes, scores);

            // Create final detections
            for (int idx : keep_indices) {
                HandDetection det;
                det.bounding_box = boxes[idx];
                det.score = scores[idx];
                det.center = cv::Point2f(
                    det.bounding_box.x + det.bounding_box.width / 2.0f,
                    det.bounding_box.y + det.bounding_box.height / 2.0f
                );
                detections.push_back(det);
            }

        } catch (const std::exception& e) {
            last_error = std::string("Error parsing detections: ") + e.what();
        }

        return detections;
    }
};

// HandDetector implementation
HandDetector::HandDetector()
    : pImpl(std::make_unique<Impl>()) {
}

HandDetector::~HandDetector() = default;

bool HandDetector::initialize(const HandDetectorConfig& config) {
    if (!config.is_valid()) {
        pImpl->last_error = "Invalid detector configuration";
        return false;
    }

    pImpl->config = config;

    try {
        // Initialize ONNX Runtime environment
        pImpl->ort_env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "HandDetector");

        // Session options
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(2);  // Optimize for CM5
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        // Set execution providers
        std::vector<std::string> providers;
        if (config.use_gpu) {
            providers.push_back("CUDAExecutionProvider");
        }
        providers.push_back("CPUExecutionProvider");

        // Load model
        pImpl->ort_session = std::make_unique<Ort::Session>(
            *pImpl->ort_env,
            config.model_path.c_str(),
            session_options
        );

        // Get input/output names
        Ort::AllocatorWithDefaultOptions allocator;

        // Input
        size_t num_input_nodes = pImpl->ort_session->GetInputCount();
        if (num_input_nodes != 1) {
            pImpl->last_error = "Expected 1 input node, got " + std::to_string(num_input_nodes);
            return false;
        }

        char* input_name_cstr = pImpl->ort_session->GetInputNameAllocated(0, allocator).release();
        pImpl->input_names.push_back(input_name_cstr);

        Ort::TypeInfo input_type_info = pImpl->ort_session->GetInputTypeInfo(0);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        pImpl->input_shape = input_tensor_info.GetShape();

        // Output
        size_t num_output_nodes = pImpl->ort_session->GetOutputCount();
        for (size_t i = 0; i < num_output_nodes; ++i) {
            char* output_name_cstr = pImpl->ort_session->GetOutputNameAllocated(i, allocator).release();
            pImpl->output_names.push_back(output_name_cstr);
        }

        // Memory info for input tensor
        pImpl->memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        pImpl->initialized = true;
        return true;

    } catch (const Ort::Exception& e) {
        pImpl->last_error = std::string("ONNX Runtime error: ") + e.what();
        return false;
    } catch (const std::exception& e) {
        pImpl->last_error = std::string("Initialization error: ") + e.what();
        return false;
    }
}

bool HandDetector::is_initialized() const {
    return pImpl->initialized;
}

bool HandDetector::detect(const cv::Mat& image, std::vector<HandDetection>& detections) {
    if (!pImpl->initialized) {
        pImpl->last_error = "Detector not initialized";
        return false;
    }

    if (image.empty()) {
        pImpl->last_error = "Empty input image";
        return false;
    }

    auto start_time = std::chrono::steady_clock::now();

    try {
        // Preprocess image
        cv::Mat preprocessed = pImpl->preprocess_image(image);

        // Create input tensor in NCHW format (required by PINTO0309 models)
        std::vector<int64_t> input_shape = {
            1,                             // Batch
            3,                             // Channels (RGB)
            pImpl->config.input_height,    // Height (192)
            pImpl->config.input_width      // Width (192)
        };

        size_t input_tensor_size = 1 * 3 * pImpl->config.input_height *
                                   pImpl->config.input_width;

        std::vector<float> input_tensor_values(input_tensor_size);

        // Convert from HWC (OpenCV) to CHW (ONNX) format - OPTIMIZED with cv::split
        // preprocessed is [H, W, C], need to convert to [C, H, W]
        std::vector<cv::Mat> channels(3);
        cv::split(preprocessed, channels);  // Split into 3 separate channel planes (fast)

        // Copy channel planes sequentially into tensor (R, G, B order)
        int H = pImpl->config.input_height;
        int W = pImpl->config.input_width;
        int channel_size = H * W;

        for (int c = 0; c < 3; ++c) {
            std::memcpy(input_tensor_values.data() + c * channel_size,
                       channels[c].data,
                       channel_size * sizeof(float));
        }

        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            pImpl->memory_info,
            input_tensor_values.data(),
            input_tensor_size,
            input_shape.data(),
            input_shape.size()
        );

        // Run inference
        std::vector<Ort::Value> output_tensors = pImpl->ort_session->Run(
            Ort::RunOptions{nullptr},
            pImpl->input_names.data(),
            &input_tensor,
            1,
            pImpl->output_names.data(),
            pImpl->output_names.size()
        );

        // Parse detections
        detections = pImpl->parse_detections(output_tensors, image.size());

        auto end_time = std::chrono::steady_clock::now();
        pImpl->last_detection_time_ms = std::chrono::duration<double, std::milli>(
            end_time - start_time).count();

        return true;

    } catch (const Ort::Exception& e) {
        pImpl->last_error = std::string("Detection error: ") + e.what();
        return false;
    } catch (const std::exception& e) {
        pImpl->last_error = std::string("Detection error: ") + e.what();
        return false;
    }
}

double HandDetector::get_last_detection_time_ms() const {
    return pImpl->last_detection_time_ms;
}

HandDetectorConfig HandDetector::get_config() const {
    return pImpl->config;
}

std::string HandDetector::get_last_error() const {
    return pImpl->last_error;
}

} // namespace gesture
} // namespace unlook
