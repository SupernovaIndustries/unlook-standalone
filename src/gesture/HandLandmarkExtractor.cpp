/**
 * @file HandLandmarkExtractor.cpp
 * @brief Implementation of hand landmark extraction using ONNX Runtime
 */

#include "unlook/gesture/HandLandmarkExtractor.hpp"
#include <onnxruntime_cxx_api.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <chrono>
#include <stdexcept>
#include <cmath>

namespace unlook {
namespace gesture {

/**
 * @brief PIMPL implementation for HandLandmarkExtractor
 */
class HandLandmarkExtractor::Impl {
public:
    bool initialized = false;
    HandLandmarkConfig config;
    std::string last_error;
    double last_extraction_time_ms = 0.0;

    // ONNX Runtime components
    std::unique_ptr<Ort::Env> ort_env;
    std::unique_ptr<Ort::Session> ort_session;
    std::unique_ptr<Ort::SessionOptions> session_options;
    Ort::AllocatorWithDefaultOptions allocator;

    // Model I/O information
    std::vector<const char*> input_names;
    std::vector<const char*> output_names;
    std::vector<int64_t> input_shape;
    std::vector<int64_t> output_shape;

    // Memory allocations for model names (must persist)
    std::string input_name_str;
    std::vector<std::string> output_name_strs;  // Support multiple outputs (PINTO0309 has 3)

    /**
     * @brief Initialize ONNX Runtime session
     */
    bool initialize_session(const std::string& model_path) {
        try {
            // Create ONNX Runtime environment
            ort_env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "HandLandmarkExtractor");

            // Create session options
            session_options = std::make_unique<Ort::SessionOptions>();
            session_options->SetIntraOpNumThreads(2);  // Optimize for ARM
            session_options->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

            // GPU support if enabled
            if (config.use_gpu) {
                // TensorRT or CUDA provider could be added here
                // For now, CPU only on Raspberry Pi
            }

            // Create session
            ort_session = std::make_unique<Ort::Session>(*ort_env, model_path.c_str(), *session_options);

            // Get input/output information
            size_t num_input_nodes = ort_session->GetInputCount();
            size_t num_output_nodes = ort_session->GetOutputCount();

            if (num_input_nodes != 1) {
                last_error = "Expected 1 input, got " + std::to_string(num_input_nodes) + " inputs";
                return false;
            }

            // PINTO0309 model has 3 outputs: xyz_x21s, hand_scores, handedness
            // We need at least 1 output (landmarks), but can handle 1-3 outputs
            if (num_output_nodes < 1 || num_output_nodes > 3) {
                last_error = "Expected 1-3 outputs (PINTO0309 model has 3), got " +
                             std::to_string(num_output_nodes) + " outputs";
                return false;
            }

            // Get input name and shape
            auto input_name = ort_session->GetInputNameAllocated(0, allocator);
            input_name_str = input_name.get();
            input_names.push_back(input_name_str.c_str());

            auto input_type_info = ort_session->GetInputTypeInfo(0);
            auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
            input_shape = input_tensor_info.GetShape();

            // Get output names and shapes for ALL outputs
            // PINTO0309 model has 3 outputs: xyz_x21s, hand_scores, handedness
            output_name_strs.clear();
            output_names.clear();

            for (size_t i = 0; i < num_output_nodes; i++) {
                auto output_name = ort_session->GetOutputNameAllocated(i, allocator);
                output_name_strs.push_back(output_name.get());
                output_names.push_back(output_name_strs.back().c_str());
            }

            // Get shape of first output (landmarks: xyz_x21s)
            auto output_type_info = ort_session->GetOutputTypeInfo(0);
            auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
            output_shape = output_tensor_info.GetShape();

            // Validate shapes
            // Expected input: [1, 3, 224, 224]
            // Expected output: [1, 21, 3] or [1, 63]
            if (input_shape.size() != 4 ||
                input_shape[1] != 3 ||
                input_shape[2] != config.input_height ||
                input_shape[3] != config.input_width) {
                last_error = "Unexpected input shape";
                return false;
            }

            return true;

        } catch (const Ort::Exception& e) {
            last_error = std::string("ONNX Runtime error: ") + e.what();
            return false;
        } catch (const std::exception& e) {
            last_error = std::string("Initialization error: ") + e.what();
            return false;
        }
    }

    /**
     * @brief Preprocess image ROI to model input format
     */
    bool preprocess(const cv::Mat& image, const cv::Rect& roi, std::vector<float>& input_tensor) {
        try {
            // Expand ROI for better landmark coverage
            cv::Rect expanded_roi = expand_roi(roi, config.roi_expansion_factor, image.size());

            // Crop and resize to model input size
            cv::Mat cropped = image(expanded_roi).clone();
            cv::Mat resized;
            cv::resize(cropped, resized, cv::Size(config.input_width, config.input_height),
                      0, 0, cv::INTER_LINEAR);

            // Convert BGR to RGB if needed
            if (resized.channels() == 3 && image.channels() == 3) {
                cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
            }

            // Normalize to [0, 1] and convert to float
            resized.convertTo(resized, CV_32F, 1.0 / 255.0);

            // Prepare tensor data (CHW format: [C, H, W])
            const size_t total_size = 3 * config.input_height * config.input_width;
            input_tensor.resize(total_size);

            // Convert HWC to CHW
            std::vector<cv::Mat> channels(3);
            cv::split(resized, channels);

            size_t channel_size = config.input_height * config.input_width;
            for (int c = 0; c < 3; ++c) {
                std::memcpy(input_tensor.data() + c * channel_size,
                           channels[c].data,
                           channel_size * sizeof(float));
            }

            return true;

        } catch (const std::exception& e) {
            last_error = std::string("Preprocessing error: ") + e.what();
            return false;
        }
    }

    /**
     * @brief Postprocess model output to hand landmarks
     */
    bool postprocess(const std::vector<float>& output_tensor,
                     const cv::Rect& original_roi,
                     const cv::Size& image_size,
                     HandLandmarks& landmarks) {
        try {
            // Expected output: 21 landmarks * 3 coordinates (x, y, z)
            if (output_tensor.size() < 63) {
                last_error = "Insufficient output data";
                return false;
            }

            // Expand ROI (same as preprocessing)
            cv::Rect expanded_roi = expand_roi(original_roi, config.roi_expansion_factor, image_size);

            // Parse landmarks (normalized [0, 1] in ROI space)
            for (int i = 0; i < 21; ++i) {
                float x_norm = output_tensor[i * 3 + 0];
                float y_norm = output_tensor[i * 3 + 1];
                float z_norm = output_tensor[i * 3 + 2];

                // Convert from ROI-normalized to image-normalized coordinates
                float x_img = (x_norm * expanded_roi.width + expanded_roi.x) / static_cast<float>(image_size.width);
                float y_img = (y_norm * expanded_roi.height + expanded_roi.y) / static_cast<float>(image_size.height);

                landmarks.points[i] = cv::Point3f(x_img, y_img, z_norm);
            }

            // Calculate average confidence (simplified - could use dedicated confidence output)
            float avg_confidence = 0.0f;
            for (int i = 0; i < 21; ++i) {
                // Check if point is within reasonable bounds
                if (landmarks.points[i].x >= 0.0f && landmarks.points[i].x <= 1.0f &&
                    landmarks.points[i].y >= 0.0f && landmarks.points[i].y <= 1.0f) {
                    avg_confidence += 1.0f;
                }
            }
            landmarks.confidence = avg_confidence / 21.0f;
            landmarks.image_size = image_size;

            return landmarks.confidence >= config.confidence_threshold;

        } catch (const std::exception& e) {
            last_error = std::string("Postprocessing error: ") + e.what();
            return false;
        }
    }

    /**
     * @brief Expand ROI with boundary clamping
     */
    static cv::Rect expand_roi(const cv::Rect& roi, float factor, const cv::Size& image_size) {
        float center_x = roi.x + roi.width / 2.0f;
        float center_y = roi.y + roi.height / 2.0f;
        float new_width = roi.width * factor;
        float new_height = roi.height * factor;

        int x = static_cast<int>(center_x - new_width / 2.0f);
        int y = static_cast<int>(center_y - new_height / 2.0f);
        int w = static_cast<int>(new_width);
        int h = static_cast<int>(new_height);

        // Clamp to image boundaries
        x = std::max(0, std::min(x, image_size.width - 1));
        y = std::max(0, std::min(y, image_size.height - 1));
        w = std::min(w, image_size.width - x);
        h = std::min(h, image_size.height - y);

        return cv::Rect(x, y, w, h);
    }
};

// ===== Public API Implementation =====

HandLandmarkExtractor::HandLandmarkExtractor()
    : pImpl(std::make_unique<Impl>()) {
}

HandLandmarkExtractor::~HandLandmarkExtractor() = default;

bool HandLandmarkExtractor::initialize(const HandLandmarkConfig& config) {
    if (!config.is_valid()) {
        pImpl->last_error = "Invalid configuration";
        return false;
    }

    pImpl->config = config;
    pImpl->initialized = pImpl->initialize_session(config.model_path);
    return pImpl->initialized;
}

bool HandLandmarkExtractor::is_initialized() const {
    return pImpl->initialized;
}

bool HandLandmarkExtractor::extract(const cv::Mat& image,
                                     const cv::Rect& hand_roi,
                                     HandLandmarks& landmarks) {
    if (!pImpl->initialized) {
        pImpl->last_error = "Extractor not initialized";
        return false;
    }

    if (image.empty()) {
        pImpl->last_error = "Empty input image";
        return false;
    }

    auto start = std::chrono::high_resolution_clock::now();

    try {
        // Preprocess image
        std::vector<float> input_tensor;
        if (!pImpl->preprocess(image, hand_roi, input_tensor)) {
            return false;
        }

        // Create input tensor
        std::vector<int64_t> input_shape = {1, 3, pImpl->config.input_height, pImpl->config.input_width};
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor_ort = Ort::Value::CreateTensor<float>(
            memory_info,
            input_tensor.data(),
            input_tensor.size(),
            input_shape.data(),
            input_shape.size()
        );

        // Run inference with ALL outputs (PINTO0309 model has 3)
        auto output_tensors = pImpl->ort_session->Run(
            Ort::RunOptions{nullptr},
            pImpl->input_names.data(),
            &input_tensor_ort,
            1,
            pImpl->output_names.data(),
            pImpl->output_names.size()  // Request all outputs (3 for PINTO0309)
        );

        // Extract FIRST output data (xyz_x21s - landmarks)
        // PINTO0309 outputs: [0]=xyz_x21s [1]=hand_scores [2]=handedness
        float* output_data = output_tensors[0].GetTensorMutableData<float>();
        auto output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
        size_t output_size = 1;
        for (auto dim : output_shape) {
            output_size *= dim;
        }

        std::vector<float> output_tensor(output_data, output_data + output_size);

        // TODO: Optionally use output_tensors[1] for hand_scores confidence
        // TODO: Optionally use output_tensors[2] for handedness (left/right)

        // Postprocess results
        if (!pImpl->postprocess(output_tensor, hand_roi, image.size(), landmarks)) {
            return false;
        }

        auto end = std::chrono::high_resolution_clock::now();
        pImpl->last_extraction_time_ms = std::chrono::duration<double, std::milli>(end - start).count();

        return true;

    } catch (const Ort::Exception& e) {
        pImpl->last_error = std::string("ONNX Runtime error: ") + e.what();
        return false;
    } catch (const std::exception& e) {
        pImpl->last_error = std::string("Extraction error: ") + e.what();
        return false;
    }
}

double HandLandmarkExtractor::get_last_extraction_time_ms() const {
    return pImpl->last_extraction_time_ms;
}

HandLandmarkConfig HandLandmarkExtractor::get_config() const {
    return pImpl->config;
}

std::string HandLandmarkExtractor::get_last_error() const {
    return pImpl->last_error;
}

} // namespace gesture
} // namespace unlook
