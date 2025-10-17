/**
 * @file HandDetector.cpp
 * @brief Implementation of hand detector using ONNX Runtime
 */

#include <unlook/gesture/HandDetector.hpp>
#include <unlook/core/Logger.hpp>

#include <onnxruntime_cxx_api.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
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
        static int preprocess_count = 0;
        preprocess_count++;

        if (preprocess_count <= 3 || preprocess_count % 100 == 0) {
            LOG_DEBUG("HandDetector: Preprocessing image #" + std::to_string(preprocess_count));
            LOG_DEBUG("  Input size: " + std::to_string(image.cols) + "x" + std::to_string(image.rows));
            LOG_DEBUG("  Input channels: " + std::to_string(image.channels()));
            LOG_DEBUG("  Input type: " + std::to_string(image.type()));
        }

        cv::Mat resized;
        cv::resize(image, resized, cv::Size(config.input_width, config.input_height));

        // Convert BGR to RGB if needed
        cv::Mat rgb;
        if (image.channels() == 3) {
            cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
        } else {
            rgb = resized.clone();
        }

        // Apply CLAHE contrast enhancement if enabled
        if (config.use_clahe) {
            // Convert RGB to HSV for CLAHE on V channel
            cv::Mat hsv;
            cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV);

            // Split channels
            std::vector<cv::Mat> hsv_channels;
            cv::split(hsv, hsv_channels);

            // Apply CLAHE on V (Value/Brightness) channel
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
                config.clahe_clip_limit,
                cv::Size(config.clahe_tile_grid_width, config.clahe_tile_grid_height)
            );
            clahe->apply(hsv_channels[2], hsv_channels[2]);

            // Merge channels back
            cv::merge(hsv_channels, hsv);

            // Convert back to RGB
            cv::cvtColor(hsv, rgb, cv::COLOR_HSV2RGB);

            if (preprocess_count <= 3) {
                LOG_INFO("  CLAHE applied: clip_limit=" + std::to_string(config.clahe_clip_limit) +
                        ", tile_size=" + std::to_string(config.clahe_tile_grid_width) + "x" +
                        std::to_string(config.clahe_tile_grid_height));
            }
        }

        // Normalize to [0, 1]
        cv::Mat normalized;
        rgb.convertTo(normalized, CV_32FC3, 1.0 / 255.0);

        if (preprocess_count <= 3) {
            // Verify normalization by checking pixel value range
            double minVal, maxVal;
            cv::minMaxLoc(normalized, &minVal, &maxVal);
            LOG_DEBUG("  After normalization: min=" + std::to_string(minVal) +
                     ", max=" + std::to_string(maxVal));
            LOG_DEBUG("  Output size: " + std::to_string(normalized.cols) + "x" +
                     std::to_string(normalized.rows));
            LOG_DEBUG("  Output channels: " + std::to_string(normalized.channels()));
        }

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

        // PINTO0309 palm detection model output format analysis
        // Based on MediaPipe palm detection architecture with SSD anchors
        // Expected outputs: multiple tensors with specific roles

        try {
            // Log output tensor information for debugging
            LOG_INFO("HandDetector: Parsing model output...");
            LOG_INFO("  Number of output tensors: " + std::to_string(output_tensors.size()));

            for (size_t i = 0; i < output_tensors.size(); ++i) {
                auto shape_info = output_tensors[i].GetTensorTypeAndShapeInfo();
                auto shape = shape_info.GetShape();

                std::string shape_str = "  Output[" + std::to_string(i) + "] shape: [";
                for (size_t j = 0; j < shape.size(); ++j) {
                    shape_str += std::to_string(shape[j]);
                    if (j < shape.size() - 1) shape_str += ", ";
                }
                shape_str += "]";
                LOG_INFO(shape_str);
            }

            // Primary output tensor (usually index 0 for detection results)
            const float* output_data = output_tensors[0].GetTensorData<float>();
            auto shape_info = output_tensors[0].GetTensorTypeAndShapeInfo();
            auto shape = shape_info.GetShape();

            if (shape.size() < 2) {
                last_error = "Unexpected output tensor shape (need at least 2 dimensions)";
                LOG_ERROR(last_error);
                return detections;
            }

            // Determine output format based on shape
            int num_detections = 0;
            int num_features = 0;

            if (shape.size() == 2) {
                // Format: [num_detections, features]
                num_detections = static_cast<int>(shape[0]);
                num_features = static_cast<int>(shape[1]);
            } else if (shape.size() == 3) {
                // Format: [batch, num_detections, features] - extract from batch 0
                num_detections = static_cast<int>(shape[1]);
                num_features = static_cast<int>(shape[2]);
            } else {
                last_error = "Unsupported output tensor shape dimensions: " + std::to_string(shape.size());
                LOG_ERROR(last_error);
                return detections;
            }

            LOG_INFO("  Parsing detections: num_detections=" + std::to_string(num_detections) +
                    ", num_features=" + std::to_string(num_features));

            // Scale factors for converting from model coordinates to image coordinates
            float scale_x = static_cast<float>(original_size.width) / config.input_width;
            float scale_y = static_cast<float>(original_size.height) / config.input_height;

            std::vector<cv::Rect> boxes;
            std::vector<float> scores;
            int detections_above_threshold = 0;
            int total_valid_detections = 0;

            // CRITICAL: Parse detections based on PINTO0309 format
            for (int i = 0; i < num_detections; ++i) {
                const float* detection = output_data + i * num_features;

                // Extract score based on format
                float score = 0.0f;

                if (num_features == 8) {
                    // PINTO0309 palm detection format
                    // [0] = pd_score (detection confidence)
                    score = detection[0];

                } else if (num_features >= 18) {
                    // MediaPipe full format: score typically at index 16 or 17
                    score = detection[16];

                    // Validate and try fallback positions if needed
                    if (score < 0.0f || score > 1.0f) {
                        score = detection[17];
                    }
                    if (score < 0.0f || score > 1.0f) {
                        score = detection[0];  // Last resort fallback
                    }

                } else if (num_features >= 5) {
                    // Simplified format: [score, ...]
                    score = detection[0];

                } else {
                    LOG_WARNING("  Detection[" + std::to_string(i) + "]: Insufficient features (" +
                               std::to_string(num_features) + ")");
                    continue;
                }

                // Log first few detections for debugging
                if (i < 5) {
                    LOG_DEBUG("  Detection[" + std::to_string(i) + "]: score=" + std::to_string(score) +
                             " (threshold=" + std::to_string(config.score_threshold) + ")");

                    // Log raw values for PINTO0309 format debugging
                    if (num_features == 8) {
                        LOG_DEBUG("    Raw PINTO0309 values: [score=" + std::to_string(detection[0]) +
                                 ", box_x=" + std::to_string(detection[1]) +
                                 ", box_y=" + std::to_string(detection[2]) +
                                 ", box_size=" + std::to_string(detection[3]) + "]");
                    }
                }

                // Count detections above threshold
                if (score >= config.score_threshold) {
                    detections_above_threshold++;
                }

                // TEMPORARY DEBUG: Lower threshold to 0.01 to see ALL detections
                float debug_threshold = std::min(config.score_threshold, 0.01f);

                if (score < debug_threshold) {
                    continue;
                }

                total_valid_detections++;

                // Parse bounding box based on PINTO0309 format
                // Reference: palm_detection.py line 196
                // Format: pd_score, box_x, box_y, box_size, kp0_x, kp0_y, kp2_x, kp2_y
                float cx, cy, w, h;

                if (num_features == 8) {
                    // PINTO0309 palm detection format (8 features)
                    // [0] = pd_score (already extracted above)
                    // [1] = box_x (center X, normalized [0,1])
                    // [2] = box_y (center Y, normalized [0,1])
                    // [3] = box_size (SINGLE value - bbox is SQUARE!)
                    // [4-7] = keypoints (kp0_x, kp0_y, kp2_x, kp2_y) - not used for bbox

                    float box_x = detection[1];     // center X normalized
                    float box_y = detection[2];     // center Y normalized
                    float box_size = detection[3];  // square box size normalized

                    // Convert normalized [0,1] to pixel coordinates
                    cx = box_x * config.input_width * scale_x;
                    cy = box_y * config.input_height * scale_y;
                    w = box_size * config.input_width * scale_x;
                    h = box_size * config.input_height * scale_y;  // Same as width (square bbox)

                } else if (num_features >= 18) {
                    // MediaPipe full format with landmarks
                    // [ymin, xmin, ymax, xmax] at indices 4-7 (normalized)
                    float ymin = detection[4];
                    float xmin = detection[5];
                    float ymax = detection[6];
                    float xmax = detection[7];

                    // Convert corners to center+size
                    cx = ((xmin + xmax) / 2.0f) * config.input_width * scale_x;
                    cy = ((ymin + ymax) / 2.0f) * config.input_height * scale_y;
                    w = (xmax - xmin) * config.input_width * scale_x;
                    h = (ymax - ymin) * config.input_height * scale_y;

                } else if (num_features >= 5) {
                    // Simplified format: [score, cx, cy, w, h]
                    cx = detection[1] * scale_x;
                    cy = detection[2] * scale_y;
                    w = detection[3] * scale_x;
                    h = detection[4] * scale_y;

                } else {
                    // Insufficient features
                    continue;
                }

                // Convert center-based to corner-based bbox
                cv::Rect bbox(
                    static_cast<int>(cx - w / 2),
                    static_cast<int>(cy - h / 2),
                    static_cast<int>(w),
                    static_cast<int>(h)
                );

                // Clamp to image bounds
                bbox.x = std::max(0, bbox.x);
                bbox.y = std::max(0, bbox.y);
                bbox.width = std::min(bbox.width, original_size.width - bbox.x);
                bbox.height = std::min(bbox.height, original_size.height - bbox.y);

                // Validate bbox
                if (bbox.width <= 0 || bbox.height <= 0) {
                    continue;
                }

                boxes.push_back(bbox);
                scores.push_back(score);

                if (i < 5) {
                    LOG_DEBUG("  Detection[" + std::to_string(i) + "]: bbox=[" +
                             std::to_string(bbox.x) + ", " + std::to_string(bbox.y) + ", " +
                             std::to_string(bbox.width) + ", " + std::to_string(bbox.height) + "]");
                }
            }

            LOG_INFO("  Total detections found: " + std::to_string(total_valid_detections) +
                    " (above threshold: " + std::to_string(detections_above_threshold) + ")");

            if (total_valid_detections == 0) {
                LOG_WARNING("  NO DETECTIONS FOUND! Check:");
                LOG_WARNING("    1. Is hand visible in frame?");
                LOG_WARNING("    2. Is threshold too high? (current: " + std::to_string(config.score_threshold) + ")");
                LOG_WARNING("    3. Is preprocessing correct? (BGR->RGB, normalization)");
                LOG_WARNING("    4. Is model output format parsed correctly?");
            }

            // Apply NMS
            std::vector<int> keep_indices = nms(boxes, scores);

            LOG_INFO("  After NMS: " + std::to_string(keep_indices.size()) + " detections kept");

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

                LOG_INFO("  HAND DETECTED: score=" + std::to_string(det.score) +
                        ", bbox=[" + std::to_string(det.bounding_box.x) + ", " +
                        std::to_string(det.bounding_box.y) + ", " +
                        std::to_string(det.bounding_box.width) + ", " +
                        std::to_string(det.bounding_box.height) + "]");
            }

        } catch (const std::exception& e) {
            last_error = std::string("Error parsing detections: ") + e.what();
            LOG_ERROR(last_error);
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
    static int detect_call_count = 0;
    detect_call_count++;

    if (!pImpl->initialized) {
        pImpl->last_error = "Detector not initialized";
        return false;
    }

    if (image.empty()) {
        pImpl->last_error = "Empty input image";
        return false;
    }

    // Log first few calls and periodically
    if (detect_call_count <= 5 || detect_call_count % 50 == 0) {
        LOG_INFO("HandDetector::detect() called #" + std::to_string(detect_call_count));
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

        // Log detection results
        if (detect_call_count <= 5 || detect_call_count % 50 == 0) {
            LOG_INFO("HandDetector::detect() completed in " +
                    std::to_string(pImpl->last_detection_time_ms) + "ms");
            LOG_INFO("  Returned " + std::to_string(detections.size()) + " detections");
        }

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
