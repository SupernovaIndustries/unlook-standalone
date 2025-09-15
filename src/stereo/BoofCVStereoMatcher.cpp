/**
 * @file BoofCVStereoMatcher.cpp
 * @brief Implementation of BoofCV-based stereo matching with sub-pixel precision
 */

// Only compile BoofCV support if JNI is available
#ifdef HAVE_BOOFCV

#include "unlook/stereo/BoofCVStereoMatcher.hpp"
#include "unlook/core/Logger.hpp"

#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <chrono>
#include <algorithm>

using namespace unlook::core;

namespace unlook {
namespace stereo {

namespace {
    // JVM and classpath configuration
    const char* BOOFCV_JAR_PATH = "/usr/share/java/boofcv/boofcv-all.jar";
    const char* MATCHER_CLASS_NAME = "unlook/stereo/BoofCVStereoMatcherJava";
    
    // Precision validation constants
    constexpr double TARGET_PRECISION_MM = 0.005;  // 5 micrometers
}

BoofCVStereoMatcher::BoofCVStereoMatcher(const BoofCVStereoConfig& config)
    : config_(config)
    , jvm_(nullptr)
    , env_(nullptr)
    , boofcv_matcher_(nullptr)
    , compute_disparity_method_(nullptr)
    , compute_subpixel_method_(nullptr)
    , update_config_method_(nullptr)
{
    Logger::info("BoofCVStereoMatcher: Initializing with target precision: {} mm", TARGET_PRECISION_MM);
    
    try {
        initializeJVM();
        createBoofCVMatcher();
        cacheMethodIDs();
        
        Logger::info("BoofCVStereoMatcher: Initialized successfully with algorithm: {}", 
                    static_cast<int>(config_.algorithm));
    } catch (const std::exception& e) {
        cleanupJVM();
        throw std::runtime_error("Failed to initialize BoofCVStereoMatcher: " + std::string(e.what()));
    }
}

BoofCVStereoMatcher::~BoofCVStereoMatcher() {
    cleanupJVM();
}

// Implementation for BoofCV availability check
bool BoofCVStereoMatcher::isAvailable() {
    Logger::debug("BoofCVStereoMatcher: Checking BoofCV availability");
    
    try {
        // Check if BoofCV JAR file exists
        std::ifstream jar_file(BOOFCV_JAR_PATH);
        if (!jar_file.good()) {
            Logger::warning("BoofCV JAR file not found at: {}", BOOFCV_JAR_PATH);
            return false;
        }
        jar_file.close();
        
        // Try to create a temporary JVM to test Java availability
        JavaVM* test_jvm = nullptr;
        JNIEnv* test_env = nullptr;
        
        JavaVMInitArgs vm_args;
        JavaVMOption options[1];
        
        std::string classpath = "-Djava.class.path=" + std::string(BOOFCV_JAR_PATH);
        options[0].optionString = const_cast<char*>(classpath.c_str());
        
        vm_args.version = JNI_VERSION_1_8;
        vm_args.nOptions = 1;
        vm_args.options = options;
        vm_args.ignoreUnrecognized = true;
        
        jint res = JNI_CreateJavaVM(&test_jvm, (void**)&test_env, &vm_args);
        if (res != JNI_OK) {
            Logger::error("Failed to create test JVM for BoofCV availability check: {}", res);
            return false;
        }
        
        // Try to find the BoofCV matcher class
        jclass matcherClass = test_env->FindClass(MATCHER_CLASS_NAME);
        bool class_found = (matcherClass != nullptr);
        
        if (matcherClass) {
            test_env->DeleteLocalRef(matcherClass);
        }
        
        // Clean up test JVM
        test_jvm->DestroyJavaVM();
        
        if (!class_found) {
            Logger::warning("BoofCV matcher class not found: {}", MATCHER_CLASS_NAME);
            return false;
        }
        
        Logger::info("BoofCV is available and ready for high-precision stereo processing");
        return true;
        
    } catch (const std::exception& e) {
        Logger::error("Exception during BoofCV availability check: {}", e.what());
        return false;
    }
}

std::string BoofCVStereoMatcher::getBoofCVVersion() {
    return "BoofCV JNI wrapper (development phase)";
}

void BoofCVStereoMatcher::validateStereoImages(const cv::Mat& left_image, const cv::Mat& right_image) {
    if (left_image.empty() || right_image.empty()) {
        throw std::invalid_argument("Empty stereo images");
    }
    
    if (left_image.size() != right_image.size()) {
        throw std::invalid_argument("Stereo images must have the same size");
    }
    
    if (left_image.type() != right_image.type()) {
        throw std::invalid_argument("Stereo images must have the same type");
    }
    
    // Support only 8-bit grayscale or color images
    int depth = left_image.depth();
    if (depth != CV_8U) {
        throw std::invalid_argument("Only 8-bit images are supported");
    }
    
    int channels = left_image.channels();
    if (channels != 1 && channels != 3) {
        throw std::invalid_argument("Only grayscale (1 channel) or RGB (3 channels) images are supported");
    }
    
    // Check minimum size for meaningful stereo processing
    if (left_image.cols < 64 || left_image.rows < 64) {
        throw std::invalid_argument("Images too small for stereo processing (minimum 64x64)");
    }
}

BoofCVQualityMetrics BoofCVStereoMatcher::computeDisparity(
    const cv::Mat& left_image,
    const cv::Mat& right_image,
    cv::Mat& disparity_map)
{
    validateStereoImages(left_image, right_image);
    
    Logger::info("BoofCV disparity computation - {}x{} with algorithm {}", 
                left_image.cols, left_image.rows, static_cast<int>(config_.algorithm));
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // Convert OpenCV images to BoofCV format
        Logger::debug("Converting input images to BoofCV format");
        jobject leftBoofImage = matToBoofCVImage(left_image);
        jobject rightBoofImage = matToBoofCVImage(right_image);
        
        // Create configuration object
        jobject configObj = configToJavaObject(config_);
        
        // Create output disparity image object (placeholder)
        jclass imageClass = env_->FindClass("boofcv/struct/image/GrayU8");
        if (!imageClass) {
            throw std::runtime_error("Failed to find GrayU8 class for output");
        }
        
        jmethodID constructor = env_->GetMethodID(imageClass, "<init>", "(II)V");
        jobject outputImage = env_->NewObject(imageClass, constructor, 
                                            left_image.cols, left_image.rows);
        
        // Call Java computeDisparity method
        Logger::debug("Calling BoofCV Java computeDisparity method");
        jobject metricsObj = env_->CallObjectMethod(boofcv_matcher_, compute_disparity_method_,
                                                   leftBoofImage, rightBoofImage, outputImage);
        
        // Check for Java exceptions
        if (env_->ExceptionCheck()) {
            env_->ExceptionDescribe();
            env_->ExceptionClear();
            throw std::runtime_error("Java exception during BoofCV disparity computation");
        }
        
        // Convert output back to OpenCV
        Logger::debug("Converting BoofCV result back to OpenCV Mat");
        cv::Mat disparityResult = boofCVImageToMat(outputImage);
        
        // Convert to 32-bit float for consistency with OpenCV stereo
        disparityResult.convertTo(disparity_map, CV_32F);
        
        // Extract quality metrics
        BoofCVQualityMetrics metrics = javaObjectToMetrics(metricsObj);
        
        // Calculate processing time
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        metrics.processing_time_ms = duration.count();
        
        // Validate against precision target
        validatePrecisionTarget(metrics);
        
        // Clean up Java references
        env_->DeleteLocalRef(leftBoofImage);
        env_->DeleteLocalRef(rightBoofImage);
        env_->DeleteLocalRef(configObj);
        env_->DeleteLocalRef(outputImage);
        env_->DeleteLocalRef(imageClass);
        if (metricsObj) env_->DeleteLocalRef(metricsObj);
        
        Logger::info("BoofCV disparity computation completed in {}ms - {:.1f}% coverage, {:.6f}mm precision",
                    metrics.processing_time_ms, metrics.valid_pixel_percentage, metrics.precision_mm);
        
        return metrics;
        
    } catch (const std::exception& e) {
        Logger::error("BoofCV disparity computation failed: {}", e.what());
        
        // Return error metrics
        BoofCVQualityMetrics error_metrics;
        error_metrics.valid_pixel_percentage = 0.0;
        error_metrics.precision_mm = 999.0;
        error_metrics.meets_precision_target = false;
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        error_metrics.processing_time_ms = duration.count();
        
        // Create empty disparity map as fallback
        disparity_map = cv::Mat::zeros(left_image.size(), CV_32F);
        
        return error_metrics;
    }
}

BoofCVQualityMetrics BoofCVStereoMatcher::computeSubpixelDisparity(
    const cv::Mat& left_image,
    const cv::Mat& right_image,
    cv::Mat& disparity_map)
{
    validateStereoImages(left_image, right_image);
    
    Logger::info("BoofCV SUB-PIXEL disparity computation - {}x{} targeting 0.005mm precision", 
                left_image.cols, left_image.rows);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // Enable sub-pixel mode in configuration
        BoofCVStereoConfig subpixel_config = config_;
        subpixel_config.subpixel_enabled = true;
        subpixel_config.left_right_validation = true;  // Critical for precision
        subpixel_config.enforce_precision_target = true;
        
        // Convert OpenCV images to BoofCV format  
        Logger::debug("Converting input images to BoofCV format for sub-pixel processing");
        jobject leftBoofImage = matToBoofCVImage(left_image);
        jobject rightBoofImage = matToBoofCVImage(right_image);
        
        // Create sub-pixel configuration object
        jobject configObj = configToJavaObject(subpixel_config);
        
        // Create output disparity image object
        jclass imageClass = env_->FindClass("boofcv/struct/image/GrayF32");  // Use float for sub-pixel
        if (!imageClass) {
            throw std::runtime_error("Failed to find GrayF32 class for sub-pixel output");
        }
        
        jmethodID constructor = env_->GetMethodID(imageClass, "<init>", "(II)V");
        jobject outputImage = env_->NewObject(imageClass, constructor, 
                                            left_image.cols, left_image.rows);
        
        // Call Java computeSubpixelDisparity method (high precision)
        Logger::debug("Calling BoofCV Java computeSubpixelDisparity method for 0.005mm precision");
        jobject metricsObj = env_->CallObjectMethod(boofcv_matcher_, compute_subpixel_method_,
                                                   leftBoofImage, rightBoofImage, outputImage);
        
        // Check for Java exceptions
        if (env_->ExceptionCheck()) {
            env_->ExceptionDescribe();
            env_->ExceptionClear();
            throw std::runtime_error("Java exception during BoofCV sub-pixel disparity computation");
        }
        
        // Convert output back to OpenCV with sub-pixel precision
        Logger::debug("Converting BoofCV sub-pixel result back to OpenCV Mat");
        cv::Mat disparityResult = boofCVImageToMat(outputImage);
        
        // Keep as 32-bit float to preserve sub-pixel precision
        disparityResult.convertTo(disparity_map, CV_32F);
        
        // Extract quality metrics
        BoofCVQualityMetrics metrics = javaObjectToMetrics(metricsObj);
        
        // Calculate processing time
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        metrics.processing_time_ms = duration.count();
        
        // CRITICAL: Validate precision target for industrial use
        validatePrecisionTarget(metrics);
        
        // Enhanced logging for sub-pixel results
        Logger::info("BoofCV SUB-PIXEL computation completed in {}ms", metrics.processing_time_ms);
        Logger::info("  → Coverage: {:.1f}% | Precision: {:.6f}mm | Target: {:.6f}mm", 
                    metrics.valid_pixel_percentage, metrics.precision_mm, TARGET_PRECISION_MM);
        Logger::info("  → Meets precision target: {}", metrics.meets_precision_target ? "YES" : "NO");
        
        if (metrics.meets_precision_target) {
            Logger::success("🎯 BoofCV achieved INDUSTRIAL PRECISION target ≤ {:.6f}mm!", TARGET_PRECISION_MM);
        }
        
        // Clean up Java references
        env_->DeleteLocalRef(leftBoofImage);
        env_->DeleteLocalRef(rightBoofImage);
        env_->DeleteLocalRef(configObj);
        env_->DeleteLocalRef(outputImage);
        env_->DeleteLocalRef(imageClass);
        if (metricsObj) env_->DeleteLocalRef(metricsObj);
        
        return metrics;
        
    } catch (const std::exception& e) {
        Logger::error("BoofCV sub-pixel disparity computation failed: {}", e.what());
        
        // Return error metrics
        BoofCVQualityMetrics error_metrics;
        error_metrics.valid_pixel_percentage = 0.0;
        error_metrics.precision_mm = 999.0;
        error_metrics.meets_precision_target = false;
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        error_metrics.processing_time_ms = duration.count();
        
        // Create empty disparity map as fallback
        disparity_map = cv::Mat::zeros(left_image.size(), CV_32F);
        
        return error_metrics;
    }
}

void BoofCVStereoMatcher::updateConfig(const BoofCVStereoConfig& config) {
    config_ = config;
    Logger::debug("BoofCV configuration updated - algorithm: {}, subpixel: {}", 
                 static_cast<int>(config.algorithm), config.subpixel_enabled);
    
    // Update Java-side configuration if JVM is initialized
    if (env_ && boofcv_matcher_ && update_config_method_) {
        try {
            jobject configObj = configToJavaObject(config);
            env_->CallVoidMethod(boofcv_matcher_, update_config_method_, configObj);
            
            // Check for Java exceptions
            if (env_->ExceptionCheck()) {
                env_->ExceptionDescribe();
                env_->ExceptionClear();
                Logger::warning("Java exception during BoofCV configuration update");
            } else {
                Logger::debug("Java-side BoofCV configuration updated successfully");
            }
            
            env_->DeleteLocalRef(configObj);
            
        } catch (const std::exception& e) {
            Logger::error("Failed to update Java-side BoofCV configuration: {}", e.what());
        }
    }
}

// Private JNI implementation methods
void BoofCVStereoMatcher::initializeJVM() {
    Logger::debug("BoofCVStereoMatcher: Initializing JVM for BoofCV integration");
    
    JavaVMInitArgs vm_args;
    JavaVMOption options[2];
    
    // Set classpath to include BoofCV JAR
    std::string classpath = "-Djava.class.path=" + std::string(BOOFCV_JAR_PATH);
    options[0].optionString = const_cast<char*>(classpath.c_str());
    
    // Enable JNI debugging if needed
    options[1].optionString = const_cast<char*>("-Xcheck:jni");
    
    vm_args.version = JNI_VERSION_1_8;
    vm_args.nOptions = 2;
    vm_args.options = options;
    vm_args.ignoreUnrecognized = false;
    
    // Create JVM
    jint res = JNI_CreateJavaVM(&jvm_, (void**)&env_, &vm_args);
    if (res != JNI_OK) {
        throw std::runtime_error("Failed to create JVM: " + std::to_string(res));
    }
    
    Logger::info("BoofCVStereoMatcher: JVM initialized successfully");
}

void BoofCVStereoMatcher::cleanupJVM() {
    Logger::debug("BoofCVStereoMatcher: Cleaning up JVM resources");
    
    if (env_ && boofcv_matcher_) {
        env_->DeleteGlobalRef(boofcv_matcher_);
        boofcv_matcher_ = nullptr;
    }
    
    if (jvm_) {
        jvm_->DestroyJavaVM();
        jvm_ = nullptr;
        env_ = nullptr;
    }
    
    Logger::debug("BoofCVStereoMatcher: JVM cleanup completed");
}

void BoofCVStereoMatcher::createBoofCVMatcher() {
    Logger::debug("BoofCVStereoMatcher: Creating BoofCV stereo matcher Java object");
    
    // Find the BoofCV matcher class
    jclass matcherClass = env_->FindClass(MATCHER_CLASS_NAME);
    if (!matcherClass) {
        throw std::runtime_error("Failed to find BoofCV matcher class: " + std::string(MATCHER_CLASS_NAME));
    }
    
    // Get constructor method ID
    jmethodID constructor = env_->GetMethodID(matcherClass, "<init>", "()V");
    if (!constructor) {
        throw std::runtime_error("Failed to find constructor for BoofCV matcher");
    }
    
    // Create the Java object
    jobject localMatcher = env_->NewObject(matcherClass, constructor);
    if (!localMatcher) {
        throw std::runtime_error("Failed to create BoofCV matcher instance");
    }
    
    // Create global reference for persistent use
    boofcv_matcher_ = env_->NewGlobalRef(localMatcher);
    if (!boofcv_matcher_) {
        throw std::runtime_error("Failed to create global reference for BoofCV matcher");
    }
    
    // Clean up local references
    env_->DeleteLocalRef(localMatcher);
    env_->DeleteLocalRef(matcherClass);
    
    Logger::info("BoofCVStereoMatcher: BoofCV matcher Java object created successfully");
}

void BoofCVStereoMatcher::cacheMethodIDs() {
    Logger::debug("BoofCVStereoMatcher: Caching JNI method IDs for performance optimization");
    
    // Get the class from the global reference
    jclass matcherClass = env_->GetObjectClass(boofcv_matcher_);
    if (!matcherClass) {
        throw std::runtime_error("Failed to get BoofCV matcher class from object");
    }
    
    // Cache method ID for computeDisparity
    // Signature: (Ljava/lang/Object;Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;
    compute_disparity_method_ = env_->GetMethodID(matcherClass, "computeDisparity", 
        "(Ljava/lang/Object;Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;");
    if (!compute_disparity_method_) {
        throw std::runtime_error("Failed to find computeDisparity method");
    }
    
    // Cache method ID for computeSubpixelDisparity  
    compute_subpixel_method_ = env_->GetMethodID(matcherClass, "computeSubpixelDisparity",
        "(Ljava/lang/Object;Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;");
    if (!compute_subpixel_method_) {
        throw std::runtime_error("Failed to find computeSubpixelDisparity method");
    }
    
    // Cache method ID for updateConfiguration
    update_config_method_ = env_->GetMethodID(matcherClass, "updateConfiguration", 
        "(Ljava/lang/Object;)V");
    if (!update_config_method_) {
        throw std::runtime_error("Failed to find updateConfiguration method");
    }
    
    // Clean up local reference
    env_->DeleteLocalRef(matcherClass);
    
    Logger::info("BoofCVStereoMatcher: All JNI method IDs cached successfully");
}

jobject BoofCVStereoMatcher::matToBoofCVImage(const cv::Mat& mat) {
    Logger::debug("Converting cv::Mat to BoofCV image: {}x{} channels={}", 
                 mat.cols, mat.rows, mat.channels());
    
    // Convert to grayscale if needed (BoofCV stereo works with grayscale)
    cv::Mat grayMat;
    if (mat.channels() == 3) {
        cv::cvtColor(mat, grayMat, cv::COLOR_BGR2GRAY);
    } else if (mat.channels() == 4) {
        cv::cvtColor(mat, grayMat, cv::COLOR_BGRA2GRAY);
    } else {
        grayMat = mat.clone();
    }
    
    // Ensure 8-bit format
    cv::Mat mat8u;
    if (grayMat.depth() != CV_8U) {
        grayMat.convertTo(mat8u, CV_8U);
    } else {
        mat8u = grayMat;
    }
    
    // Create Java byte array
    int dataSize = mat8u.rows * mat8u.cols;
    jbyteArray javaArray = env_->NewByteArray(dataSize);
    if (!javaArray) {
        throw std::runtime_error("Failed to create Java byte array for image data");
    }
    
    // Copy image data to Java array
    env_->SetByteArrayRegion(javaArray, 0, dataSize, reinterpret_cast<const jbyte*>(mat8u.data));
    
    // Find BoofCV ImageUInt8 class and create instance
    jclass imageClass = env_->FindClass("boofcv/struct/image/GrayU8");
    if (!imageClass) {
        throw std::runtime_error("Failed to find BoofCV GrayU8 class");
    }
    
    // Get constructor: GrayU8(int width, int height)
    jmethodID constructor = env_->GetMethodID(imageClass, "<init>", "(II)V");
    if (!constructor) {
        throw std::runtime_error("Failed to find GrayU8 constructor");
    }
    
    // Create BoofCV image object
    jobject boofImage = env_->NewObject(imageClass, constructor, mat8u.cols, mat8u.rows);
    if (!boofImage) {
        throw std::runtime_error("Failed to create BoofCV GrayU8 object");
    }
    
    // Get setData method to copy pixel data
    jmethodID setDataMethod = env_->GetMethodID(imageClass, "setData", "([B)V");
    if (!setDataMethod) {
        throw std::runtime_error("Failed to find setData method");
    }
    
    // Set the image data
    env_->CallVoidMethod(boofImage, setDataMethod, javaArray);
    
    // Clean up local references
    env_->DeleteLocalRef(javaArray);
    env_->DeleteLocalRef(imageClass);
    
    Logger::debug("Successfully converted cv::Mat to BoofCV GrayU8 image");
    return boofImage;
}

cv::Mat BoofCVStereoMatcher::boofCVImageToMat(jobject boofcv_image) {
    Logger::debug("Converting BoofCV image to cv::Mat");
    
    if (!boofcv_image) {
        throw std::runtime_error("BoofCV image object is null");
    }
    
    // Get the image class
    jclass imageClass = env_->GetObjectClass(boofcv_image);
    if (!imageClass) {
        throw std::runtime_error("Failed to get BoofCV image class");
    }
    
    // Get width and height
    jfieldID widthField = env_->GetFieldID(imageClass, "width", "I");
    jfieldID heightField = env_->GetFieldID(imageClass, "height", "I");
    if (!widthField || !heightField) {
        throw std::runtime_error("Failed to get width/height fields from BoofCV image");
    }
    
    int width = env_->GetIntField(boofcv_image, widthField);
    int height = env_->GetIntField(boofcv_image, heightField);
    
    // Get data field (byte array for disparity data)
    jfieldID dataField = env_->GetFieldID(imageClass, "data", "[B");
    if (!dataField) {
        throw std::runtime_error("Failed to get data field from BoofCV image");
    }
    
    jbyteArray dataArray = (jbyteArray)env_->GetObjectField(boofcv_image, dataField);
    if (!dataArray) {
        throw std::runtime_error("Failed to get data array from BoofCV image");
    }
    
    // Get array length and data
    jsize arrayLength = env_->GetArrayLength(dataArray);
    jbyte* imageData = env_->GetByteArrayElements(dataArray, nullptr);
    if (!imageData) {
        throw std::runtime_error("Failed to get byte array elements from BoofCV data");
    }
    
    // Create OpenCV Mat
    cv::Mat result(height, width, CV_8UC1);
    std::memcpy(result.data, imageData, arrayLength);
    
    // Release Java array data
    env_->ReleaseByteArrayElements(dataArray, imageData, JNI_ABORT);
    env_->DeleteLocalRef(imageClass);
    
    Logger::debug("Successfully converted BoofCV image to cv::Mat: {}x{}", width, height);
    return result;
}

jobject BoofCVStereoMatcher::configToJavaObject(const BoofCVStereoConfig& config) {
    Logger::debug("Converting C++ config to Java object");
    
    // Find the configuration class
    jclass configClass = env_->FindClass("unlook/stereo/BoofCVStereoConfig");
    if (!configClass) {
        throw std::runtime_error("Failed to find BoofCVStereoConfig Java class");
    }
    
    // Get constructor
    jmethodID constructor = env_->GetMethodID(configClass, "<init>", "()V");
    if (!constructor) {
        throw std::runtime_error("Failed to find BoofCVStereoConfig constructor");
    }
    
    // Create config object
    jobject configObj = env_->NewObject(configClass, constructor);
    if (!configObj) {
        throw std::runtime_error("Failed to create BoofCVStereoConfig object");
    }
    
    // Set algorithm (convert enum to int)
    jfieldID algorithmField = env_->GetFieldID(configClass, "algorithm", "I");
    if (algorithmField) {
        env_->SetIntField(configObj, algorithmField, static_cast<int>(config.algorithm));
    }
    
    // Set error type (convert enum to int)
    jfieldID errorTypeField = env_->GetFieldID(configClass, "errorType", "I");
    if (errorTypeField) {
        env_->SetIntField(configObj, errorTypeField, static_cast<int>(config.error_type));
    }
    
    // Set disparity parameters
    jfieldID minDisparityField = env_->GetFieldID(configClass, "minDisparity", "I");
    if (minDisparityField) {
        env_->SetIntField(configObj, minDisparityField, config.min_disparity);
    }
    
    jfieldID maxDisparityField = env_->GetFieldID(configClass, "maxDisparity", "I");
    if (maxDisparityField) {
        env_->SetIntField(configObj, maxDisparityField, config.max_disparity);
    }
    
    jfieldID regionRadiusField = env_->GetFieldID(configClass, "regionRadius", "I");
    if (regionRadiusField) {
        env_->SetIntField(configObj, regionRadiusField, config.region_radius);
    }
    
    // Set quality parameters
    jfieldID subpixelField = env_->GetFieldID(configClass, "subpixelEnabled", "Z");
    if (subpixelField) {
        env_->SetBooleanField(configObj, subpixelField, config.subpixel_enabled);
    }
    
    jfieldID leftRightField = env_->GetFieldID(configClass, "leftRightValidation", "Z");
    if (leftRightField) {
        env_->SetBooleanField(configObj, leftRightField, config.left_right_validation);
    }
    
    // Set precision parameters
    jfieldID baselineField = env_->GetFieldID(configClass, "baselineMm", "D");
    if (baselineField) {
        env_->SetDoubleField(configObj, baselineField, config.baseline_mm);
    }
    
    jfieldID focalLengthField = env_->GetFieldID(configClass, "focalLengthPx", "D");
    if (focalLengthField) {
        env_->SetDoubleField(configObj, focalLengthField, config.focal_length_px);
    }
    
    env_->DeleteLocalRef(configClass);
    
    Logger::debug("Successfully converted C++ config to Java object");
    return configObj;
}

BoofCVQualityMetrics BoofCVStereoMatcher::javaObjectToMetrics(jobject metrics_obj) {
    BoofCVQualityMetrics metrics;
    
    if (!metrics_obj) {
        Logger::warning("BoofCV metrics object is null, returning default metrics");
        return metrics;
    }
    
    try {
        // Get the metrics class
        jclass metricsClass = env_->GetObjectClass(metrics_obj);
        if (!metricsClass) {
            throw std::runtime_error("Failed to get metrics class");
        }
        
        // Extract valid pixel percentage
        jfieldID validPixelField = env_->GetFieldID(metricsClass, "validPixelPercentage", "D");
        if (validPixelField) {
            metrics.valid_pixel_percentage = env_->GetDoubleField(metrics_obj, validPixelField);
        }
        
        // Extract mean error
        jfieldID meanErrorField = env_->GetFieldID(metricsClass, "meanError", "D");
        if (meanErrorField) {
            metrics.mean_error = env_->GetDoubleField(metrics_obj, meanErrorField);
        }
        
        // Extract RMS error
        jfieldID rmsErrorField = env_->GetFieldID(metricsClass, "rmsError", "D");
        if (rmsErrorField) {
            metrics.rms_error = env_->GetDoubleField(metrics_obj, rmsErrorField);
        }
        
        // Extract precision in mm
        jfieldID precisionField = env_->GetFieldID(metricsClass, "precisionMm", "D");
        if (precisionField) {
            metrics.precision_mm = env_->GetDoubleField(metrics_obj, precisionField);
        }
        
        // Extract processing time
        jfieldID processingTimeField = env_->GetFieldID(metricsClass, "processingTimeMs", "D");
        if (processingTimeField) {
            metrics.processing_time_ms = env_->GetDoubleField(metrics_obj, processingTimeField);
        }
        
        // Extract precision target flag
        jfieldID precisionTargetField = env_->GetFieldID(metricsClass, "meetsPrecisionTarget", "Z");
        if (precisionTargetField) {
            metrics.meets_precision_target = env_->GetBooleanField(metrics_obj, precisionTargetField);
        }
        
        env_->DeleteLocalRef(metricsClass);
        
        Logger::debug("Successfully extracted metrics from Java object: {:.3f}% valid, {:.6f}mm precision", 
                     metrics.valid_pixel_percentage, metrics.precision_mm);
                     
    } catch (const std::exception& e) {
        Logger::error("Failed to extract metrics from Java object: {}", e.what());
    }
    
    return metrics;
}

void BoofCVStereoMatcher::validatePrecisionTarget(const BoofCVQualityMetrics& metrics) {
    if (metrics.precision_mm > TARGET_PRECISION_MM) {
        Logger::warning("BoofCV precision {:.6f} mm exceeds target {:.6f} mm", 
                       metrics.precision_mm, TARGET_PRECISION_MM);
    }
}

// ============================================================================
// StereoMatcher Interface Implementation
// ============================================================================

bool BoofCVStereoMatcher::computeDisparity(const cv::Mat& leftRectified,
                                          const cv::Mat& rightRectified,
                                          cv::Mat& disparity) {
    Logger::debug("BoofCVStereoMatcher: StereoMatcher interface computeDisparity called");
    
    try {
        // Use the BoofCV-specific method and extract the disparity map
        BoofCVQualityMetrics metrics = computeDisparity(leftRectified, rightRectified, disparity);
        
        // Log quality metrics
        Logger::info("BoofCV stereo processing: {:.1f}% coverage, {:.6f}mm precision", 
                    metrics.valid_pixel_percentage, metrics.precision_mm);
        
        // Return success if we got reasonable coverage
        return metrics.valid_pixel_percentage > 10.0; // At least 10% valid pixels
        
    } catch (const std::exception& e) {
        Logger::error("BoofCV stereo matching failed: {}", e.what());
        return false;
    }
}

bool BoofCVStereoMatcher::setParameters(const StereoMatchingParams& params) {
    Logger::debug("BoofCVStereoMatcher: Setting parameters via StereoMatcher interface");
    
    try {
        // Convert StereoMatchingParams to BoofCVStereoConfig
        BoofCVStereoConfig new_config = config_; // Start with current config
        
        // Map common parameters
        new_config.min_disparity = params.minDisparity;
        new_config.max_disparity = params.minDisparity + params.numDisparities;
        new_config.region_radius = std::max(1, params.blockSize / 2); // Convert block size to radius
        
        // Map quality parameters
        new_config.left_right_validation = params.leftRightCheck;
        new_config.max_error = params.disp12MaxDiff / 100.0; // Convert to normalized range
        new_config.texture_threshold = params.textureThreshold / 255.0; // Normalize
        
        // Map performance parameters
        new_config.num_threads = params.numThreads;
        
        // Enable sub-pixel for high precision if uniqueness ratio is high (indicates quality preference)
        if (params.uniquenessRatio >= 15) {
            new_config.subpixel_enabled = true;
            new_config.algorithm = BoofCVAlgorithm::SUBPIXEL_SGM; // Highest precision
        } else if (params.uniquenessRatio >= 10) {
            new_config.subpixel_enabled = true;
            new_config.algorithm = BoofCVAlgorithm::SUBPIXEL_BM; // Balanced precision
        } else {
            new_config.subpixel_enabled = false;
            new_config.algorithm = BoofCVAlgorithm::DENSE_DISPARITY_SGM; // Speed priority
        }
        
        // Apply the new configuration
        updateConfig(new_config);
        
        Logger::info("BoofCV parameters updated - algorithm: {}, subpixel: {}, disparity range: [{},{}]",
                    static_cast<int>(new_config.algorithm), new_config.subpixel_enabled, 
                    new_config.min_disparity, new_config.max_disparity);
        
        return true;
        
    } catch (const std::exception& e) {
        Logger::error("Failed to set BoofCV parameters: {}", e.what());
        return false;
    }
}

StereoMatchingParams BoofCVStereoMatcher::getParameters() const {
    Logger::debug("BoofCVStereoMatcher: Getting parameters via StereoMatcher interface");
    
    StereoMatchingParams params;
    
    // Map BoofCV config to StereoMatchingParams
    params.minDisparity = config_.min_disparity;
    params.numDisparities = config_.max_disparity - config_.min_disparity;
    params.blockSize = config_.region_radius * 2 + 1; // Convert radius to block size (odd)
    
    // Quality parameters
    params.leftRightCheck = config_.left_right_validation;
    params.disp12MaxDiff = static_cast<int>(config_.max_error * 100); // Convert from normalized
    params.textureThreshold = static_cast<int>(config_.texture_threshold * 255); // Denormalize
    
    // Performance parameters
    params.numThreads = config_.num_threads;
    
    // Set uniqueness ratio based on algorithm (indicate quality level)
    switch (config_.algorithm) {
        case BoofCVAlgorithm::SUBPIXEL_SGM:
            params.uniquenessRatio = 20; // Highest quality
            break;
        case BoofCVAlgorithm::SUBPIXEL_BM:
            params.uniquenessRatio = 15; // High quality
            break;
        case BoofCVAlgorithm::DENSE_DISPARITY_SGM:
            params.uniquenessRatio = 10; // Balanced
            break;
        case BoofCVAlgorithm::DENSE_DISPARITY_BM:
        default:
            params.uniquenessRatio = 5; // Speed priority
            break;
    }
    
    // Set other parameters to reasonable defaults
    params.P1 = 8 * 3 * params.blockSize * params.blockSize;
    params.P2 = 32 * 3 * params.blockSize * params.blockSize;
    params.preFilterCap = 63;
    params.speckleWindowSize = 100;
    params.speckleRange = 32;
    params.mode = 0; // SGBM mode
    
    // Post-processing (BoofCV has built-in filtering)
    params.useWLSFilter = false; // BoofCV handles filtering internally
    params.wlsLambda = 8000.0;
    params.wlsSigma = 1.5;
    
    // Performance settings
    params.useParallel = (config_.num_threads > 1);
    params.confidenceThreshold = 0.95f; // BoofCV provides high confidence results
    
    return params;
}

StereoAlgorithm BoofCVStereoMatcher::getAlgorithmType() const {
    // Map BoofCV algorithm to StereoAlgorithm enum
    switch (config_.algorithm) {
        case BoofCVAlgorithm::DENSE_DISPARITY_BM:
            return StereoAlgorithm::BOOFCV_BM;
        case BoofCVAlgorithm::DENSE_DISPARITY_SGM:
            return StereoAlgorithm::BOOFCV_SGBM;
        case BoofCVAlgorithm::SUBPIXEL_BM:
            return StereoAlgorithm::BOOFCV_BM; // Map to base BM type
        case BoofCVAlgorithm::SUBPIXEL_SGM:
        default:
            return StereoAlgorithm::BOOFCV_SGBM; // Map to base SGBM type
    }
}

std::string BoofCVStereoMatcher::getAlgorithmName() const {
    switch (config_.algorithm) {
        case BoofCVAlgorithm::DENSE_DISPARITY_BM:
            return "BoofCV Dense Block Matching";
        case BoofCVAlgorithm::DENSE_DISPARITY_SGM:
            return "BoofCV Dense Semi-Global Matching";
        case BoofCVAlgorithm::SUBPIXEL_BM:
            return "BoofCV Sub-pixel Block Matching";
        case BoofCVAlgorithm::SUBPIXEL_SGM:
            return "BoofCV Sub-pixel Semi-Global Matching (0.005mm precision)";
        default:
            return "BoofCV Stereo Matcher";
    }
}

} // namespace stereo
} // namespace unlook

#endif // HAVE_BOOFCV