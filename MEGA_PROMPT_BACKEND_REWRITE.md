# MEGA-PROMPT: COMPLETE BACKEND REWRITE - STEREO PROCESSING PIPELINE

## 🎯 MISSION CRITICAL

**PROBLEMA CRITICO:** Backend di stereo processing (rettifica → disparity → depth → PLY) produce risultati distorti e inaffidabili. 3 giorni fa funzionava, ora è completamente rotto.

**SOLUZIONE:** **CANCELLARE E RICOSTRUIRE DA ZERO** l'intero backend di processing seguendo best practices dei competitor (RealSense, Luxonis, OpenCV-contrib) e sfruttando al **MASSIMO la GPU Raspberry Pi 5**.

**COSA MANTENERE:** ✅
- Camera system (funziona bene)
- Hardware sync (funziona bene)
- VCSEL controller AS1170 (funziona)
- GUI framework (HandheldScanWidget, CalibrationWidget)

**COSA CANCELLARE E RIFARE DA ZERO:** ❌→✅
- `VCSELStereoMatcher.cpp` - ELIMINARE COMPLETAMENTE
- `HandheldScanPipeline.cpp` - RICOSTRUIRE DA ZERO
- Tutto il codice di rettifica, disparity, depth conversion
- Tutti i file NEON (`census_neon.cpp`, `hamming_neon.cpp`)

**TARGET FINALE:**
- **Resolution:** 1280x720 (calibrazione DEVE matchare!)
- **Performance:** 10+ FPS per frame stereo processing
- **Precision:** 500mm @ 0.1mm, 1000mm @ 0.5mm
- **GPU Utilization:** >70% VideoCore VII usage
- **Debug Output:** Salvare TUTTE le immagini intermedie

---

## 🔴 CALIBRATION CRITICAL SECTION

### **PROBLEMA IDENTIFICATO:**
```yaml
# CALIBRAZIONE VECCHIA (SBAGLIATA!)
/unlook_calib/test_old.yaml:
  image_size: [ 1456, 1088 ]  # ← NATIVE IMX296, ma dataset era 1280x720!

# CALIBRAZIONE NUOVA (CORRETTA)
/unlook_calib/calib-20251108_171118.yaml:
  image_width: 1280
  image_height: 720
```

**ROOT CAUSE:** Rectification maps calcolate per risoluzione SBAGLIATA → immagini rettificate distorte → stereo matching fallisce → depth maps inutilizzabili.

### **MANDATORY CALIBRATION VALIDATION (PRIMA DI OGNI PROCESSING):**

```cpp
namespace unlook::calibration {

/**
 * @brief CRITICAL: Validate calibration matches image resolution
 *
 * This function MUST be called before ANY stereo processing!
 * If validation fails, ABORT immediately with clear error message.
 */
struct CalibrationValidation {
    // Image resolution consistency
    cv::Size calibImageSize;       // From YAML: image_width × image_height
    cv::Size actualImageSize;      // From camera: actual captured size
    bool resolutionMatch;          // MUST be true!

    // Rectification maps consistency
    cv::Size map1LeftSize;         // map1Left.size()
    cv::Size map2LeftSize;         // map2Left.size()
    cv::Size map1RightSize;        // map1Right.size()
    cv::Size map2RightSize;        // map2Right.size()
    bool mapsConsistent;           // ALL maps must match calibImageSize

    // Camera intrinsics validation
    double fxLeft, fyLeft, cxLeft, cyLeft;
    double fxRight, fyRight, cxRight, cyRight;
    bool intrinsicsValid;          // fx,fy > 0, cx,cy within image bounds

    // Stereo extrinsics validation
    double baselineMM;             // From T vector, should be ~70mm
    bool baselineValid;            // 60mm < baseline < 80mm

    // Rectification quality
    double cyLeftRect, cyRightRect;   // From P1, P2 projection matrices
    double cyDifference;              // MUST be < 0.1 pixels (epipolar alignment!)
    bool epipolarAligned;

    // Overall validation
    bool allChecksPass;
    std::string errorMessage;

    /**
     * @brief Validate calibration BEFORE any processing
     * @return true if ALL checks pass, false otherwise
     */
    static CalibrationValidation validate(
        const CalibrationManager& calib,
        const cv::Size& actualImageSize);

    /**
     * @brief Print detailed validation report
     */
    void printReport() const;
};

} // namespace unlook::calibration
```

### **MANDATORY CHECKS IN CODE:**

```cpp
// IN HandheldScanPipeline::initialize()
auto validation = CalibrationValidation::validate(calibrationManager_, imageSize);

if (!validation.allChecksPass) {
    logger_.error("╔════════════════════════════════════════╗");
    logger_.error("║  CALIBRATION VALIDATION FAILED!        ║");
    logger_.error("╚════════════════════════════════════════╝");
    logger_.error(validation.errorMessage);
    validation.printReport();
    throw core::Exception("Calibration validation failed: " + validation.errorMessage);
}

logger_.info("✓ Calibration validation PASSED");
logger_.info("  - Image size: " + std::to_string(validation.calibImageSize.width) + "x" +
                              std::to_string(validation.calibImageSize.height));
logger_.info("  - Baseline: " + std::to_string(validation.baselineMM) + " mm");
logger_.info("  - Epipolar alignment: " + std::to_string(validation.cyDifference) + " px");
```

### **CALIBRATION FIELDS TO CHECK (EXHAUSTIVE LIST):**

```yaml
# MANDATORY FIELDS (MUST exist and be valid)
image_width: 1280           # OR image_size: [1280, 720]
image_height: 720
baseline_mm: 70.017         # 60-80mm range
rms_error: 0.169            # < 0.5 pixels

# Camera intrinsics (LEFT)
camera_matrix_left:         # 3x3, fx fy > 0, cx cy within bounds
  - [ fx,  0, cx ]
  - [  0, fy, cy ]
  - [  0,  0,  1 ]
distortion_coeffs_left:     # 5x1, radial k1 k2, tangential p1 p2, k3

# Camera intrinsics (RIGHT)
camera_matrix_right:        # Same checks as LEFT
distortion_coeffs_right:

# Stereo extrinsics
rotation_matrix:            # 3x3, det(R) ≈ 1.0
translation_vector:         # 3x1, T[0] = baseline (mm)
essential_matrix:           # 3x3
fundamental_matrix:         # 3x3

# Rectification transforms
rectification_transform_left:    # 3x3 rotation matrix R1
rectification_transform_right:   # 3x3 rotation matrix R2
projection_matrix_left:          # 3x4, P1[1,2] = cy_left
projection_matrix_right:         # 3x4, P2[1,2] = cy_right, CHECK: cy_left ≈ cy_right!
disparity_to_depth_matrix:       # 4x4 Q matrix

# CRITICAL: P1 and P2 MUST have same cy (row 1, col 2)!
# If cy_left != cy_right, epipolar lines NOT horizontal → SGBM fails!
```

---

## 🚀 GPU ACCELERATION - RASPBERRY PI 5 MAXIMUM PERFORMANCE

### **Hardware Specs:**
- **GPU:** VideoCore VII (Broadcom BCM2712)
- **Vulkan:** 1.2 support
- **OpenCL:** NOT supported on VideoCore VII (only VC4CL on older Pi)
- **Memory:** Unified memory architecture (zero-copy!)

### **STRATEGY:**
1. **Primary:** OpenCV CUDA backend (if available via custom build)
2. **Secondary:** Vulkan compute shaders for SGM aggregation
3. **Fallback:** ARM NEON CPU (già implementato)

### **INSTALLATION & SETUP:**

```bash
# 1. Check Vulkan support
sudo apt-get update
sudo apt-get install -y vulkan-tools mesa-vulkan-drivers

# Verify Vulkan
vulkaninfo | grep "deviceName"
# Should show: "V3D 7.1.x" or similar VideoCore VII

# 2. Install OpenCV with GPU support (custom build)
cd /tmp
git clone --depth 1 --branch 4.10.0 https://github.com/opencv/opencv.git
git clone --depth 1 --branch 4.10.0 https://github.com/opencv/opencv_contrib.git

mkdir opencv/build && cd opencv/build

cmake \
  -D CMAKE_BUILD_TYPE=Release \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
  -D WITH_VULKAN=ON \
  -D WITH_OPENCL=OFF \
  -D WITH_V4L=ON \
  -D ENABLE_NEON=ON \
  -D CPU_BASELINE=NEON \
  -D ENABLE_VFPV3=ON \
  -D WITH_TBB=ON \
  -D BUILD_TBB=ON \
  -D WITH_OPENMP=ON \
  -D BUILD_PERF_TESTS=OFF \
  -D BUILD_TESTS=OFF \
  -D BUILD_EXAMPLES=OFF \
  ..

make -j4
sudo make install
sudo ldconfig

# 3. Verify OpenCV GPU capabilities
python3 -c "import cv2; print(cv2.getBuildInformation())" | grep -i vulkan
```

### **CODE IMPLEMENTATION:**

```cpp
// VulkanSGMAccelerator.hpp
namespace unlook::stereo {

/**
 * @brief Vulkan-accelerated Semi-Global Matching
 *
 * Uses VideoCore VII GPU via Vulkan compute shaders for:
 * - Cost aggregation (4-path or 8-path SGM)
 * - Subpixel refinement
 * - Left-right consistency check
 *
 * Performance target: 5-10x faster than CPU
 */
class VulkanSGMAccelerator {
public:
    struct Config {
        int minDisparity = 0;
        int numDisparities = 128;
        int P1 = 4;           // Small penalty
        int P2 = 24;          // Large penalty
        int pathCount = 4;    // 4 or 8 paths
        bool useSubpixel = true;
        bool leftRightCheck = true;
    };

    VulkanSGMAccelerator();
    ~VulkanSGMAccelerator();

    /**
     * @brief Initialize Vulkan compute pipeline
     * @return true if Vulkan available and initialized
     */
    bool initialize();

    /**
     * @brief Check if GPU acceleration is available
     */
    bool isAvailable() const { return vulkanAvailable_; }

    /**
     * @brief Compute disparity using Vulkan GPU
     * @param costVolume Input cost volume [H×W×D]
     * @param disparity Output disparity map [H×W] CV_16S
     * @return true on success
     */
    bool computeDisparity(const cv::Mat& costVolume, cv::Mat& disparity);

private:
    bool vulkanAvailable_ = false;
    Config config_;

    // Vulkan objects
    VkInstance instance_;
    VkPhysicalDevice physicalDevice_;
    VkDevice device_;
    VkQueue computeQueue_;
    VkCommandPool commandPool_;
    VkDescriptorPool descriptorPool_;
    VkPipeline computePipeline_;
    VkPipelineLayout pipelineLayout_;

    bool createVulkanInstance();
    bool selectPhysicalDevice();
    bool createLogicalDevice();
    bool createComputePipeline();
    void cleanupVulkan();
};

} // namespace unlook::stereo
```

### **GPU PERFORMANCE MONITORING:**

```cpp
// Add to HandheldScanPipeline
struct GPUStats {
    float gpuUtilization;      // 0-100%
    size_t gpuMemoryUsedMB;
    size_t gpuMemoryTotalMB;
    bool vulkanActive;
    std::string gpuName;
};

GPUStats getGPUStats() const;

// Log GPU usage
logger_.info("GPU: " + stats.gpuName +
            " | Utilization: " + std::to_string(stats.gpuUtilization) + "%" +
            " | Memory: " + std::to_string(stats.gpuMemoryUsedMB) + "MB/" +
                           std::to_string(stats.gpuMemoryTotalMB) + "MB");
```

---

## 📚 COMPETITOR RESEARCH & BEST PRACTICES

### **1. Intel RealSense D400 Series**

**Reference:** [librealsense GitHub](https://github.com/IntelRealSense/librealsense)

**Key Algorithms:**
```cpp
// From rs-depth-quality tool
struct RealSenseDepthQuality {
    // 1. Texture-aware stereo matching
    float textureThreshold = 0.5f;     // Skip low-texture regions

    // 2. Subpixel refinement (quadratic fitting)
    int subpixelBits = 4;              // 1/16 pixel accuracy

    // 3. Temporal filtering (IIR filter)
    float temporalAlpha = 0.4f;        // Smooth over time
    float temporalDelta = 20.0f;       // Jump threshold (mm)

    // 4. Spatial edge-preserving filter
    int spatialRadius = 2;             // pixels
    float spatialSigma = 0.5f;

    // 5. Hole filling (inpainting)
    bool holeFilling = true;
};
```

**Lessons:**
- Use **texture confidence** per pixel
- Implement **temporal filtering** for multi-frame
- **Edge-preserving spatial filter** before point cloud
- **Hole filling** for complete meshes

### **2. Luxonis OAK-D**

**Reference:** [DepthAI Documentation](https://docs.luxonis.com/projects/api/en/latest/components/nodes/stereo_depth/)

**Key Configurations:**
```cpp
struct LuxonisDepthConfig {
    // Extended disparity (2x range)
    bool extendedDisparity = false;    // Trade resolution for range

    // Subpixel mode
    enum SubpixelMode {
        SUBPIXEL_3BIT = 3,   // 1/8 pixel (fast)
        SUBPIXEL_4BIT = 4,   // 1/16 pixel (balanced)
        SUBPIXEL_5BIT = 5    // 1/32 pixel (slow, high precision)
    };
    SubpixelMode subpixel = SUBPIXEL_4BIT;

    // Median filter (remove speckles)
    enum MedianFilter {
        MEDIAN_OFF = 0,
        KERNEL_3x3 = 3,
        KERNEL_5x5 = 5,
        KERNEL_7x7 = 7
    };
    MedianFilter median = KERNEL_5x5;

    // Left-right check threshold (occlusion detection)
    int lrCheckThreshold = 10;         // pixels

    // Confidence threshold
    int confidenceThreshold = 200;     // 0-255
};
```

**Lessons:**
- **Median filter** essential for speckle removal
- **LR consistency** check catches occlusions
- **Confidence map** per pixel
- **Extended disparity** mode for long-range

### **3. OpenCV Contrib - StereoBinarySGBM**

**Reference:** [opencv_contrib/stereo](https://github.com/opencv/opencv_contrib/tree/4.x/modules/stereo)

**Implementation Details:**
```cpp
// From stereo/src/descriptor.cpp
enum CensusType {
    CV_DENSE_CENSUS,              // All pixels in window
    CV_SPARSE_CENSUS,             // Selected positions
    CV_CS_CENSUS,                 // Center-symmetric
    CV_MODIFIED_CENSUS_TRANSFORM, // With threshold (BEST for VCSEL!)
    CV_MEAN_VARIATION,
    CV_MODIFIED_CS_CENSUS,
    CV_STAR_KERNEL
};

// Best for VCSEL: CV_MODIFIED_CENSUS_TRANSFORM
// Why: Handles illumination changes (VCSEL dots vs ambient)
// Threshold: 4-8 (intensity tolerance)

// From stereo/src/matching.cpp
// Hardware POPCOUNT for Hamming distance
#if CV_POPCNT
if (checkHardwareSupport(CV_CPU_POPCNT)) {
    hammingDist = _mm_popcnt_u32(xorResult);  // x86
}
#endif

// ARM NEON equivalent:
uint8x16_t xor_vec = veorq_u8(left_desc, right_desc);
uint8x16_t popcount = vcntq_u8(xor_vec);   // Hardware bit count!
```

**Lessons:**
- Use **Modified Census Transform** with threshold
- Hardware **POPCOUNT** (NEON vcntq_u8) critical for speed
- **Binary descriptors** faster than intensity-based

### **4. Research Paper - MDPI Sensors 2022**

**Reference:** [High-Precision Depth Estimation (MDPI)](https://www.mdpi.com/1424-8220/22/18/6933)

**Key Findings:**
```cpp
/**
 * @brief Multi-scale coarse-to-fine stereo matching
 *
 * From: "High-Precision Depth Estimation with the 3D Ken Burns Effect"
 * MDPI Sensors 2022
 */
struct MultiScaleMatching {
    // Stage 1: Coarse matching @ 1/4 resolution
    // - Fast, wide disparity range
    // - Initialize disparity hypothesis

    // Stage 2: Medium matching @ 1/2 resolution
    // - Refine around coarse estimates
    // - Narrow disparity search

    // Stage 3: Fine matching @ full resolution
    // - Subpixel refinement
    // - Small search window

    // Speedup: 3-5x vs single-scale
    // Accuracy: Same or better (better initialization)
};

/**
 * @brief Adaptive cost aggregation weights
 */
struct AdaptiveWeights {
    // Weight based on color similarity
    float colorSigma = 7.0f;

    // Weight based on spatial distance
    float spatialSigma = 9.0f;

    // Combined: w = exp(-(color_diff/colorSigma)^2 - (spatial_diff/spatialSigma)^2)
};
```

**Lessons:**
- **Multi-scale** pyramid for speed + accuracy
- **Adaptive aggregation** weights (color + spatial)
- **Coarse-to-fine** reduces search space

---

## 🔨 COMPLETE BACKEND ARCHITECTURE (NEW)

### **File Structure:**

```
src/stereo/
├── StereoProcessor.cpp              (NEW - main pipeline coordinator)
├── RectificationEngine.cpp          (NEW - handles all rectification)
├── DisparityComputer.cpp            (NEW - AD-Census + SGBM)
├── DepthConverter.cpp               (NEW - disparity → depth with Q matrix)
├── PointCloudGenerator.cpp          (NEW - depth → PLY with filtering)
├── gpu/
│   ├── VulkanSGMAccelerator.cpp     (NEW - GPU acceleration)
│   └── shaders/
│       ├── sgm_cost_aggregation.comp (NEW - Vulkan compute shader)
│       └── wls_filter.comp          (NEW - edge-preserving filter)
└── filters/
    ├── TemporalFilter.cpp           (NEW - multi-frame IIR)
    ├── SpatialFilter.cpp            (NEW - edge-preserving)
    ├── MedianFilter.cpp             (NEW - speckle removal)
    └── HoleFiller.cpp               (NEW - inpainting)

src/api/
└── HandheldScanPipeline.cpp         (REWRITE - orchestrates everything)

include/unlook/stereo/
└── (corresponding headers)
```

### **Main Pipeline Flow:**

```cpp
/**
 * @brief Complete stereo processing pipeline (REWRITTEN FROM SCRATCH)
 *
 * Based on best practices from:
 * - Intel RealSense D400
 * - Luxonis OAK-D
 * - OpenCV-contrib StereoBinarySGBM
 * - MDPI Sensors 2022 research paper
 */
class StereoProcessor {
public:
    struct ProcessingParams {
        // Input resolution (MUST match calibration!)
        cv::Size inputSize = cv::Size(1280, 720);

        // Disparity range
        int minDisparity = 0;
        int numDisparities = 128;

        // AD-Census parameters
        float lambdaAD = 0.3f;           // AD weight
        float lambdaCensus = 0.7f;       // Census weight
        int censusWindowSize = 9;        // 9x9 window
        int censusThreshold = 4;         // Illumination tolerance

        // SGM parameters
        int P1 = 4;                      // Small smoothness
        int P2 = 24;                     // Large smoothness
        int uniquenessRatio = 15;
        int speckleWindowSize = 200;
        int speckleRange = 2;

        // Subpixel
        bool useSubpixel = true;
        int subpixelScale = 16;          // 1/16 pixel

        // Filters
        bool useMedianFilter = true;
        int medianKernelSize = 5;
        bool useTemporalFilter = true;
        float temporalAlpha = 0.4f;
        bool useSpatialFilter = true;
        bool useHoleFilling = true;

        // GPU
        bool useGPU = true;
        bool forceVulkan = false;
    };

    struct ProcessingResult {
        // Intermediate outputs (for debugging)
        cv::Mat leftRectified;           // Rectified left image
        cv::Mat rightRectified;          // Rectified right image
        cv::Mat rawDisparity;            // Raw disparity (before filtering)
        cv::Mat filteredDisparity;       // After median + spatial
        cv::Mat confidenceMap;           // Confidence 0-255 per pixel
        cv::Mat depthMap;                // Depth in mm (CV_32F)

        // Final outputs
        PointCloud pointCloud;           // Filtered point cloud
        std::string plyPath;             // Saved PLY file path

        // Quality metrics
        float validPixelPercentage;      // % of valid depth pixels
        float meanDepthMM;               // Mean depth value
        float depthStdDevMM;             // Depth variance

        // Performance metrics
        std::chrono::milliseconds rectificationTime;
        std::chrono::milliseconds disparityTime;
        std::chrono::milliseconds filteringTime;
        std::chrono::milliseconds pointCloudTime;
        std::chrono::milliseconds totalTime;

        bool gpuUsed;
        float gpuUtilization;
    };

    /**
     * @brief Process stereo image pair to point cloud
     *
     * CRITICAL: Images MUST match calibration resolution!
     *
     * @param leftImage Left camera image (MUST be calibration size)
     * @param rightImage Right camera image (MUST be calibration size)
     * @param debugDir Directory to save debug images (empty = no debug)
     * @return ProcessingResult with all outputs and metrics
     */
    ProcessingResult process(
        const cv::Mat& leftImage,
        const cv::Mat& rightImage,
        const std::string& debugDir = "");

private:
    CalibrationManager* calibration_;
    RectificationEngine* rectifier_;
    DisparityComputer* disparityComputer_;
    DepthConverter* depthConverter_;
    PointCloudGenerator* pointCloudGen_;
    VulkanSGMAccelerator* gpuAccelerator_;

    // Filters
    TemporalFilter* temporalFilter_;
    SpatialFilter* spatialFilter_;
    MedianFilter* medianFilter_;
    HoleFiller* holeFiller_;

    ProcessingParams params_;
};
```

---

## 📸 DEBUG OUTPUT REQUIREMENTS

### **MANDATORY DEBUG IMAGES (save to scanXXXXX/ directory):**

```cpp
void StereoProcessor::saveDebugImages(const ProcessingResult& result,
                                     const std::string& debugDir) {
    if (debugDir.empty()) return;

    std::filesystem::create_directories(debugDir);

    // Stage 1: Input validation
    cv::imwrite(debugDir + "/00_input_left.png", leftImage);
    cv::imwrite(debugDir + "/00_input_right.png", rightImage);

    // Log input sizes
    logger_.info("Input left: " + std::to_string(leftImage.cols) + "x" +
                                 std::to_string(leftImage.rows));
    logger_.info("Calib size: " + std::to_string(calibration_->getImageSize().width) + "x" +
                                 std::to_string(calibration_->getImageSize().height));

    // Stage 2: Rectification
    cv::imwrite(debugDir + "/01_rectified_left.png", result.leftRectified);
    cv::imwrite(debugDir + "/01_rectified_right.png", result.rightRectified);

    // Draw epipolar lines to verify rectification
    cv::Mat epipolarViz = drawEpipolarLines(result.leftRectified, result.rightRectified);
    cv::imwrite(debugDir + "/01_epipolar_check.png", epipolarViz);

    // Stage 3: Raw disparity
    cv::Mat disparityViz;
    cv::normalize(result.rawDisparity, disparityViz, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::applyColorMap(disparityViz, disparityViz, cv::COLORMAP_JET);
    cv::imwrite(debugDir + "/02_disparity_raw.png", disparityViz);

    // Stage 4: Filtered disparity
    cv::normalize(result.filteredDisparity, disparityViz, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::applyColorMap(disparityViz, disparityViz, cv::COLORMAP_JET);
    cv::imwrite(debugDir + "/03_disparity_filtered.png", disparityViz);

    // Stage 5: Confidence map
    cv::imwrite(debugDir + "/04_confidence_map.png", result.confidenceMap);

    // Stage 6: Depth map
    cv::Mat depthViz;
    cv::normalize(result.depthMap, depthViz, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::applyColorMap(depthViz, depthViz, cv::COLORMAP_TURBO);
    cv::imwrite(debugDir + "/05_depth_map.png", depthViz);

    // Stage 7: Depth statistics
    cv::Mat depthHist = computeDepthHistogram(result.depthMap);
    cv::imwrite(debugDir + "/06_depth_histogram.png", depthHist);

    // Stage 8: Point cloud saved as PLY
    // (già salvato in result.plyPath)
    logger_.info("PLY saved: " + result.plyPath);

    // Save processing report
    std::ofstream report(debugDir + "/processing_report.txt");
    report << "=== STEREO PROCESSING REPORT ===" << std::endl;
    report << "Input size: " << leftImage.size() << std::endl;
    report << "Calibration size: " << calibration_->getImageSize() << std::endl;
    report << "Valid pixels: " << result.validPixelPercentage << "%" << std::endl;
    report << "Mean depth: " << result.meanDepthMM << " mm" << std::endl;
    report << "Depth std dev: " << result.depthStdDevMM << " mm" << std::endl;
    report << "GPU used: " << (result.gpuUsed ? "YES" : "NO") << std::endl;
    report << "Rectification: " << result.rectificationTime.count() << " ms" << std::endl;
    report << "Disparity: " << result.disparityTime.count() << " ms" << std::endl;
    report << "Filtering: " << result.filteringTime.count() << " ms" << std::endl;
    report << "Point cloud: " << result.pointCloudTime.count() << " ms" << std::endl;
    report << "TOTAL: " << result.totalTime.count() << " ms" << std::endl;
    report.close();
}
```

---

## ✅ VALIDATION & SUCCESS CRITERIA

### **1. Calibration Validation (BEFORE ANY PROCESSING):**
- [ ] Image size matches calibration (leftImage.size() == calib.imageSize)
- [ ] All rectification maps exist and match size
- [ ] Camera intrinsics valid (fx,fy > 0, cx,cy in bounds)
- [ ] Baseline in range (60-80mm)
- [ ] Epipolar alignment (cy_left ≈ cy_right, diff < 0.1 px)

### **2. Rectification Quality:**
- [ ] Rectified images NOT distorted (check visually)
- [ ] Epipolar lines horizontal (draw and verify)
- [ ] Valid pixel percentage > 95% (no huge black borders)

### **3. Disparity Quality:**
- [ ] Valid pixels in CENTER of image (not just edges!)
- [ ] Valid pixel percentage > 40%
- [ ] Smooth depth gradients (no salt-and-pepper noise after median filter)

### **4. Depth Quality:**
- [ ] Known object at 500mm → measured 495-505mm (±1% error)
- [ ] Known object at 1000mm → measured 980-1020mm (±2% error)
- [ ] No depth inversions (closer objects not further)

### **5. Performance:**
- [ ] GPU utilization > 70% (if Vulkan active)
- [ ] Total processing time < 150ms per frame @ 1280x720
- [ ] Multi-frame scan (10 frames) < 2 seconds

### **6. Debug Output:**
- [ ] All 8 debug images saved correctly
- [ ] Epipolar lines visualization shows horizontal lines
- [ ] Depth histogram shows reasonable distribution
- [ ] Processing report contains all metrics

---

## 🚀 IMPLEMENTATION PRIORITY

### **PHASE 1: FOUNDATION (CRITICAL)**
1. ✅ CalibrationValidation class + exhaustive checks
2. ✅ RectificationEngine (with validation)
3. ✅ Debug image saving infrastructure
4. ✅ Unit tests for calibration loading

### **PHASE 2: STEREO MATCHING**
1. ✅ DisparityComputer with AD-Census (CPU fallback)
2. ✅ VulkanSGMAccelerator (GPU)
3. ✅ Automatic GPU/CPU selection
4. ✅ Benchmark CPU vs GPU

### **PHASE 3: DEPTH & FILTERING**
1. ✅ DepthConverter with Q matrix
2. ✅ MedianFilter (speckle removal)
3. ✅ SpatialFilter (edge-preserving)
4. ✅ TemporalFilter (multi-frame)
5. ✅ HoleFiller (inpainting)

### **PHASE 4: POINT CLOUD**
1. ✅ PointCloudGenerator with filtering
2. ✅ PLY export with normals + colors
3. ✅ Statistical outlier removal

### **PHASE 5: INTEGRATION**
1. ✅ HandheldScanPipeline orchestration
2. ✅ Multi-frame capture + fusion
3. ✅ GUI updates (progress bars, GPU stats)

---

## 📦 BUILD SYSTEM UPDATES

```cmake
# CMakeLists.txt additions

# Find Vulkan (GPU acceleration)
find_package(Vulkan REQUIRED)
if(Vulkan_FOUND)
    message(STATUS "Vulkan found: ${Vulkan_LIBRARY}")
    add_definitions(-DHAVE_VULKAN)
else()
    message(WARNING "Vulkan NOT found - GPU acceleration disabled")
endif()

# Compile Vulkan shaders
find_program(GLSLC glslc)
if(GLSLC)
    add_custom_target(compile_shaders ALL
        COMMAND ${GLSLC} -fshader-stage=compute
                ${CMAKE_SOURCE_DIR}/src/stereo/gpu/shaders/sgm_cost_aggregation.comp
                -o ${CMAKE_BINARY_DIR}/sgm_cost_aggregation.spv
        COMMENT "Compiling Vulkan compute shaders"
    )
endif()

# New stereo backend library
add_library(unlook_stereo_backend
    src/stereo/StereoProcessor.cpp
    src/stereo/RectificationEngine.cpp
    src/stereo/DisparityComputer.cpp
    src/stereo/DepthConverter.cpp
    src/stereo/PointCloudGenerator.cpp
    src/stereo/gpu/VulkanSGMAccelerator.cpp
    src/stereo/filters/TemporalFilter.cpp
    src/stereo/filters/SpatialFilter.cpp
    src/stereo/filters/MedianFilter.cpp
    src/stereo/filters/HoleFiller.cpp
)

target_link_libraries(unlook_stereo_backend
    PRIVATE
        opencv_core
        opencv_calib3d
        opencv_imgproc
        opencv_ximgproc
        ${Vulkan_LIBRARIES}
)

if(Vulkan_FOUND)
    target_include_directories(unlook_stereo_backend PRIVATE ${Vulkan_INCLUDE_DIRS})
endif()
```

---

## 🎯 FINAL CHECKLIST

- [ ] **DELETED:** Old VCSELStereoMatcher.cpp
- [ ] **DELETED:** Old census_neon.cpp, hamming_neon.cpp
- [ ] **CREATED:** All new backend files (StereoProcessor, etc.)
- [ ] **TESTED:** Calibration validation catches mismatches
- [ ] **TESTED:** Rectified images look correct (no distortion)
- [ ] **TESTED:** Disparity maps have valid pixels in center
- [ ] **TESTED:** GPU acceleration working (>70% utilization)
- [ ] **TESTED:** Debug images save correctly
- [ ] **TESTED:** Point cloud exports to PLY
- [ ] **VERIFIED:** Known object at 500mm measures ±5mm
- [ ] **VERIFIED:** Processing time < 150ms per frame

---

## 💬 NOTES FOR AGENT

**REMEMBER:**
- 3 days ago it worked, now it's broken → probably calibration mismatch
- **VALIDATE CALIBRATION FIRST** before any processing!
- Check cy values in P1 and P2 matrices (MUST be equal!)
- Save ALL debug images for forensic analysis
- Focus on GPU utilization (VideoCore VII is powerful!)
- Follow RealSense/Luxonis best practices (they've solved these problems)

**IF RECTIFICATION STILL BROKEN:**
1. Print calibration imageSize
2. Print actual image size
3. Print rectification map sizes
4. Print P1 and P2 cy values
5. Draw epipolar lines and check if horizontal

**FORZA! Ricostruiamo tutto correttamente! 💪🔥**
