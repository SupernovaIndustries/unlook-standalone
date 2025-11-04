# MEGA-PROMPT: AD-CENSUS HANDHELD SCANNER IMPLEMENTATION

## 🎯 MISSION
Implementare COMPLETO sistema di scansione 3D handheld professionale con AD-Census stereo matching, ARM NEON optimization, tentativo GPU acceleration, IMU stability detection, e multi-frame fusion.

**RESOLUTION:** Full HD 1456x1088 (native IMX296 resolution)
**TARGETS:** 500mm @ 0.1mm precision, 1000mm @ 0.5mm precision, 3-5 FPS per frame (acceptable for multi-frame handheld scanning)

**CRITICAL:** Implementare TUTTO in questa sessione. Nessuna feature lasciata non implementata. Ristrutturazione COMPLETA del depth processing pipeline. ELIMINARE completamente il vecchio depth_test_widget e sostituirlo con nuovo handheld_scan_widget.

---

## 📋 EXECUTION PROTOCOL

### STEP 0: INITIAL SETUP ✅ (DONE)
- [x] Git commit stato attuale (CHECKPOINT fatto)

### STEP 1: AGENT COORDINATION (PARALLEL EXECUTION)

**Launch questi agent IN PARALLELO con singolo messaggio:**

1. **stereo-vision-optimizer**
   - Task: Implementare AD-Census algorithm con ARM NEON optimization
   - Input: Research findings (sotto), libSGM code, opencv_contrib code
   - Output: VCSELStereoMatcher class con AD-Census completo

2. **hardware-interface-controller**
   - Task: Implementare BMI270 IMU driver (I2C bus 1, address 0x69)
   - Input: BMI270 datasheet, stability detection requirements
   - Output: BMI270Driver class + StabilityDetector class

3. **realtime-pipeline-architect**
   - Task: Ristrutturare depth processing pipeline per multi-frame fusion
   - Input: Multi-frame requirements, target FPS
   - Output: HandheldScanPipeline class con multi-frame averaging

4. **ux-ui-design-architect**
   - Task: Implementare GUI per handheld scanning (stability indicator, progress)
   - **CRITICAL:** ELIMINARE completamente depth_test_widget (rimuovere .cpp, .hpp, .ui files)
   - **CRITICAL:** Sostituire depth_test_widget con nuovo handheld_scan_widget in main_window
   - Input: UX requirements (sotto)
   - Output: HandheldScanWidget con stability feedback, vecchio widget RIMOSSO

**IMPORTANTE:** Tutti gli agent devono lavorare in parallelo. Build CMake solo DOPO che tutti hanno finito.

---

## 🔬 RESEARCH FINDINGS (DA IERI SERA)

### Intel RealSense D400 AD-Census Algorithm
**Paper:** "On building an accurate stereo matching system on graphics hardware" (Xing Mei et al., 2011, 504 citations)

**Key Findings:**
- **Algorithm:** AD-Census = Absolute Difference (AD) + Census Transform
- **Approach:** Matching cost = λ₁ × AD_cost + λ₂ × Census_cost
- **Aggregation:** Cross-based cost aggregation + scanline optimization
- **Performance:** Top Middlebury benchmark, 0.1s su GPU
- **VCSEL:** Static IR pattern per migliorare accuracy in low-texture scenes

**Why AD-Census for VCSEL:**
1. Census Transform → illumination invariant → robusto a VCSEL dots
2. AD cost → accuracy on texture → preciso dove c'è texture ambientale
3. Fusion → best of both worlds → robust + accurate

### libSGM (Fixstars) Analysis
**Repository:** https://github.com/fixstars/libSGM

**Key Implementations Found:**
```cpp
// File: /tmp/libSGM/src/census_transform.cu (Lines 29-99)
// Census 9x7 window (62 bit descriptor)
static constexpr int WINDOW_WIDTH = 9;
static constexpr int WINDOW_HEIGHT = 7;

template <typename T>
__global__ void census_transform_kernel(uint64_t* dest, const T* src, ...) {
    // Line 90: Core bit comparison
    f = (f << 1) | (a > b);  // Build binary descriptor
}

// Symmetric Census (31 bit descriptor) - compara coppie simmetriche
```

**Performance Metrics:**
- GTX 1080 Ti: 495 FPS @ 1024x440, disparity 128
- Parameters: P1=10, P2=120, uniqueness=0.95

### opencv_contrib StereoBinarySGBM Analysis
**Repository:** https://github.com/opencv/opencv_contrib/tree/master/modules/stereo

**Key Implementations Found:**
```cpp
// File: /tmp/opencv_contrib/modules/stereo/src/stereo_binary_sgbm.cpp
// Line 94: Default kernel type
kernelType = CV_MODIFIED_CENSUS_TRANSFORM;

// File: /tmp/opencv_contrib/modules/stereo/src/matching.hpp
// Line 169-178: Hamming distance with SSE POPCOUNT
int xorul = left[iwj] ^ right[iw + j2];
#if CV_POPCNT
if (checkHardwareSupport(CV_CPU_POPCNT)) {
    c[iwj * (v+1) + d] = (short)_mm_popcnt_u32(xorul);  // SSE optimization!
}
#endif
```

**Census Variants Available:**
1. **CV_MODIFIED_CENSUS_TRANSFORM (MCT)** - 2 bits + threshold (robust to illumination!)
2. **CV_MEAN_VARIATION (MV)** - Compare center + window mean (robust to noise)
3. **CV_CS_CENSUS** - Center Symmetric Census
4. **CV_STAR_KERNEL** - 9x9 sparse positions

### ARM NEON Capabilities (Raspberry Pi CM5)
**CPU:** Cortex-A76, BCM2712, 4 cores @ 2.4 GHz

**NEON Features Available:**
```bash
# From /proc/cpuinfo
Features: fp asimd evtstrm aes pmull sha1 sha2 crc32 atomics fphp asimdhp
          cpuid asimdrdm lrcpc dcpop asimddp
```

**Key Instructions for AD-Census:**
- `vcntq_u8()` - Hardware POPCOUNT for Hamming distance! 🚀
- `vabdq_u8()` - Absolute difference (single instruction)
- `veorq_u8()` - XOR for census comparison
- `vcgtq_u8()` - Greater-than comparison for census bit generation

**Performance Estimates @ FULL HD 1456x1088:**
```
Pixels: 1,584,128 (5.16x more than VGA)

Census transform (9x9 NEON):      ~10ms
Hamming distance (NEON POPCOUNT): ~15ms
AD computation (NEON):            ~5ms
Cost fusion:                      ~2.5ms
SGM 4-path:                       ~230ms  (bottleneck!)
Post-processing:                  ~25ms
──────────────────────────────────────────
TOTAL:                            ~288ms → 3.5 FPS ✅

Multi-frame scan: 10 frames @ 3.5 FPS = 2.8 seconds
→ Acceptable for handheld high-precision scanning!
```

### GPU Capabilities (VideoCore VII)
**Current Status:**
- OpenCL: **NOT supported** on VideoCore VII (only VideoCore IV had VC4CL)
- Vulkan 1.2: **Supported** but requires custom compute shaders
- OpenCV: No automatic backend for VideoCore VII

**Action:** Tentare Vulkan compute shaders per SGM aggregation (futuro optimization).

---

## 🎯 TARGET SPECIFICATIONS

### Precision Requirements:
- **500mm distance:** 0.1mm precision
- **1000mm distance:** 0.5mm precision

### Physical Calculations:
```
Depth precision: ΔZ = (Z² / (f × B)) × Δd

@ 500mm:  1 pixel error = 2.03mm
          → Need 1/20 pixel subpixel accuracy
          → With 10-frame averaging: √10 = 3.16x improvement
          → Achievable: 2.03mm / (16 × 3.16) = 0.04mm ✅✅

@ 1000mm: 1 pixel error = 8.12mm
          → Need 1/81 pixel subpixel accuracy
          → With 10-frame averaging: √10 = 3.16x improvement
          → Achievable: 8.12mm / (16 × 3.16) = 0.16mm ✅ (target 0.5mm, ampio margine!)
```

### Performance Target:
- **Full HD Processing:** 3-5 FPS per frame @ 1456x1088
- **Multi-frame scan:** 10 frames @ 3.5 FPS = 2.8 sec per high-precision scan (acceptable!)
- **Handheld mode:** IMU stability detection mandatory
- **Resolution:** 1456x1088 native (NO downsampling, full precision)

---

## 🔧 HARDWARE SPECIFICATIONS

### Cameras:
- **Sensors:** 2x IMX296 Global Shutter (1456x1088 SBGGR10)
- **Resolution:** **1456x1088 FULL HD (native, NO downsampling)**
- **Baseline:** 70.017mm (from calibration/calib_boofcv_test3.yaml)
- **Focal length:** ~1755 pixels
- **Hardware sync:** XVS/XHS, <1ms precision
- **Mapping:** Camera 1 = LEFT/MASTER, Camera 0 = RIGHT/SLAVE
- **CRITICAL:** Configure cameras to capture at FULL 1456x1088 resolution

### VCSEL Illuminator:
- **LED1 (VCSEL):** ams OSRAM BELAGO1.1 (15k dots, 940nm)
- **LED2 (Flood):** Flood illuminator
- **Controller:** AS1170 (I2C bus 1, address 0x30, GPIO 17 strobe)
- **Current:** 280mA (verified minimum for VCSEL)
- **Mode:** FLASH MODE (0x1B) with GPIO HIGH continuous

### IMU:
- **Model:** Bosch BMI270 6-axis (3-axis gyro + 3-axis accel)
- **Interface:** I2C bus 1, address 0x69
- **Update rate:** Up to 1600 Hz
- **Purpose:** Stability detection, motion compensation, pose tracking

### Raspberry Pi CM5:
- **CPU:** Cortex-A76 (4 cores @ 2.4 GHz)
- **NEON:** Advanced SIMD with dot product (asimddp)
- **GPU:** VideoCore VII (Vulkan 1.2, no OpenCL)
- **Memory:** Unified memory architecture

---

## 📐 ALGORITHM ARCHITECTURE

### Pipeline Overview:
```
IMU Stability Detection
    ↓
Multi-Frame Capture (10-15 frames)
    ↓
For each frame:
    ├─ Pattern Isolation (VCSEL - ambient)
    ├─ Census Transform (9x9 window, NEON optimized)
    ├─ AD Cost (Absolute Difference, NEON optimized)
    ├─ Hamming Distance (NEON POPCOUNT)
    ├─ Cost Fusion (λ_AD × AD + λ_Census × Census)
    └─ SGM Aggregation (4-path)
    ↓
Multi-Frame Fusion (weighted median, outlier rejection)
    ↓
WLS Filtering (edge-preserving smoothing)
    ↓
Point Cloud Generation
```

### AD-Census Parameters (Optimized for 400-1000mm):
```cpp
// Disparity range optimization
minDisparity = 48;      // ~1500mm far plane
numDisparities = 256;   // 48+256=304px → ~230mm near plane
                        // Range: 230-1500mm (covers target 400-1000mm)

// Census Transform
censusWindowSize = 9;   // 9x9 = 80 bit descriptor
censusType = CV_MODIFIED_CENSUS_TRANSFORM;
censusThreshold = 4;    // Tolerance for illumination variations

// AD-Census Fusion
lambda_AD = 0.3;        // AD weight (texture precision)
lambda_Census = 0.7;    // Census weight (illumination robustness)

// SGM Parameters (VCSEL-optimized)
P1 = 4;                 // Small penalty (preserve VCSEL dots detail)
P2 = 24;                // Moderate penalty (avoid false smoothing)
uniquenessRatio = 25;   // Strict (reject ambiguous dot matches)
preFilterCap = 31;      // Moderate normalization

// Subpixel
subpixelMethod = PARABOLIC_FITTING;  // Better than quadratic
subpixelScale = 16;     // 1/16 pixel accuracy

// Multi-Frame
numFrames = 10;         // Balance speed (0.67s) vs precision
outlierSigma = 2.5;     // Reject outliers beyond 2.5 standard deviations
fusionMethod = WEIGHTED_MEDIAN;  // Robust to outliers
```

### IMU Stability Detection:
```cpp
struct StabilityParams {
    float gyro_threshold = 0.5;      // degrees/sec (all axes)
    float accel_variance = 0.1;      // m/s² variance threshold
    int stable_duration_ms = 500;    // Stable for 500ms before capture
};

bool isStable() {
    return (abs(gyro_x) < 0.5 && abs(gyro_y) < 0.5 && abs(gyro_z) < 0.5)
        && (accel_variance < 0.1)
        && (stable_duration > 500ms);
}
```

---

## 💻 CODE IMPLEMENTATION REQUIREMENTS

### NEW CLASSES TO IMPLEMENT:

#### 1. VCSELStereoMatcher (stereo-vision-optimizer agent)
```cpp
namespace unlook::stereo {

class VCSELStereoMatcher : public StereoMatcher {
public:
    struct ADCensusParams {
        // Census
        int censusWindowSize = 9;
        CensusType censusType = CV_MODIFIED_CENSUS_TRANSFORM;
        int censusThreshold = 4;

        // AD-Census fusion
        float lambdaAD = 0.3f;
        float lambdaCensus = 0.7f;

        // SGM
        int minDisparity = 48;
        int numDisparities = 256;
        int P1 = 4;
        int P2 = 24;
        int uniquenessRatio = 25;

        // Performance
        bool useNEON = true;
        bool attemptVulkan = true;  // Try Vulkan compute shaders
    };

    cv::Mat compute(const cv::Mat& leftVCSEL,
                    const cv::Mat& rightVCSEL,
                    const cv::Mat& leftAmbient,
                    const cv::Mat& rightAmbient) override;

private:
    // Pattern isolation
    cv::Mat patternIsolation(const cv::Mat& vcsel, const cv::Mat& ambient);

    // Census transform (NEON optimized)
    void censusTransformNEON(const cv::Mat& image, cv::Mat& census);

    // AD cost (NEON optimized)
    void computeADCostNEON(const cv::Mat& left, const cv::Mat& right,
                           cv::Mat& adCost);

    // Hamming distance (NEON POPCOUNT)
    void hammingDistanceNEON(const cv::Mat& censusLeft,
                             const cv::Mat& censusRight,
                             cv::Mat& censusCost);

    // Cost fusion
    cv::Mat fuseCosts(const cv::Mat& adCost, const cv::Mat& censusCost);

    // SGM aggregation
    cv::Mat sgmAggregation(const cv::Mat& fusedCost);

    // Vulkan acceleration attempt
    bool initializeVulkanCompute();
    cv::Mat sgmVulkan(const cv::Mat& cost);

    ADCensusParams params_;
    bool vulkanAvailable_ = false;
};

} // namespace unlook::stereo
```

#### 2. BMI270Driver + StabilityDetector (hardware-interface-controller agent)
```cpp
namespace unlook::hardware {

class BMI270Driver {
public:
    struct IMUData {
        float gyro_x, gyro_y, gyro_z;      // degrees/sec
        float accel_x, accel_y, accel_z;   // m/s²
        uint64_t timestamp_us;
    };

    BMI270Driver(int i2c_bus = 1, uint8_t address = 0x69);

    bool initialize();
    IMUData read();

private:
    int i2c_fd_;
    uint8_t address_;
};

class StabilityDetector {
public:
    struct StabilityParams {
        float gyro_threshold = 0.5f;       // deg/sec
        float accel_variance = 0.1f;       // m/s²
        int stable_duration_ms = 500;
    };

    void update(const BMI270Driver::IMUData& imu);
    bool isStable() const;
    float getStabilityScore() const;  // 0.0-1.0 for GUI

private:
    std::deque<BMI270Driver::IMUData> history_;
    std::chrono::steady_clock::time_point stable_since_;
    StabilityParams params_;
};

} // namespace unlook::hardware
```

#### 3. HandheldScanPipeline (realtime-pipeline-architect agent)
```cpp
namespace unlook::api {

class HandheldScanPipeline {
public:
    struct ScanParams {
        int numFrames = 10;
        float targetPrecisionMM = 0.1f;
        float maxDistanceMM = 1000.0f;
        float outlierSigma = 2.5f;
    };

    struct ScanResult {
        cv::Mat depthMap;
        PointCloud pointCloud;
        float achievedPrecisionMM;
        int validPixelPercentage;
        std::chrono::milliseconds scanDuration;
    };

    // Main handheld scan
    ScanResult scanWithStability(const ScanParams& params);

private:
    // Wait for IMU stability
    bool waitForStability(std::function<void(float)> progressCallback);

    // Multi-frame capture
    std::vector<StereoFrame> captureMultiFrame(int numFrames);

    // Process each frame with AD-Census
    std::vector<cv::Mat> processFrames(const std::vector<StereoFrame>& frames);

    // Multi-frame fusion
    cv::Mat fuseDepthMaps(const std::vector<cv::Mat>& depthMaps,
                          float outlierSigma);

    // WLS filtering
    cv::Mat wlsFilter(const cv::Mat& depth, const cv::Mat& guide);

    VCSELStereoMatcher* stereoMatcher_;
    StabilityDetector* stabilityDetector_;
    CameraSystem* cameraSystem_;
};

} // namespace unlook::api
```

#### 4. HandheldScanWidget (ux-ui-design-architect agent)
```cpp
namespace unlook::gui {

class HandheldScanWidget : public QWidget {
    Q_OBJECT

public:
    HandheldScanWidget(QWidget* parent = nullptr);

signals:
    void scanCompleted(const PointCloud& cloud);

private slots:
    void onStartScan();
    void onStabilityUpdate(float stabilityScore);
    void onCaptureProgress(int current, int total);
    void onProcessingProgress(int percentage);

private:
    // UI Components
    QProgressBar* stabilityBar_;      // 0-100% stability indicator
    QLabel* stabilityLabel_;          // "Hold steady..." / "Stable!"
    QProgressBar* captureProgress_;   // Multi-frame capture progress
    QLabel* fpsLabel_;                // Real-time FPS
    QLabel* precisionEstimate_;       // Estimated precision
    QPushButton* scanButton_;

    // Visual feedback
    void updateStabilityIndicator(float score);
    void showCaptureAnimation();

    HandheldScanPipeline* scanPipeline_;
};

} // namespace unlook::gui
```

---

## 🔨 IMPLEMENTATION STEPS (DETAILED)

### AGENT 1: stereo-vision-optimizer

**Files to Create/Modify:**
- `src/stereo/VCSELStereoMatcher.cpp` (NEW)
- `include/unlook/stereo/VCSELStereoMatcher.hpp` (NEW)
- `src/stereo/neon/census_neon.cpp` (NEW - ARM NEON implementations)
- `src/stereo/neon/hamming_neon.cpp` (NEW)
- `src/stereo/neon/ad_cost_neon.cpp` (NEW)

**ARM NEON Implementation Examples:**

```cpp
// census_neon.cpp
#include <arm_neon.h>

void censusTransformNEON(const uint8_t* image, uint32_t* census,
                         int width, int height) {
    const int radius = 4;  // 9x9 window

    for (int y = radius; y < height - radius; y++) {
        for (int x = radius; x < width - radius; x += 16) {
            // Load 16 center pixels
            uint8x16_t center = vld1q_u8(&image[y * width + x]);

            uint32_t census_bits[16] = {0};

            // Compare with 9x9 neighborhood
            for (int dy = -radius; dy <= radius; dy++) {
                for (int dx = -radius; dx <= radius; dx++) {
                    if (dx == 0 && dy == 0) continue;

                    uint8x16_t neighbor = vld1q_u8(&image[(y+dy)*width + x+dx]);
                    uint8x16_t cmp = vcgtq_u8(neighbor, center);

                    // Accumulate bits
                    for (int i = 0; i < 16; i++) {
                        census_bits[i] = (census_bits[i] << 1) |
                                        (vgetq_lane_u8(cmp, i) ? 1 : 0);
                    }
                }
            }

            // Store results
            vst1q_u32(&census[y * width + x], vld1q_u32(census_bits));
            vst1q_u32(&census[y * width + x + 4], vld1q_u32(census_bits + 4));
            vst1q_u32(&census[y * width + x + 8], vld1q_u32(census_bits + 8));
            vst1q_u32(&census[y * width + x + 12], vld1q_u32(census_bits + 12));
        }
    }
}

// hamming_neon.cpp
void hammingDistanceNEON(const uint32_t* left_census,
                         const uint32_t* right_census,
                         uint16_t* cost_volume,
                         int width, int height, int max_disp) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x += 4) {
            uint32x4_t left_desc = vld1q_u32(&left_census[y * width + x]);

            for (int d = 0; d < max_disp; d++) {
                int x_right = std::max(0, x - d);
                uint32x4_t right_desc = vld1q_u32(&right_census[y*width + x_right]);

                // XOR to find different bits
                uint32x4_t xor_result = veorq_u32(left_desc, right_desc);

                // POPCOUNT using vcnt (hardware bit count!)
                uint8x16_t xor_bytes = vreinterpretq_u8_u32(xor_result);
                uint8x16_t bit_count = vcntq_u8(xor_bytes);

                // Sum to get Hamming distance
                uint16x8_t sum16_low = vpaddlq_u8(vget_low_u8(bit_count));
                uint16x8_t sum16_high = vpaddlq_u8(vget_high_u8(bit_count));
                uint32x4_t sum32 = vpaddlq_u16(vcombine_u16(
                    vget_low_u16(sum16_low), vget_low_u16(sum16_high)));

                // Store
                uint16_t hamming[4];
                vst1q_u16(hamming, vreinterpretq_u16_u32(sum32));
                for (int i = 0; i < 4; i++) {
                    cost_volume[(y*width + x + i)*max_disp + d] = hamming[i];
                }
            }
        }
    }
}

// ad_cost_neon.cpp
void absoluteDifferenceNEON(const uint8_t* left, const uint8_t* right,
                            uint16_t* ad_cost,
                            int width, int height, int max_disp) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x += 16) {
            uint8x16_t left_pixels = vld1q_u8(&left[y * width + x]);

            for (int d = 0; d < max_disp; d++) {
                int x_right = std::max(0, x - d);
                uint8x16_t right_pixels = vld1q_u8(&right[y * width + x_right]);

                // Absolute difference (single NEON instruction!)
                uint8x16_t ad = vabdq_u8(left_pixels, right_pixels);

                // Expand to uint16_t and store
                uint16x8_t ad_low = vmovl_u8(vget_low_u8(ad));
                uint16x8_t ad_high = vmovl_u8(vget_high_u8(ad));

                vst1q_u16(&ad_cost[(y*width + x)*max_disp + d], ad_low);
                vst1q_u16(&ad_cost[(y*width + x+8)*max_disp + d], ad_high);
            }
        }
    }
}
```

**Vulkan Compute Shader Attempt:**
```cpp
// vulkan_sgm.cpp (experimental - tentativo di usare GPU)
#include <vulkan/vulkan.h>

bool VCSELStereoMatcher::initializeVulkanCompute() {
    // Try to initialize Vulkan compute
    VkInstance instance;
    VkInstanceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;

    if (vkCreateInstance(&createInfo, nullptr, &instance) != VK_SUCCESS) {
        core::Logger::getInstance().warn("Vulkan not available, using CPU only");
        return false;
    }

    // Enumerate physical devices
    uint32_t deviceCount = 0;
    vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr);

    if (deviceCount == 0) {
        core::Logger::getInstance().warn("No Vulkan devices found");
        return false;
    }

    // Continue Vulkan setup for compute shaders...
    // This is experimental - focus on NEON first!
    vulkanAvailable_ = true;
    return true;
}
```

### AGENT 2: hardware-interface-controller

**Files to Create/Modify:**
- `src/hardware/BMI270Driver.cpp` (NEW)
- `include/unlook/hardware/BMI270Driver.hpp` (NEW)
- `src/hardware/StabilityDetector.cpp` (NEW)
- `include/unlook/hardware/StabilityDetector.hpp` (NEW)

**BMI270 I2C Implementation:**
```cpp
// BMI270Driver.cpp
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

BMI270Driver::BMI270Driver(int i2c_bus, uint8_t address)
    : address_(address) {

    std::string device = "/dev/i2c-" + std::to_string(i2c_bus);
    i2c_fd_ = open(device.c_str(), O_RDWR);

    if (i2c_fd_ < 0) {
        throw core::Exception("Failed to open I2C bus " + std::to_string(i2c_bus));
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, address_) < 0) {
        close(i2c_fd_);
        throw core::Exception("Failed to set I2C address 0x" +
                            std::to_string(address_));
    }
}

bool BMI270Driver::initialize() {
    // BMI270 initialization sequence

    // 1. Check chip ID (should be 0x24)
    uint8_t chip_id = readRegister(0x00);
    if (chip_id != 0x24) {
        core::Logger::getInstance().error("BMI270 chip ID mismatch: 0x" +
                                         std::to_string(chip_id));
        return false;
    }

    // 2. Soft reset
    writeRegister(0x7E, 0xB6);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 3. Load config file (BMI270 requires config blob)
    // This is complex - see BMI270 datasheet section 5.2
    // For now, use default configuration

    // 4. Configure accelerometer
    // ACC_CONF: ODR=100Hz, BWP=normal, filter_perf=0
    writeRegister(0x40, 0x28);  // 100Hz ODR

    // ACC_RANGE: ±2g
    writeRegister(0x41, 0x00);

    // 5. Configure gyroscope
    // GYR_CONF: ODR=100Hz, BWP=normal, filter_perf=0
    writeRegister(0x42, 0x28);  // 100Hz ODR

    // GYR_RANGE: ±500 dps
    writeRegister(0x43, 0x02);

    // 6. Enable sensors
    // PWR_CTRL: enable ACC + GYR
    writeRegister(0x7D, 0x0E);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    core::Logger::getInstance().info("BMI270 initialized successfully");
    return true;
}

BMI270Driver::IMUData BMI270Driver::read() {
    IMUData data;

    // Read gyro data (6 bytes starting at 0x0C)
    uint8_t gyro_buf[6];
    readBlock(0x0C, gyro_buf, 6);

    int16_t gyro_x_raw = (gyro_buf[1] << 8) | gyro_buf[0];
    int16_t gyro_y_raw = (gyro_buf[3] << 8) | gyro_buf[2];
    int16_t gyro_z_raw = (gyro_buf[5] << 8) | gyro_buf[4];

    // Convert to degrees/sec (±500 dps range)
    const float gyro_scale = 500.0f / 32768.0f;
    data.gyro_x = gyro_x_raw * gyro_scale;
    data.gyro_y = gyro_y_raw * gyro_scale;
    data.gyro_z = gyro_z_raw * gyro_scale;

    // Read accel data (6 bytes starting at 0x12)
    uint8_t accel_buf[6];
    readBlock(0x12, accel_buf, 6);

    int16_t accel_x_raw = (accel_buf[1] << 8) | accel_buf[0];
    int16_t accel_y_raw = (accel_buf[3] << 8) | accel_buf[2];
    int16_t accel_z_raw = (accel_buf[5] << 8) | accel_buf[4];

    // Convert to m/s² (±2g range)
    const float accel_scale = (2.0f * 9.81f) / 32768.0f;
    data.accel_x = accel_x_raw * accel_scale;
    data.accel_y = accel_y_raw * accel_scale;
    data.accel_z = accel_z_raw * accel_scale;

    data.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    return data;
}
```

**StabilityDetector Implementation:**
```cpp
// StabilityDetector.cpp
void StabilityDetector::update(const BMI270Driver::IMUData& imu) {
    history_.push_back(imu);

    // Keep last 1 second of data
    while (history_.size() > 100) {  // 100 samples @ 100Hz
        history_.pop_front();
    }

    // Check stability criteria
    bool gyro_stable = (std::abs(imu.gyro_x) < params_.gyro_threshold &&
                       std::abs(imu.gyro_y) < params_.gyro_threshold &&
                       std::abs(imu.gyro_z) < params_.gyro_threshold);

    // Calculate accelerometer variance
    float accel_mean = 0.0f;
    for (const auto& data : history_) {
        float accel_mag = std::sqrt(data.accel_x * data.accel_x +
                                   data.accel_y * data.accel_y +
                                   data.accel_z * data.accel_z);
        accel_mean += accel_mag;
    }
    accel_mean /= history_.size();

    float accel_variance = 0.0f;
    for (const auto& data : history_) {
        float accel_mag = std::sqrt(data.accel_x * data.accel_x +
                                   data.accel_y * data.accel_y +
                                   data.accel_z * data.accel_z);
        float diff = accel_mag - accel_mean;
        accel_variance += diff * diff;
    }
    accel_variance /= history_.size();

    bool accel_stable = (accel_variance < params_.accel_variance);

    // Update stability duration
    auto now = std::chrono::steady_clock::now();
    if (gyro_stable && accel_stable) {
        if (stable_since_ == std::chrono::steady_clock::time_point()) {
            stable_since_ = now;
        }
    } else {
        stable_since_ = std::chrono::steady_clock::time_point();
    }
}

bool StabilityDetector::isStable() const {
    if (stable_since_ == std::chrono::steady_clock::time_point()) {
        return false;
    }

    auto duration = std::chrono::steady_clock::now() - stable_since_;
    return duration >= std::chrono::milliseconds(params_.stable_duration_ms);
}

float StabilityDetector::getStabilityScore() const {
    if (history_.empty()) return 0.0f;

    // Calculate score 0.0-1.0 based on gyro + accel stability
    const auto& latest = history_.back();

    float gyro_score = 1.0f - std::min(1.0f,
        (std::abs(latest.gyro_x) + std::abs(latest.gyro_y) + std::abs(latest.gyro_z)) /
        (params_.gyro_threshold * 3.0f));

    // ... accel variance score ...

    return (gyro_score * 0.7f + accel_score * 0.3f);
}
```

### AGENT 3: realtime-pipeline-architect

**Files to Create/Modify:**
- `src/api/HandheldScanPipeline.cpp` (NEW)
- `include/unlook/api/HandheldScanPipeline.hpp` (NEW)

**Multi-Frame Fusion Implementation:**
```cpp
// HandheldScanPipeline.cpp
cv::Mat HandheldScanPipeline::fuseDepthMaps(
    const std::vector<cv::Mat>& depthMaps,
    float outlierSigma) {

    if (depthMaps.empty()) {
        throw core::Exception("No depth maps to fuse");
    }

    cv::Mat fused = cv::Mat::zeros(depthMaps[0].size(), CV_32F);

    int height = fused.rows;
    int width = fused.cols;

    // For each pixel, compute weighted median
    #pragma omp parallel for collapse(2)
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            std::vector<float> values;
            values.reserve(depthMaps.size());

            // Collect all depth values for this pixel
            for (const auto& depth : depthMaps) {
                float val = depth.at<float>(y, x);
                if (val > 0) {  // Valid depth
                    values.push_back(val);
                }
            }

            if (values.empty()) {
                fused.at<float>(y, x) = 0.0f;
                continue;
            }

            // Calculate mean and std dev
            float mean = std::accumulate(values.begin(), values.end(), 0.0f) /
                        values.size();

            float variance = 0.0f;
            for (float val : values) {
                float diff = val - mean;
                variance += diff * diff;
            }
            variance /= values.size();
            float stddev = std::sqrt(variance);

            // Outlier rejection
            std::vector<float> inliers;
            for (float val : values) {
                if (std::abs(val - mean) <= outlierSigma * stddev) {
                    inliers.push_back(val);
                }
            }

            if (inliers.empty()) {
                fused.at<float>(y, x) = 0.0f;
                continue;
            }

            // Weighted median of inliers
            std::sort(inliers.begin(), inliers.end());
            fused.at<float>(y, x) = inliers[inliers.size() / 2];
        }
    }

    return fused;
}

HandheldScanPipeline::ScanResult HandheldScanPipeline::scanWithStability(
    const ScanParams& params) {

    auto start_time = std::chrono::steady_clock::now();

    ScanResult result;

    // 1. Wait for stability
    core::Logger::getInstance().info("Waiting for IMU stability...");

    bool stable = waitForStability([](float score) {
        // Progress callback for GUI
        core::Logger::getInstance().debug("Stability: " +
                                         std::to_string(score * 100) + "%");
    });

    if (!stable) {
        throw core::Exception("Failed to achieve stability");
    }

    // 2. Multi-frame capture
    core::Logger::getInstance().info("Capturing " +
                                    std::to_string(params.numFrames) +
                                    " frames...");

    auto frames = captureMultiFrame(params.numFrames);

    // 3. Process each frame with AD-Census
    core::Logger::getInstance().info("Processing frames with AD-Census...");

    auto depthMaps = processFrames(frames);

    // 4. Multi-frame fusion
    core::Logger::getInstance().info("Fusing " +
                                    std::to_string(depthMaps.size()) +
                                    " depth maps...");

    result.depthMap = fuseDepthMaps(depthMaps, params.outlierSigma);

    // 5. WLS filtering
    core::Logger::getInstance().info("Applying WLS filter...");

    result.depthMap = wlsFilter(result.depthMap, frames[0].leftVCSEL);

    // 6. Generate point cloud
    // ... point cloud generation ...

    result.scanDuration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time);

    core::Logger::getInstance().info("Scan completed in " +
                                    std::to_string(result.scanDuration.count()) +
                                    "ms");

    return result;
}
```

### AGENT 4: ux-ui-design-architect

**Files to DELETE (OLD DEPTH WIDGET):**
- `src/gui/depth_test_widget.cpp` (DELETE)
- `include/unlook/gui/depth_test_widget.hpp` (DELETE)
- `src/gui/ui/depth_test_widget.ui` (DELETE)

**Files to CREATE (NEW HANDHELD WIDGET):**
- `src/gui/handheld_scan_widget.cpp` (NEW)
- `include/unlook/gui/handheld_scan_widget.hpp` (NEW)
- `src/gui/ui/handheld_scan_widget.ui` (NEW - Qt Designer file)

**Files to MODIFY:**
- `src/gui/main_window.cpp` - Replace depth_test_widget with handheld_scan_widget
- `include/unlook/gui/main_window.hpp` - Update includes and member variables
- `src/gui/CMakeLists.txt` - Remove old widget, add new widget

**CRITICAL GUI Refactoring Steps:**

1. **DELETE old depth_test_widget files:**
```bash
rm src/gui/depth_test_widget.cpp
rm include/unlook/gui/depth_test_widget.hpp
rm src/gui/ui/depth_test_widget.ui
```

2. **Update main_window.hpp:**
```cpp
// REMOVE:
#include <unlook/gui/depth_test_widget.hpp>
DepthTestWidget* depth_test_widget_;

// ADD:
#include <unlook/gui/handheld_scan_widget.hpp>
HandheldScanWidget* handheld_scan_widget_;
```

3. **Update main_window.cpp:**
```cpp
// In MainWindow constructor, REPLACE:
depth_test_widget_ = new DepthTestWidget(this);
central_widget_->addTab(depth_test_widget_, "Depth Test");

// WITH:
handheld_scan_widget_ = new HandheldScanWidget(this);
central_widget_->addTab(handheld_scan_widget_, "Handheld Scan");
```

4. **Update src/gui/CMakeLists.txt:**
```cmake
# REMOVE:
src/gui/depth_test_widget.cpp

# ADD:
src/gui/handheld_scan_widget.cpp
```

**GUI Implementation:**
```cpp
// handheld_scan_widget.cpp
HandheldScanWidget::HandheldScanWidget(QWidget* parent)
    : QWidget(parent) {

    setupUi();

    // Connect to scan pipeline
    scanPipeline_ = new HandheldScanPipeline(this);

    // Update UI at 30 Hz
    QTimer* updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, &HandheldScanWidget::updateUI);
    updateTimer->start(33);  // ~30 FPS UI updates
}

void HandheldScanWidget::setupUi() {
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // Stability indicator
    QHBoxLayout* stabilityLayout = new QHBoxLayout();
    stabilityLabel_ = new QLabel("Hold Steady...");
    stabilityLabel_->setStyleSheet("font-size: 18pt; color: red;");
    stabilityLayout->addWidget(stabilityLabel_);

    stabilityBar_ = new QProgressBar();
    stabilityBar_->setRange(0, 100);
    stabilityBar_->setTextVisible(true);
    stabilityBar_->setFormat("Stability: %p%");
    stabilityLayout->addWidget(stabilityBar_);
    mainLayout->addLayout(stabilityLayout);

    // Capture progress
    captureProgress_ = new QProgressBar();
    captureProgress_->setRange(0, 100);
    captureProgress_->setTextVisible(true);
    captureProgress_->setFormat("Capturing: %p%");
    captureProgress_->setVisible(false);
    mainLayout->addWidget(captureProgress_);

    // FPS and precision info
    QHBoxLayout* infoLayout = new QHBoxLayout();
    fpsLabel_ = new QLabel("FPS: --");
    fpsLabel_->setStyleSheet("font-size: 12pt;");
    infoLayout->addWidget(fpsLabel_);

    precisionEstimate_ = new QLabel("Precision: --");
    precisionEstimate_->setStyleSheet("font-size: 12pt;");
    infoLayout->addWidget(precisionEstimate_);
    mainLayout->addLayout(infoLayout);

    // Scan button
    scanButton_ = new QPushButton("Start Handheld Scan");
    scanButton_->setStyleSheet("font-size: 16pt; padding: 10px;");
    connect(scanButton_, &QPushButton::clicked, this,
           &HandheldScanWidget::onStartScan);
    mainLayout->addWidget(scanButton_);

    mainLayout->addStretch();
}

void HandheldScanWidget::updateStabilityIndicator(float score) {
    int percentage = static_cast<int>(score * 100);
    stabilityBar_->setValue(percentage);

    if (score > 0.9f) {
        stabilityLabel_->setText("Stable! ✓");
        stabilityLabel_->setStyleSheet("font-size: 18pt; color: green;");
    } else if (score > 0.7f) {
        stabilityLabel_->setText("Almost there...");
        stabilityLabel_->setStyleSheet("font-size: 18pt; color: orange;");
    } else {
        stabilityLabel_->setText("Hold Steady...");
        stabilityLabel_->setStyleSheet("font-size: 18pt; color: red;");
    }
}

void HandheldScanWidget::onStartScan() {
    scanButton_->setEnabled(false);
    captureProgress_->setVisible(true);

    // Start scan in background thread
    QFuture<HandheldScanPipeline::ScanResult> future =
        QtConcurrent::run([this]() {
            HandheldScanPipeline::ScanParams params;
            params.numFrames = 10;
            params.targetPrecisionMM = 0.1f;
            return scanPipeline_->scanWithStability(params);
        });

    // Monitor progress
    QFutureWatcher<HandheldScanPipeline::ScanResult>* watcher =
        new QFutureWatcher<HandheldScanPipeline::ScanResult>(this);

    connect(watcher, &QFutureWatcher<HandheldScanPipeline::ScanResult>::finished,
            this, [this, watcher]() {
        auto result = watcher->result();

        precisionEstimate_->setText("Precision: " +
            QString::number(result.achievedPrecisionMM, 'f', 2) + " mm");

        emit scanCompleted(result.pointCloud);

        scanButton_->setEnabled(true);
        captureProgress_->setVisible(false);
    });

    watcher->setFuture(future);
}
```

---

## 📊 VALIDATION & TESTING

### Performance Benchmarks:
```cpp
// Test suite to implement
class ADCensusPerformanceTest {
public:
    void testHDPerformance() {
        // Target: 3-5 FPS @ 1456x1088 Full HD
        // Acceptable: 3.5 FPS (288ms per frame)
        // Multi-frame scan: 10 frames = 2.8 seconds
    }

    void testNEONOptimization() {
        // Compare NEON vs non-NEON
        // Expected: 3-5x speedup
    }

    void testMultiFrameFusion() {
        // Verify outlier rejection
        // Verify precision improvement
    }
};
```

### Precision Validation:
```cpp
class PrecisionValidationTest {
public:
    void testKnownDistances() {
        // Setup: Objects at 400mm, 500mm, 750mm, 1000mm
        // Measure: Achieved depth accuracy
        // Accept: 400-500mm < 0.15mm, 1000mm < 0.6mm
    }

    void testMultiFrameImprovement() {
        // Compare: Single-frame vs 10-frame fusion
        // Expected: ~3x precision improvement
    }
};
```

---

## 🚀 BUILD & DEPLOYMENT

### CMakeLists.txt Updates:
```cmake
# Add NEON optimization flags
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a+simd")
    add_definitions(-DHAVE_NEON)
endif()

# Try to find Vulkan (optional GPU acceleration)
find_package(Vulkan QUIET)
if(Vulkan_FOUND)
    add_definitions(-DHAVE_VULKAN)
    include_directories(${Vulkan_INCLUDE_DIRS})
endif()

# New libraries
add_library(unlook_stereo_vcsel
    src/stereo/VCSELStereoMatcher.cpp
    src/stereo/neon/census_neon.cpp
    src/stereo/neon/hamming_neon.cpp
    src/stereo/neon/ad_cost_neon.cpp
)

add_library(unlook_hardware_imu
    src/hardware/BMI270Driver.cpp
    src/hardware/StabilityDetector.cpp
)

add_library(unlook_api_handheld
    src/api/HandheldScanPipeline.cpp
)

# Link everything
target_link_libraries(unlook_stereo_vcsel
    PRIVATE opencv_core opencv_stereo
)

if(Vulkan_FOUND)
    target_link_libraries(unlook_stereo_vcsel PRIVATE ${Vulkan_LIBRARIES})
endif()

target_link_libraries(unlook_hardware_imu
    PRIVATE i2c
)
```

### Final Build Command:
```bash
# After all agents complete
cd /home/alessandro/unlook-standalone
./build.sh --clean -j4
```

---

## ✅ SUCCESS CRITERIA

### Functional Requirements:
- [x] AD-Census stereo matching implemented
- [x] ARM NEON optimizations working
- [x] BMI270 IMU driver functional
- [x] Stability detection working
- [x] Multi-frame capture and fusion
- [x] GUI handheld scan widget
- [x] Real-time performance (15+ FPS)

### Performance Requirements:
- [ ] Full HD 1456x1088: 3-5 FPS per frame
- [ ] Multi-frame scan (10 frames): 2.5-3 seconds total
- [ ] 500mm distance: ≤0.15mm precision (10-frame fusion)
- [ ] 1000mm distance: ≤0.6mm precision (10-frame fusion)
- [ ] Stability detection: <1 second to achieve stable state

### Code Quality:
- [ ] All code compiles without warnings
- [ ] NEON code portable (fallback to non-NEON)
- [ ] Proper error handling
- [ ] Logging at appropriate levels
- [ ] Thread-safe where needed

---

## 🎓 KNOWLEDGE TRANSFER

### Key Research Papers to Reference in Code:
```cpp
/**
 * @brief AD-Census stereo matching for VCSEL structured light
 *
 * Based on:
 * - "On building an accurate stereo matching system on graphics hardware"
 *   (Xing Mei et al., 2011)
 * - Intel RealSense D400 Series implementation
 * - libSGM (Fixstars) census transform optimization
 * - opencv_contrib StereoBinarySGBM
 *
 * Algorithm combines:
 * - Absolute Difference (AD) for texture accuracy
 * - Census Transform for illumination invariance
 * - ARM NEON SIMD for 3-5x performance boost
 *
 * @see https://github.com/fixstars/libSGM
 * @see https://github.com/opencv/opencv_contrib/tree/master/modules/stereo
 */
```

---

## 🔄 AGENT EXECUTION ORDER

**PARALLEL PHASE (All agents start simultaneously):**
1. stereo-vision-optimizer → VCSELStereoMatcher + NEON
2. hardware-interface-controller → BMI270Driver + StabilityDetector
3. realtime-pipeline-architect → HandheldScanPipeline
4. ux-ui-design-architect → HandheldScanWidget

**SEQUENTIAL PHASE (After all agents complete):**
5. code-integrity-architect → Code review and quality check
6. cmake-build-system-architect → Update CMakeLists.txt, build system
7. BUILD → `./build.sh --clean -j4`
8. testing-validation-framework → Run performance and precision tests

---

## 💬 FINAL NOTES

**This is a COMPLETE implementation.** No features should be left as "TODO" or "future work". Everything must be implemented, tested, and working by the end of this prompt execution.

**GPU Note:** Vulkan compute shader implementation is experimental. If it doesn't work easily, skip it and focus on NEON. We can add GPU acceleration later in PC companion mode.

**IMU Note:** BMI270 requires config file upload (complex). If this causes issues, start with simple register configuration and add advanced features later.

**Priority:** AD-Census + NEON > IMU > Multi-frame > GUI > Vulkan

**Mantieni sempre focus su precision target: 500mm @ 0.1mm, 1000mm @ 0.5mm!**

---

## 🚀 READY TO EXECUTE!

All agents have complete context, research findings, implementation details, and success criteria. Launch in parallel and build at the end!

**FORZA UNLOOK! 💪🎯**
