# MEGA-PROMPT: PROFESSIONAL STEREO CALIBRATION SYSTEM

## 🎯 MISSION
Implementare sistema COMPLETO di calibrazione stereo professionale con GUI intuitiva, auto-capture dataset, processing automatico, validazione qualità, e integration nel sistema. Sostituire "Face Enrollment" button con "Calibration" system.

**RESOLUTION:** HD 1280x720 (come AD-Census system)
**TARGET:** RMS error < 0.3 pixel, baseline accuracy < 0.5mm, fully automated workflow

**CRITICAL:** Implementare TUTTO in questa sessione. Nessun placeholder, nessun codice fake, solo CODICE REALE E FUNZIONANTE. Sistema deve essere production-ready.

---

## ⚙️ SYSTEM REQUIREMENTS & SETUP

### Resolution Configuration:
- **Capture from cameras:** 1456x1088 (native IMX296)
- **Processing resolution:** **HD 1280x720** (downsampled con cv::INTER_AREA)
- **ALL calibration processing:** HD 1280x720
- **Dataset images saved:** HD 1280x720 PNG files
- **Why HD:** Same resolution as AD-Census system for consistency

### Dependencies to Install:

**CRITICAL: Agents devono installare automaticamente se mancanti!**

```bash
# 1. nlohmann-json (for dataset JSON metadata)
sudo apt-get update
sudo apt-get install -y nlohmann-json3-dev

# 2. OpenCV ArUco module (for ChArUco pattern detection)
# Already included in opencv_contrib - verify:
pkg-config --modversion opencv4
# Should show opencv_aruco in modules
```

**CMakeLists.txt deve fare auto-check:**
```cmake
# JSON library
find_package(nlohmann_json 3.10.0 QUIET)
if(NOT nlohmann_json_FOUND)
    message(STATUS "nlohmann_json not found, installing...")
    execute_process(COMMAND sudo apt-get install -y nlohmann-json3-dev)
    find_package(nlohmann_json 3.10.0 REQUIRED)
endif()

# OpenCV ArUco
find_package(OpenCV REQUIRED COMPONENTS core calib3d aruco)
if(NOT OpenCV_aruco_FOUND)
    message(FATAL_ERROR "OpenCV aruco module not found - install opencv-contrib")
endif()
```

### Directories to Create:
```bash
# Calibration dataset storage
sudo mkdir -p /unlook_calib_dataset
sudo chmod 777 /unlook_calib_dataset

# Calibration files storage
sudo mkdir -p /unlook_calib
sudo chmod 777 /unlook_calib
```

---

## 📋 EXECUTION PROTOCOL

### STEP 0: INSTALL DEPENDENCIES
**Prima di iniziare, cmake-build-system-architect deve verificare e installare:**
- nlohmann-json3-dev
- OpenCV aruco module
- Creare directories /unlook_calib_dataset e /unlook_calib

### STEP 1: AGENT COORDINATION (PARALLEL EXECUTION)

**Launch questi agent IN PARALLELO con singolo messaggio:**

1. **stereo-calibration-specialist**
   - Task: Implementare backend completo calibrazione stereo
   - Input: OpenCV calibration, BoofCV research, ChArUco support
   - Output: StereoCalibrationProcessor class con validazione qualità

2. **ux-ui-design-architect**
   - Task: Implementare GUI calibration system (2 widget: capture + processing)
   - **CRITICAL:** Sostituire "Face Enrollment" con "Calibration" in main_window
   - Input: UX requirements (sotto)
   - Output: CalibrationWidget, DatasetCaptureWidget, DatasetProcessingWidget

3. **camera-sync-manager**
   - Task: Implementare auto-capture con LED sync e preview overlay
   - Input: Camera preview requirements, LED control
   - Output: CalibrationCaptureManager con pattern detection overlay

4. **technical-documentation-specialist**
   - Task: Documentare calibration workflow, parametri, troubleshooting
   - Input: Calibration process, validation criteria
   - Output: Calibration guide, JSON schema documentation

**IMPORTANTE:** Tutti gli agent devono lavorare in parallelo. Build CMake solo DOPO che tutti hanno finito.

---

## 🔬 CALIBRATION REQUIREMENTS

### Pattern Types Supportati:
1. **Checkerboard (Classic)**
   - Standard OpenCV checkerboard
   - Configurabile: rows, cols, square_size_mm
   - Veloce, affidabile
   - Default: 9x6 pattern, 25mm squares

2. **ChArUco (Advanced - RECOMMENDED)**
   - ArUco markers + checkerboard
   - Robust to partial occlusion
   - Better for VCSEL (markers visibili con IR)
   - Default: 7x10 pattern, 24mm squares, ArUco 17mm
   - Dictionary: DICT_4X4_250

3. **Circle Grid (Optional futuro)**
   - Asymmetric circle pattern
   - Subpixel accuracy eccellente

### Calibration Parameters Output:
```yaml
# /unlook_calib/calib-TIMESTAMP.yaml

calibration_date: "2025-01-04T15:30:00"
dataset_timestamp: "20250104_153000"
dataset_path: "/unlook_calib_dataset/dataset_20250104_153000"

# Pattern info
pattern_type: "charuco"  # checkerboard | charuco | circle_grid
pattern_rows: 7
pattern_cols: 10
square_size_mm: 24.0
aruco_marker_size_mm: 17.0
aruco_dictionary: "DICT_4X4_250"

# Image info
image_width: 1280
image_height: 720
num_image_pairs: 50
valid_image_pairs: 48

# Camera intrinsics LEFT
camera_matrix_left:
  fx: 1220.5
  fy: 1221.2
  cx: 640.1
  cy: 360.3

distortion_coeffs_left:
  k1: -0.15
  k2: 0.08
  p1: 0.001
  p2: -0.002
  k3: -0.02

# Camera intrinsics RIGHT
camera_matrix_right:
  fx: 1218.9
  fy: 1219.6
  cx: 638.7
  cy: 359.8

distortion_coeffs_right:
  k1: -0.14
  k2: 0.07
  p1: 0.0009
  p2: -0.0018
  k3: -0.018

# Stereo extrinsics
rotation_matrix:
  - [0.9998, -0.0001, 0.0201]
  - [0.0001, 1.0000, 0.0003]
  - [-0.0201, -0.0003, 0.9998]

translation_vector:
  tx: -70.12  # Baseline in mm (CRITICAL!)
  ty: 0.15
  tz: -0.08

# Essential and Fundamental matrices
essential_matrix:
  - [...]
  - [...]
  - [...]

fundamental_matrix:
  - [...]
  - [...]
  - [...]

# Rectification
rectification_transform_left:
  - [...]
  - [...]
  - [...]

rectification_transform_right:
  - [...]
  - [...]
  - [...]

projection_matrix_left:
  - [...]
  - [...]
  - [...]
  - [...]

projection_matrix_right:
  - [...]
  - [...]
  - [...]
  - [...]

disparity_to_depth_matrix:
  - [...]
  - [...]
  - [...]
  - [...]

# Rectification maps (save separately as binary)
rectification_map_left_x: "calib-TIMESTAMP-map-left-x.bin"
rectification_map_left_y: "calib-TIMESTAMP-map-left-y.bin"
rectification_map_right_x: "calib-TIMESTAMP-map-right-x.bin"
rectification_map_right_y: "calib-TIMESTAMP-map-right-y.bin"

# Quality metrics
rms_reprojection_error: 0.28  # pixels (TARGET: < 0.3)
mean_reprojection_error_left: 0.26
mean_reprojection_error_right: 0.29
max_reprojection_error: 0.87

# Epipolar validation
mean_epipolar_error: 0.15  # pixels (TARGET: < 0.5)
max_epipolar_error: 0.42

# Baseline validation
baseline_mm: 70.12
baseline_expected_mm: 70.0
baseline_error_mm: 0.12  # TARGET: < 0.5mm
baseline_error_percent: 0.17

# Validation flags
quality_passed: true
rms_check: "PASS"  # PASS | WARNING | FAIL
baseline_check: "PASS"
epipolar_check: "PASS"

# Warnings/Errors
warnings:
  - "Image pair 23: Low corner detection count (42/70)"
  - "Image pair 31: Slight motion blur detected"

errors: []

# System info
opencv_version: "4.6.0"
calibration_method: "opencv_stereo_calibrate"  # or boofcv_jni
calibration_flags: ["FIX_INTRINSIC", "CALIB_FIX_PRINCIPAL_POINT"]
calibration_duration_seconds: 45.3
```

### Dataset JSON Format:
```json
{
  "dataset_info": {
    "timestamp": "20250104_153000",
    "creation_date": "2025-01-04T15:30:00",
    "dataset_path": "/unlook_calib_dataset/dataset_20250104_153000"
  },
  "pattern_config": {
    "type": "charuco",
    "rows": 7,
    "cols": 10,
    "square_size_mm": 24.0,
    "aruco_marker_size_mm": 17.0,
    "aruco_dictionary": "DICT_4X4_250"
  },
  "capture_config": {
    "image_width": 1280,
    "image_height": 720,
    "capture_delay_seconds": 5,
    "target_image_pairs": 50,
    "captured_image_pairs": 50,
    "vcsel_enabled": true,
    "vcsel_current_ma": 280
  },
  "image_pairs": [
    {
      "index": 0,
      "left_filename": "left/frame_000.png",
      "right_filename": "right/frame_000.png",
      "timestamp": "2025-01-04T15:30:05",
      "corners_detected_left": 70,
      "corners_detected_right": 70,
      "quality_score": 0.95
    },
    ...
  ],
  "quality_summary": {
    "total_pairs": 50,
    "valid_pairs": 48,
    "mean_corners_detected": 68.5,
    "mean_quality_score": 0.92
  }
}
```

---

## 💻 CODE IMPLEMENTATION

### AGENT 1: stereo-calibration-specialist

**Files to CREATE:**
- `src/calibration/StereoCalibrationProcessor.cpp` (NEW)
- `include/unlook/calibration/StereoCalibrationProcessor.hpp` (NEW)
- `src/calibration/CalibrationValidator.cpp` (NEW)
- `include/unlook/calibration/CalibrationValidator.hpp` (NEW)
- `src/calibration/PatternDetector.cpp` (NEW)
- `include/unlook/calibration/PatternDetector.hpp` (NEW)

**Implementation:**

```cpp
namespace unlook::calibration {

// Pattern types
enum class PatternType {
    CHECKERBOARD,
    CHARUCO,
    CIRCLE_GRID
};

// Pattern configuration
struct PatternConfig {
    PatternType type;
    int rows;
    int cols;
    float squareSizeMM;
    float arucoMarkerSizeMM;  // For ChArUco
    cv::aruco::PREDEFINED_DICTIONARY_NAME arucoDict;  // For ChArUco

    // Defaults
    PatternConfig()
        : type(PatternType::CHARUCO)
        , rows(7)
        , cols(10)
        , squareSizeMM(24.0f)
        , arucoMarkerSizeMM(17.0f)
        , arucoDict(cv::aruco::DICT_4X4_250)
    {}
};

// Calibration result
struct CalibrationResult {
    // Timestamps
    std::string calibrationDate;
    std::string datasetTimestamp;
    std::string datasetPath;

    // Pattern info
    PatternConfig patternConfig;

    // Image info
    cv::Size imageSize;
    int numImagePairs;
    int validImagePairs;

    // Camera matrices
    cv::Mat cameraMatrixLeft;
    cv::Mat distCoeffsLeft;
    cv::Mat cameraMatrixRight;
    cv::Mat distCoeffsRight;

    // Stereo extrinsics
    cv::Mat R;  // Rotation
    cv::Mat T;  // Translation (baseline!)
    cv::Mat E;  // Essential matrix
    cv::Mat F;  // Fundamental matrix

    // Rectification
    cv::Mat R1, R2;  // Rectification transforms
    cv::Mat P1, P2;  // Projection matrices
    cv::Mat Q;       // Disparity-to-depth matrix

    // Rectification maps (for remap)
    cv::Mat map1Left, map2Left;
    cv::Mat map1Right, map2Right;

    // Quality metrics
    double rmsReprojectionError;
    double meanReprojectionErrorLeft;
    double meanReprojectionErrorRight;
    double maxReprojectionError;

    double meanEpipolarError;
    double maxEpipolarError;

    // Baseline validation
    double baselineMM;
    double baselineExpectedMM;
    double baselineErrorMM;
    double baselineErrorPercent;

    // Validation flags
    bool qualityPassed;
    std::string rmsCheck;       // "PASS" | "WARNING" | "FAIL"
    std::string baselineCheck;
    std::string epipolarCheck;

    std::vector<std::string> warnings;
    std::vector<std::string> errors;

    // Timing
    double calibrationDurationSeconds;
};

class StereoCalibrationProcessor {
public:
    StereoCalibrationProcessor();

    // Main calibration function
    CalibrationResult calibrateFromDataset(const std::string& datasetPath);

    // Save/load calibration
    void saveCalibration(const CalibrationResult& result,
                        const std::string& outputPath);
    CalibrationResult loadCalibration(const std::string& calibPath);

    // Set as system default
    void setAsSystemDefault(const std::string& calibPath);

private:
    // Pattern detection
    bool detectPattern(const cv::Mat& image,
                      const PatternConfig& config,
                      std::vector<cv::Point2f>& corners);

    // Checkerboard detection
    bool detectCheckerboard(const cv::Mat& image,
                           const PatternConfig& config,
                           std::vector<cv::Point2f>& corners);

    // ChArUco detection (RECOMMENDED)
    bool detectCharuco(const cv::Mat& image,
                      const PatternConfig& config,
                      std::vector<cv::Point2f>& corners);

    // Process dataset
    bool loadDatasetImages(const std::string& datasetPath,
                          std::vector<cv::Mat>& leftImages,
                          std::vector<cv::Mat>& rightImages);

    // Calibration steps
    bool calibrateIntrinsics(const std::vector<std::vector<cv::Point2f>>& imagePoints,
                            const std::vector<std::vector<cv::Point3f>>& objectPoints,
                            const cv::Size& imageSize,
                            cv::Mat& cameraMatrix,
                            cv::Mat& distCoeffs);

    bool calibrateStereo(const std::vector<std::vector<cv::Point2f>>& leftPoints,
                        const std::vector<std::vector<cv::Point2f>>& rightPoints,
                        const std::vector<std::vector<cv::Point3f>>& objectPoints,
                        const cv::Size& imageSize,
                        CalibrationResult& result);

    // Validation
    void validateCalibration(CalibrationResult& result);
    double computeEpipolarError(const CalibrationResult& result,
                               const std::vector<cv::Point2f>& leftPoints,
                               const std::vector<cv::Point2f>& rightPoints);

    // JSON I/O
    nlohmann::json loadDatasetInfo(const std::string& datasetPath);
    void saveCalibrationYAML(const CalibrationResult& result,
                            const std::string& outputPath);
};

class CalibrationValidator {
public:
    struct ValidationCriteria {
        double maxRMSError = 0.3;           // pixels
        double maxEpipolarError = 0.5;      // pixels
        double maxBaselineErrorMM = 0.5;    // mm
        double maxBaselineErrorPercent = 1.0; // %
        int minValidImagePairs = 30;
    };

    bool validate(const CalibrationResult& result,
                 const ValidationCriteria& criteria);

    std::vector<std::string> getWarnings() const { return warnings_; }
    std::vector<std::string> getErrors() const { return errors_; }

private:
    std::vector<std::string> warnings_;
    std::vector<std::string> errors_;
};

class PatternDetector {
public:
    PatternDetector(const PatternConfig& config);

    // Detect pattern and draw overlay
    bool detect(const cv::Mat& image,
               std::vector<cv::Point2f>& corners,
               cv::Mat& overlayImage);

    // Get detection confidence score
    float getConfidenceScore() const { return confidenceScore_; }

private:
    PatternConfig config_;
    float confidenceScore_;

    // ChArUco detector
    cv::Ptr<cv::aruco::Dictionary> arucoDict_;
    cv::Ptr<cv::aruco::CharucoBoard> charucoBoard_;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
};

} // namespace unlook::calibration
```

**Main Calibration Algorithm:**

```cpp
CalibrationResult StereoCalibrationProcessor::calibrateFromDataset(
    const std::string& datasetPath) {

    auto startTime = std::chrono::steady_clock::now();

    CalibrationResult result;
    result.datasetPath = datasetPath;

    // 1. Load dataset info JSON
    core::Logger::getInstance().info("Loading dataset info...");
    auto datasetInfo = loadDatasetInfo(datasetPath);
    result.datasetTimestamp = datasetInfo["dataset_info"]["timestamp"];

    // Parse pattern config
    result.patternConfig.type = parsePatternType(
        datasetInfo["pattern_config"]["type"]);
    result.patternConfig.rows = datasetInfo["pattern_config"]["rows"];
    result.patternConfig.cols = datasetInfo["pattern_config"]["cols"];
    result.patternConfig.squareSizeMM =
        datasetInfo["pattern_config"]["square_size_mm"];

    // 2. Load all image pairs
    core::Logger::getInstance().info("Loading image pairs...");
    std::vector<cv::Mat> leftImages, rightImages;
    loadDatasetImages(datasetPath, leftImages, rightImages);

    result.numImagePairs = leftImages.size();
    result.imageSize = leftImages[0].size();

    // 3. Detect patterns in all images
    core::Logger::getInstance().info("Detecting calibration patterns...");
    std::vector<std::vector<cv::Point2f>> leftImagePoints;
    std::vector<std::vector<cv::Point2f>> rightImagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;

    int validPairs = 0;
    for (size_t i = 0; i < leftImages.size(); i++) {
        std::vector<cv::Point2f> cornersLeft, cornersRight;

        bool leftDetected = detectPattern(leftImages[i], result.patternConfig,
                                         cornersLeft);
        bool rightDetected = detectPattern(rightImages[i], result.patternConfig,
                                          cornersRight);

        if (leftDetected && rightDetected) {
            leftImagePoints.push_back(cornersLeft);
            rightImagePoints.push_back(cornersRight);

            // Generate 3D object points
            std::vector<cv::Point3f> objPoints;
            for (int row = 0; row < result.patternConfig.rows; row++) {
                for (int col = 0; col < result.patternConfig.cols; col++) {
                    objPoints.push_back(cv::Point3f(
                        col * result.patternConfig.squareSizeMM,
                        row * result.patternConfig.squareSizeMM,
                        0.0f
                    ));
                }
            }
            objectPoints.push_back(objPoints);

            validPairs++;
            core::Logger::getInstance().debug("Pair " + std::to_string(i) +
                                             ": Pattern detected");
        } else {
            core::Logger::getInstance().warn("Pair " + std::to_string(i) +
                                            ": Pattern detection failed");
            result.warnings.push_back("Image pair " + std::to_string(i) +
                                     ": Pattern not detected");
        }
    }

    result.validImagePairs = validPairs;

    if (validPairs < 30) {
        result.errors.push_back("Insufficient valid image pairs: " +
                               std::to_string(validPairs) + " (need >= 30)");
        result.qualityPassed = false;
        return result;
    }

    core::Logger::getInstance().info("Valid image pairs: " +
                                    std::to_string(validPairs) + "/" +
                                    std::to_string(result.numImagePairs));

    // 4. Calibrate left camera intrinsics
    core::Logger::getInstance().info("Calibrating left camera intrinsics...");
    calibrateIntrinsics(leftImagePoints, objectPoints, result.imageSize,
                       result.cameraMatrixLeft, result.distCoeffsLeft);

    // 5. Calibrate right camera intrinsics
    core::Logger::getInstance().info("Calibrating right camera intrinsics...");
    calibrateIntrinsics(rightImagePoints, objectPoints, result.imageSize,
                       result.cameraMatrixRight, result.distCoeffsRight);

    // 6. Stereo calibration
    core::Logger::getInstance().info("Performing stereo calibration...");
    calibrateStereo(leftImagePoints, rightImagePoints, objectPoints,
                   result.imageSize, result);

    // 7. Compute rectification
    core::Logger::getInstance().info("Computing rectification transforms...");
    cv::stereoRectify(
        result.cameraMatrixLeft, result.distCoeffsLeft,
        result.cameraMatrixRight, result.distCoeffsRight,
        result.imageSize, result.R, result.T,
        result.R1, result.R2, result.P1, result.P2, result.Q,
        cv::CALIB_ZERO_DISPARITY, -1, result.imageSize
    );

    // 8. Compute rectification maps
    core::Logger::getInstance().info("Computing rectification maps...");
    cv::initUndistortRectifyMap(
        result.cameraMatrixLeft, result.distCoeffsLeft,
        result.R1, result.P1, result.imageSize, CV_32FC1,
        result.map1Left, result.map2Left
    );
    cv::initUndistortRectifyMap(
        result.cameraMatrixRight, result.distCoeffsRight,
        result.R2, result.P2, result.imageSize, CV_32FC1,
        result.map1Right, result.map2Right
    );

    // 9. Validate calibration
    core::Logger::getInstance().info("Validating calibration quality...");
    validateCalibration(result);

    // 10. Compute baseline
    result.baselineMM = std::abs(result.T.at<double>(0));
    result.baselineExpectedMM = 70.0;  // Expected baseline
    result.baselineErrorMM = std::abs(result.baselineMM - result.baselineExpectedMM);
    result.baselineErrorPercent = (result.baselineErrorMM / result.baselineExpectedMM) * 100.0;

    // Baseline check
    if (result.baselineErrorMM < 0.5) {
        result.baselineCheck = "PASS";
    } else if (result.baselineErrorMM < 1.0) {
        result.baselineCheck = "WARNING";
        result.warnings.push_back("Baseline error " +
                                 std::to_string(result.baselineErrorMM) +
                                 "mm exceeds 0.5mm threshold");
    } else {
        result.baselineCheck = "FAIL";
        result.errors.push_back("Baseline error " +
                               std::to_string(result.baselineErrorMM) +
                               "mm exceeds 1.0mm limit");
    }

    auto endTime = std::chrono::steady_clock::now();
    result.calibrationDurationSeconds =
        std::chrono::duration<double>(endTime - startTime).count();

    core::Logger::getInstance().info("Calibration completed in " +
                                    std::to_string(result.calibrationDurationSeconds) +
                                    " seconds");

    return result;
}
```

### AGENT 2: ux-ui-design-architect

**Files to DELETE (OLD FACE ENROLLMENT):**
- Remove "Face Enrollment" button from main_window
- Remove any face enrollment widget references

**Files to CREATE (CALIBRATION SYSTEM):**
- `src/gui/calibration_widget.cpp` (NEW - Tab container)
- `include/unlook/gui/calibration_widget.hpp` (NEW)
- `src/gui/dataset_capture_widget.cpp` (NEW)
- `include/unlook/gui/dataset_capture_widget.hpp` (NEW)
- `src/gui/ui/dataset_capture_widget.ui` (NEW)
- `src/gui/dataset_processing_widget.cpp` (NEW)
- `include/unlook/gui/dataset_processing_widget.hpp` (NEW)
- `src/gui/ui/dataset_processing_widget.ui` (NEW)

**Files to MODIFY:**
- `src/gui/main_window.cpp` - Add "Calibration" button, remove "Face Enrollment"
- `include/unlook/gui/main_window.hpp` - Add calibration widget member

**GUI Architecture:**

```cpp
namespace unlook::gui {

// Main calibration widget (tab container)
class CalibrationWidget : public QWidget {
    Q_OBJECT
public:
    CalibrationWidget(QWidget* parent = nullptr);

private:
    QTabWidget* tabWidget_;
    DatasetCaptureWidget* captureWidget_;
    DatasetProcessingWidget* processingWidget_;
};

// Dataset capture widget
class DatasetCaptureWidget : public QWidget {
    Q_OBJECT
public:
    DatasetCaptureWidget(QWidget* parent = nullptr);

signals:
    void datasetCaptureCompleted(const QString& datasetPath);

private slots:
    void onStartCapture();
    void onCaptureFrame();
    void onPatternTypeChanged(int index);

private:
    // Preview
    QLabel* leftPreview_;
    QLabel* rightPreview_;
    QLabel* patternOverlayLeft_;
    QLabel* patternOverlayRight_;

    // Controls (NO KEYBOARD - Qt widgets only!)
    QComboBox* patternTypeCombo_;         // Checkerboard | ChArUco | Circle Grid
    QSpinBox* rowsSpinBox_;               // Pattern rows
    QSpinBox* colsSpinBox_;               // Pattern cols
    QDoubleSpinBox* squareSizeSpinBox_;   // Square size (mm)
    QDoubleSpinBox* arucoSizeSpinBox_;    // ArUco marker size (mm) - for ChArUco

    // Capture controls
    QPushButton* startCaptureButton_;
    QProgressBar* captureProgress_;
    QLabel* statusLabel_;
    QLabel* detectionStatusLabel_;        // "Pattern detected ✓" or "Not detected ✗"

    // Capture state
    bool isCapturing_;
    int captureCount_;
    int targetCaptures_;
    QTimer* captureTimer_;

    // Camera and LED
    CameraSystem* cameraSystem_;
    AS1170Controller* ledController_;

    // Pattern detector
    calibration::PatternDetector* patternDetector_;

    // Dataset storage
    QString currentDatasetPath_;
    nlohmann::json datasetInfo_;

    // Methods
    void setupUi();
    void updatePreview();
    void captureAndSaveFrame();
    void saveDatasetInfo();
};

// Dataset processing widget
class DatasetProcessingWidget : public QWidget {
    Q_OBJECT
public:
    DatasetProcessingWidget(QWidget* parent = nullptr);

public slots:
    void loadAndProcessDataset(const QString& datasetPath = "");

private slots:
    void onProcessDataset();
    void onSelectDataset();

private:
    // Dataset info display
    QLabel* datasetPathLabel_;
    QLabel* datasetTimestampLabel_;
    QLabel* imagePairsLabel_;
    QLabel* patternInfoLabel_;

    // Processing controls
    QPushButton* processButton_;
    QPushButton* selectDatasetButton_;
    QProgressBar* processingProgress_;

    // Log output
    QTextEdit* logOutput_;

    // Calibration results display
    QGroupBox* resultsGroup_;
    QLabel* rmsErrorLabel_;
    QLabel* baselineLabel_;
    QLabel* epipolarErrorLabel_;
    QLabel* qualityStatusLabel_;

    // Statistics table
    QTableWidget* statisticsTable_;

    // Processor
    calibration::StereoCalibrationProcessor* processor_;

    // Methods
    void setupUi();
    void loadLatestDataset();
    void displayResults(const calibration::CalibrationResult& result);
    void updateLog(const QString& message);
};

} // namespace unlook::gui
```

**Dataset Capture Widget Implementation:**

```cpp
void DatasetCaptureWidget::setupUi() {
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // ===== PREVIEW SECTION =====
    QHBoxLayout* previewLayout = new QHBoxLayout();

    // Left camera preview
    QVBoxLayout* leftLayout = new QVBoxLayout();
    QLabel* leftLabel = new QLabel("Left Camera");
    leftLabel->setStyleSheet("font-weight: bold; font-size: 14pt;");
    leftPreview_ = new QLabel();
    leftPreview_->setMinimumSize(640, 480);
    leftPreview_->setScaledContents(true);
    leftPreview_->setStyleSheet("border: 2px solid #333;");

    patternOverlayLeft_ = new QLabel();
    patternOverlayLeft_->setAlignment(Qt::AlignCenter);
    patternOverlayLeft_->setStyleSheet("font-size: 12pt; color: green;");

    leftLayout->addWidget(leftLabel);
    leftLayout->addWidget(leftPreview_);
    leftLayout->addWidget(patternOverlayLeft_);

    // Right camera preview
    QVBoxLayout* rightLayout = new QVBoxLayout();
    QLabel* rightLabel = new QLabel("Right Camera");
    rightLabel->setStyleSheet("font-weight: bold; font-size: 14pt;");
    rightPreview_ = new QLabel();
    rightPreview_->setMinimumSize(640, 480);
    rightPreview_->setScaledContents(true);
    rightPreview_->setStyleSheet("border: 2px solid #333;");

    patternOverlayRight_ = new QLabel();
    patternOverlayRight_->setAlignment(Qt::AlignCenter);
    patternOverlayRight_->setStyleSheet("font-size: 12pt; color: green;");

    rightLayout->addWidget(rightLabel);
    rightLayout->addWidget(rightPreview_);
    rightLayout->addWidget(patternOverlayRight_);

    previewLayout->addLayout(leftLayout);
    previewLayout->addLayout(rightLayout);
    mainLayout->addLayout(previewLayout);

    // ===== PATTERN CONFIGURATION =====
    QGroupBox* configGroup = new QGroupBox("Pattern Configuration");
    QFormLayout* configLayout = new QFormLayout();

    patternTypeCombo_ = new QComboBox();
    patternTypeCombo_->addItem("Checkerboard", (int)calibration::PatternType::CHECKERBOARD);
    patternTypeCombo_->addItem("ChArUco (Recommended)", (int)calibration::PatternType::CHARUCO);
    patternTypeCombo_->addItem("Circle Grid", (int)calibration::PatternType::CIRCLE_GRID);
    patternTypeCombo_->setCurrentIndex(1);  // ChArUco default
    connect(patternTypeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &DatasetCaptureWidget::onPatternTypeChanged);

    rowsSpinBox_ = new QSpinBox();
    rowsSpinBox_->setRange(4, 20);
    rowsSpinBox_->setValue(7);

    colsSpinBox_ = new QSpinBox();
    colsSpinBox_->setRange(4, 20);
    colsSpinBox_->setValue(10);

    squareSizeSpinBox_ = new QDoubleSpinBox();
    squareSizeSpinBox_->setRange(5.0, 100.0);
    squareSizeSpinBox_->setValue(24.0);
    squareSizeSpinBox_->setSuffix(" mm");
    squareSizeSpinBox_->setDecimals(1);

    arucoSizeSpinBox_ = new QDoubleSpinBox();
    arucoSizeSpinBox_->setRange(5.0, 50.0);
    arucoSizeSpinBox_->setValue(17.0);
    arucoSizeSpinBox_->setSuffix(" mm");
    arucoSizeSpinBox_->setDecimals(1);

    configLayout->addRow("Pattern Type:", patternTypeCombo_);
    configLayout->addRow("Rows:", rowsSpinBox_);
    configLayout->addRow("Columns:", colsSpinBox_);
    configLayout->addRow("Square Size:", squareSizeSpinBox_);
    configLayout->addRow("ArUco Marker Size:", arucoSizeSpinBox_);

    configGroup->setLayout(configLayout);
    mainLayout->addWidget(configGroup);

    // ===== CAPTURE CONTROLS =====
    QHBoxLayout* controlsLayout = new QHBoxLayout();

    startCaptureButton_ = new QPushButton("Start Dataset Capture (50 pairs)");
    startCaptureButton_->setStyleSheet("font-size: 14pt; padding: 10px;");
    connect(startCaptureButton_, &QPushButton::clicked,
            this, &DatasetCaptureWidget::onStartCapture);

    controlsLayout->addWidget(startCaptureButton_);
    mainLayout->addLayout(controlsLayout);

    // ===== STATUS =====
    QHBoxLayout* statusLayout = new QHBoxLayout();

    statusLabel_ = new QLabel("Ready to capture");
    statusLabel_->setStyleSheet("font-size: 12pt;");

    detectionStatusLabel_ = new QLabel("Move checkerboard into view");
    detectionStatusLabel_->setStyleSheet("font-size: 12pt; color: orange;");

    statusLayout->addWidget(statusLabel_);
    statusLayout->addStretch();
    statusLayout->addWidget(detectionStatusLabel_);
    mainLayout->addLayout(statusLayout);

    // ===== PROGRESS =====
    captureProgress_ = new QProgressBar();
    captureProgress_->setRange(0, 50);
    captureProgress_->setValue(0);
    captureProgress_->setTextVisible(true);
    captureProgress_->setFormat("Captured: %v / %m pairs");
    mainLayout->addWidget(captureProgress_);

    // ===== PREVIEW TIMER =====
    QTimer* previewTimer = new QTimer(this);
    connect(previewTimer, &QTimer::timeout, this, &DatasetCaptureWidget::updatePreview);
    previewTimer->start(33);  // 30 FPS preview

    // ===== CAPTURE TIMER =====
    captureTimer_ = new QTimer(this);
    captureTimer_->setSingleShot(false);
    captureTimer_->setInterval(5000);  // 5 seconds between captures
    connect(captureTimer_, &QTimer::timeout, this, &DatasetCaptureWidget::onCaptureFrame);
}

void DatasetCaptureWidget::updatePreview() {
    // Capture frames from cameras
    auto leftFrame = cameraSystem_->captureLeft();
    auto rightFrame = cameraSystem_->captureRight();

    // Downsample to HD 1280x720
    cv::Mat leftHD, rightHD;
    cv::resize(leftFrame, leftHD, cv::Size(1280, 720), 0, 0, cv::INTER_AREA);
    cv::resize(rightFrame, rightHD, cv::Size(1280, 720), 0, 0, cv::INTER_AREA);

    // Detect pattern and draw overlay
    std::vector<cv::Point2f> cornersLeft, cornersRight;
    cv::Mat overlayLeft, overlayRight;

    bool leftDetected = patternDetector_->detect(leftHD, cornersLeft, overlayLeft);
    bool rightDetected = patternDetector_->detect(rightHD, cornersRight, overlayRight);

    // Update detection status
    if (leftDetected && rightDetected) {
        detectionStatusLabel_->setText("✓ Pattern Detected");
        detectionStatusLabel_->setStyleSheet("font-size: 12pt; color: green;");
        patternOverlayLeft_->setText("✓ Detected: " +
                                     QString::number(cornersLeft.size()) + " corners");
        patternOverlayRight_->setText("✓ Detected: " +
                                      QString::number(cornersRight.size()) + " corners");
    } else {
        detectionStatusLabel_->setText("✗ Pattern Not Detected");
        detectionStatusLabel_->setStyleSheet("font-size: 12pt; color: red;");
        patternOverlayLeft_->setText(leftDetected ? "✓ Detected" : "✗ Not detected");
        patternOverlayRight_->setText(rightDetected ? "✓ Detected" : "✗ Not detected");
    }

    // Convert to QPixmap and display
    leftPreview_->setPixmap(QPixmap::fromImage(
        QImage(overlayLeft.data, overlayLeft.cols, overlayLeft.rows,
               overlayLeft.step, QImage::Format_RGB888).rgbSwapped()));
    rightPreview_->setPixmap(QPixmap::fromImage(
        QImage(overlayRight.data, overlayRight.cols, overlayRight.rows,
               overlayRight.step, QImage::Format_RGB888).rgbSwapped()));
}

void DatasetCaptureWidget::onStartCapture() {
    if (isCapturing_) {
        // Stop capture
        captureTimer_->stop();
        isCapturing_ = false;
        startCaptureButton_->setText("Start Dataset Capture (50 pairs)");
        statusLabel_->setText("Capture stopped");
        return;
    }

    // Create dataset directory
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    currentDatasetPath_ = "/unlook_calib_dataset/dataset_" + timestamp;

    QDir().mkpath(currentDatasetPath_ + "/left");
    QDir().mkpath(currentDatasetPath_ + "/right");

    // Initialize dataset info JSON
    datasetInfo_["dataset_info"]["timestamp"] = timestamp.toStdString();
    datasetInfo_["dataset_info"]["creation_date"] =
        QDateTime::currentDateTime().toString(Qt::ISODate).toStdString();
    datasetInfo_["dataset_info"]["dataset_path"] =
        currentDatasetPath_.toStdString();

    // Pattern config
    datasetInfo_["pattern_config"]["type"] =
        patternTypeCombo_->currentText().toStdString();
    datasetInfo_["pattern_config"]["rows"] = rowsSpinBox_->value();
    datasetInfo_["pattern_config"]["cols"] = colsSpinBox_->value();
    datasetInfo_["pattern_config"]["square_size_mm"] = squareSizeSpinBox_->value();
    datasetInfo_["pattern_config"]["aruco_marker_size_mm"] = arucoSizeSpinBox_->value();

    // Capture config
    datasetInfo_["capture_config"]["image_width"] = 1280;   // HD resolution
    datasetInfo_["capture_config"]["image_height"] = 720;   // HD resolution
    datasetInfo_["capture_config"]["capture_delay_seconds"] = 5;
    datasetInfo_["capture_config"]["target_image_pairs"] = 50;
    datasetInfo_["capture_config"]["vcsel_enabled"] = true;
    datasetInfo_["capture_config"]["vcsel_current_ma"] = 280;

    // Enable VCSEL LED
    ledController_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1,
                               true, 280);

    // Start capture
    captureCount_ = 0;
    targetCaptures_ = 50;
    isCapturing_ = true;

    startCaptureButton_->setText("Stop Capture");
    statusLabel_->setText("Capturing... move checkerboard between captures");

    captureTimer_->start();

    // Capture first frame immediately
    onCaptureFrame();
}

void DatasetCaptureWidget::onCaptureFrame() {
    if (captureCount_ >= targetCaptures_) {
        // Capture complete
        captureTimer_->stop();
        isCapturing_ = false;

        // Disable VCSEL
        ledController_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1,
                                   false, 0);

        // Save dataset info
        saveDatasetInfo();

        startCaptureButton_->setText("Start Dataset Capture (50 pairs)");
        statusLabel_->setText("Dataset capture complete!");

        QMessageBox::information(this, "Capture Complete",
            "Dataset saved to:\n" + currentDatasetPath_ +
            "\n\nProceed to Processing tab to calibrate.");

        emit datasetCaptureCompleted(currentDatasetPath_);
        return;
    }

    // Capture and save frame pair
    captureAndSaveFrame();
    captureCount_++;
    captureProgress_->setValue(captureCount_);

    statusLabel_->setText("Captured " + QString::number(captureCount_) +
                         " / " + QString::number(targetCaptures_) +
                         " pairs - move checkerboard for next capture");
}

void DatasetCaptureWidget::captureAndSaveFrame() {
    // Capture frames
    auto leftFrame = cameraSystem_->captureLeft();
    auto rightFrame = cameraSystem_->captureRight();

    // Downsample to HD
    cv::Mat leftHD, rightHD;
    cv::resize(leftFrame, leftHD, cv::Size(1280, 720), 0, 0, cv::INTER_AREA);
    cv::resize(rightFrame, rightHD, cv::Size(1280, 720), 0, 0, cv::INTER_AREA);

    // Save images
    QString leftPath = currentDatasetPath_ + "/left/frame_" +
                      QString::number(captureCount_).rightJustified(3, '0') + ".png";
    QString rightPath = currentDatasetPath_ + "/right/frame_" +
                       QString::number(captureCount_).rightJustified(3, '0') + ".png";

    cv::imwrite(leftPath.toStdString(), leftHD);
    cv::imwrite(rightPath.toStdString(), rightHD);

    // Detect pattern for quality info
    std::vector<cv::Point2f> cornersLeft, cornersRight;
    cv::Mat dummy;
    bool leftDetected = patternDetector_->detect(leftHD, cornersLeft, dummy);
    bool rightDetected = patternDetector_->detect(rightHD, cornersRight, dummy);

    // Add to dataset info JSON
    nlohmann::json pairInfo;
    pairInfo["index"] = captureCount_;
    pairInfo["left_filename"] = ("left/frame_" +
        QString::number(captureCount_).rightJustified(3, '0') + ".png").toStdString();
    pairInfo["right_filename"] = ("right/frame_" +
        QString::number(captureCount_).rightJustified(3, '0') + ".png").toStdString();
    pairInfo["timestamp"] = QDateTime::currentDateTime().toString(Qt::ISODate).toStdString();
    pairInfo["corners_detected_left"] = cornersLeft.size();
    pairInfo["corners_detected_right"] = cornersRight.size();
    pairInfo["quality_score"] = (leftDetected && rightDetected) ?
        patternDetector_->getConfidenceScore() : 0.0;

    datasetInfo_["image_pairs"].push_back(pairInfo);
}
```

### AGENT 3: camera-sync-manager

**Task:** Integrate camera capture with LED control for calibration capture

**Implementation:** Already handled in DatasetCaptureWidget above (LED enable during capture)

### AGENT 4: technical-documentation-specialist

**Files to CREATE:**
- `docs/CALIBRATION_GUIDE.md` - Complete calibration workflow
- `docs/CALIBRATION_TROUBLESHOOTING.md` - Common issues and solutions
- `docs/CALIBRATION_PARAMETERS.md` - Explanation of all parameters

---

## 🚀 BUILD & DEPLOYMENT

### CMakeLists.txt Updates:
```cmake
# JSON library (nlohmann/json)
find_package(nlohmann_json 3.10.0 QUIET)
if(NOT nlohmann_json_FOUND)
    message(STATUS "nlohmann_json not found, installing...")
    execute_process(
        COMMAND sudo apt-get install -y nlohmann-json3-dev
    )
endif()

# OpenCV ArUco module (for ChArUco)
find_package(OpenCV REQUIRED aruco)

# Calibration library
add_library(unlook_calibration
    src/calibration/StereoCalibrationProcessor.cpp
    src/calibration/CalibrationValidator.cpp
    src/calibration/PatternDetector.cpp
)

target_link_libraries(unlook_calibration
    PRIVATE opencv_core opencv_calib3d opencv_aruco
    PRIVATE nlohmann_json::nlohmann_json
)

# GUI updates
add_library(unlook_gui
    ...
    src/gui/calibration_widget.cpp
    src/gui/dataset_capture_widget.cpp
    src/gui/dataset_processing_widget.cpp
)
```

### System Directories:
```bash
# Create calibration directories
mkdir -p /unlook_calib_dataset
mkdir -p /unlook_calib
chmod 777 /unlook_calib_dataset
chmod 777 /unlook_calib

# Symlink for system default calibration
ln -sf /unlook_calib/calib-latest.yaml /unlook_calib/default.yaml
```

---

## ✅ SUCCESS CRITERIA

### Functional Requirements:
- [x] Calibration button in main window (replace Face Enrollment)
- [x] Dataset capture widget with dual preview
- [x] Pattern detection overlay (real-time)
- [x] Auto-capture 50 pairs with 5 sec delay
- [x] Dataset saved with JSON metadata
- [x] Dataset processing widget with auto-load
- [x] Full stereo calibration pipeline
- [x] Validation with quality checks
- [x] YAML output with all parameters
- [x] System default integration
- [x] Statistics display in GUI

### Quality Requirements:
- [ ] RMS reprojection error < 0.3 pixels
- [ ] Baseline error < 0.5mm
- [ ] Epipolar error < 0.5 pixels
- [ ] Pattern detection success rate > 95%
- [ ] Processing time < 60 seconds

### Code Quality:
- [ ] No placeholder code
- [ ] All functions implemented
- [ ] Proper error handling
- [ ] JSON schema validation
- [ ] Logging at appropriate levels

---

## 📚 USAGE WORKFLOW

### User Workflow:
```
1. Main Window → Click "Calibration" button
   ↓
2. Calibration Widget → "Dataset Capture" tab
   ↓
3. Configure pattern (ChArUco recommended)
   - Rows: 7
   - Cols: 10
   - Square size: 24mm
   - ArUco size: 17mm
   ↓
4. Position checkerboard in front of cameras
   ↓
5. Wait for "✓ Pattern Detected" indicator
   ↓
6. Click "Start Dataset Capture (50 pairs)"
   ↓
7. System captures first frame immediately
   ↓
8. Move checkerboard to different position/angle
   ↓
9. Wait 5 seconds → auto-capture
   ↓
10. Repeat step 8-9 for 50 captures
   ↓
11. Dataset saved to /unlook_calib_dataset/dataset_TIMESTAMP/
   ↓
12. Switch to "Dataset Processing" tab
   ↓
13. Latest dataset auto-loaded
   ↓
14. Click "Process Dataset"
   ↓
15. Watch log output, see progress
   ↓
16. Calibration complete!
   ↓
17. View statistics: RMS error, baseline, epipolar error
   ↓
18. Calibration auto-saved to /unlook_calib/calib-TIMESTAMP.yaml
   ↓
19. System default updated automatically
   ↓
20. Done! Scanner now uses new calibration
```

---

## 🔧 MAIN WINDOW INTEGRATION

```cpp
// In main_window.cpp constructor

// REMOVE old Face Enrollment button
// DELETE any face enrollment references

// ADD Calibration button
QPushButton* calibrationButton = new QPushButton("Calibration");
calibrationButton->setStyleSheet("font-size: 16pt; padding: 10px;");
connect(calibrationButton, &QPushButton::clicked, this, [this]() {
    if (!calibrationWidget_) {
        calibrationWidget_ = new CalibrationWidget(this);
    }
    central_widget_->addTab(calibrationWidget_, "Calibration");
    central_widget_->setCurrentWidget(calibrationWidget_);
});
mainLayout->addWidget(calibrationButton);
```

---

## 🎓 VALIDATION LOGIC

```cpp
void CalibrationValidator::validate(const CalibrationResult& result,
                                   const ValidationCriteria& criteria) {
    warnings_.clear();
    errors_.clear();

    // 1. RMS Error Check
    if (result.rmsReprojectionError > criteria.maxRMSError) {
        if (result.rmsReprojectionError > criteria.maxRMSError * 2) {
            errors_.push_back("RMS error " +
                std::to_string(result.rmsReprojectionError) +
                " pixels EXCEEDS limit (max: " +
                std::to_string(criteria.maxRMSError) + ")");
        } else {
            warnings_.push_back("RMS error " +
                std::to_string(result.rmsReprojectionError) +
                " pixels above recommended (target: " +
                std::to_string(criteria.maxRMSError) + ")");
        }
    }

    // 2. Epipolar Error Check
    if (result.meanEpipolarError > criteria.maxEpipolarError) {
        warnings_.push_back("Epipolar error " +
            std::to_string(result.meanEpipolarError) +
            " pixels above target (max: " +
            std::to_string(criteria.maxEpipolarError) + ")");
    }

    // 3. Baseline Check
    if (result.baselineErrorMM > criteria.maxBaselineErrorMM) {
        if (result.baselineErrorMM > criteria.maxBaselineErrorMM * 2) {
            errors_.push_back("Baseline error " +
                std::to_string(result.baselineErrorMM) +
                "mm CRITICAL (max: " +
                std::to_string(criteria.maxBaselineErrorMM) + "mm)");
        } else {
            warnings_.push_back("Baseline error " +
                std::to_string(result.baselineErrorMM) +
                "mm above tolerance");
        }
    }

    // 4. Valid Pairs Check
    if (result.validImagePairs < criteria.minValidImagePairs) {
        errors_.push_back("Insufficient valid image pairs: " +
            std::to_string(result.validImagePairs) +
            " (need >= " +
            std::to_string(criteria.minValidImagePairs) + ")");
    }

    // Overall quality
    result.qualityPassed = errors_.empty();
}
```

---

## 🚀 EXECUTION ORDER

**PARALLEL PHASE:**
1. stereo-calibration-specialist → Backend calibration
2. ux-ui-design-architect → GUI widgets
3. camera-sync-manager → LED integration
4. technical-documentation-specialist → Documentation

**SEQUENTIAL PHASE:**
5. code-integrity-architect → Code review
6. cmake-build-system-architect → Install dependencies + Build system
   - Install nlohmann-json3-dev se mancante
   - Verify OpenCV aruco module
   - Create /unlook_calib_dataset and /unlook_calib directories
   - Update CMakeLists.txt
7. BUILD → `./build.sh --clean -j4`
8. testing-validation-framework → Validation tests

---

## 💬 FINAL NOTES

**NO PLACEHOLDER CODE.** Everything must be fully implemented and working.

**NO FAKE CODE.** All calibration algorithms must be real OpenCV/BoofCV calls.

**EASE OF USE.** User should be able to calibrate in < 5 minutes total.

**AUTO-EVERYTHING.** Auto-capture, auto-process, auto-validate, auto-integrate.

**PROFESSIONAL OUTPUT.** YAML with ALL parameters stereo scanner needs.

**ROBUST VALIDATION.** Clear feedback on calibration quality.

---

## 🎯 READY TO EXECUTE!

Launch agents in parallel, implement everything, build at the end.

**FORZA UNLOOK CALIBRATION! 🎯📐✨**
