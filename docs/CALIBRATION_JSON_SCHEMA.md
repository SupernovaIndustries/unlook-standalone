# Stereo Calibration JSON Schema Documentation

## Overview

The Unlook calibration system uses JSON metadata to track dataset information, pattern configuration, and capture parameters. This document defines the complete schema for `dataset_info.json` files.

---

## JSON Schema Definition

### Root Object: `dataset_info.json`

```json
{
  "dataset_info": {
    "timestamp": "string (YYYYMMDD_HHMMSS)",
    "creation_date": "string (ISO 8601)",
    "dataset_path": "string (absolute path)"
  },
  "pattern_config": {
    "type": "string (charuco|checkerboard|circle_grid)",
    "rows": "integer (4-20)",
    "cols": "integer (4-20)",
    "square_size_mm": "number (5.0-100.0)",
    "aruco_marker_size_mm": "number (5.0-50.0)",
    "aruco_dictionary": "string"
  },
  "capture_config": {
    "image_width": "integer (pixels)",
    "image_height": "integer (pixels)",
    "capture_delay_seconds": "integer",
    "target_image_pairs": "integer (typically 50)",
    "captured_image_pairs": "integer",
    "vcsel_enabled": "boolean",
    "vcsel_current_ma": "integer"
  },
  "image_pairs": [
    {
      "index": "integer",
      "left_filename": "string",
      "right_filename": "string",
      "timestamp": "string (ISO 8601)",
      "corners_detected_left": "integer",
      "corners_detected_right": "integer",
      "quality_score": "number (0.0-1.0)"
    }
  ],
  "quality_summary": {
    "total_pairs": "integer",
    "valid_pairs": "integer",
    "mean_corners_detected": "number",
    "mean_quality_score": "number"
  }
}
```

---

## Detailed Field Definitions

### dataset_info Object

Container for dataset metadata and location.

#### timestamp (String: YYYYMMDD_HHMMSS)

**Format**: Timestamp in `YYYYMMDD_HHMMSS` format

**Example**: `"20250104_153000"`

**Meaning**:
- YYYY: Year (4 digits)
- MM: Month (01-12)
- DD: Day (01-31)
- HH: Hour (00-23)
- MM: Minute (00-59)
- SS: Second (00-59)

**Purpose**: Unique dataset identifier, used for filenames

**Validation**:
```json
{
  "type": "string",
  "pattern": "^\\d{8}_\\d{6}$"
}
```

#### creation_date (String: ISO 8601)

**Format**: ISO 8601 datetime with timezone

**Example**: `"2025-01-04T15:30:00+01:00"`

**Meaning**:
- Date and time of dataset creation
- ISO 8601 format includes timezone information
- Enables accurate tracking across time zones

**Validation**:
```json
{
  "type": "string",
  "format": "date-time"
}
```

#### dataset_path (String: Absolute Path)

**Format**: Absolute filesystem path

**Example**: `"/unlook_calib_dataset/dataset_20250104_153000"`

**Meaning**:
- Complete path to dataset directory
- Should be readable and writable
- Contains `left/`, `right/`, and `dataset_info.json`

**Validation**:
```json
{
  "type": "string",
  "pattern": "^/unlook_calib_dataset/dataset_\\d{8}_\\d{6}$"
}
```

---

### pattern_config Object

Calibration pattern configuration and specifications.

#### type (String: Enum)

**Valid Values**:
- `"charuco"` - ArUco markers + checkerboard (RECOMMENDED)
- `"checkerboard"` - Classic black/white checkerboard
- `"circle_grid"` - Asymmetric circle pattern

**Default**: `"charuco"`

**Example**: `"charuco"`

**Meaning**: Pattern type used for corner detection

**Validation**:
```json
{
  "type": "string",
  "enum": ["charuco", "checkerboard", "circle_grid"]
}
```

#### rows (Integer: 4-20)

**Range**: 4 to 20 inclusive

**Typical Value**: 7

**Example**: `7`

**Meaning**: Number of pattern rows (vertical count)

**Constraints**:
- Must match printed pattern exactly
- Affects number of detection points (rows × cols)
- Example: 7×10 = 70 points per image

**Validation**:
```json
{
  "type": "integer",
  "minimum": 4,
  "maximum": 20
}
```

#### cols (Integer: 4-20)

**Range**: 4 to 20 inclusive

**Typical Value**: 10

**Example**: `10`

**Meaning**: Number of pattern columns (horizontal count)

**Constraints**:
- Must match printed pattern exactly
- Typically > rows for better horizontal coverage
- Example: 7×10 = 70 points per image

**Validation**:
```json
{
  "type": "integer",
  "minimum": 4,
  "maximum": 20
}
```

#### square_size_mm (Number: 5.0-100.0)

**Range**: 5.0 to 100.0 millimeters

**Typical Value**: 24.0

**Example**: `24.0`

**Unit**: Millimeters

**CRITICAL**: Must be measured with calipers and entered exactly

**Constraints**:
- Tolerance: ±0.5 mm maximum
- 1mm error = 1mm baseline error
- See CALIBRATION_PARAMETERS.md for details

**Validation**:
```json
{
  "type": "number",
  "minimum": 5.0,
  "maximum": 100.0,
  "multipleOf": 0.1
}
```

**Verification Example**:
```json
{
  "square_size_mm": 24.0,
  "note": "Measured with calipers: 23.95-24.05 mm average"
}
```

#### aruco_marker_size_mm (Number: 5.0-50.0)

**Range**: 5.0 to 50.0 millimeters

**Typical Value**: 17.0

**Example**: `17.0`

**Unit**: Millimeters

**Meaning**: Physical size of ArUco markers embedded in pattern

**Constraints**:
- Must be < square_size_mm
- Typical: 70% of square size
- Example: 24mm square -> 17mm marker

**Typical Relationships**:
```json
{
  "square_size_mm": 24.0,
  "aruco_marker_size_mm": 17.0,
  "ratio": 0.708
}
```

**Validation**:
```json
{
  "type": "number",
  "minimum": 5.0,
  "maximum": 50.0,
  "multipleOf": 0.1
}
```

#### aruco_dictionary (String: Enum)

**Valid Values**:
- `"DICT_4X4_50"` - 4×4 bits, 50 markers
- `"DICT_4X4_100"` - 4×4 bits, 100 markers
- `"DICT_4X4_250"` - 4×4 bits, 250 markers (RECOMMENDED)
- `"DICT_4X4_1000"` - 4×4 bits, 1000 markers
- `"DICT_5X5_100"` - 5×5 bits, 100 markers
- `"DICT_5X5_250"` - 5×5 bits, 250 markers
- `"DICT_6X6_250"` - 6×6 bits, 250 markers
- `"DICT_6X6_1000"` - 6×6 bits, 1000 markers

**Default**: `"DICT_4X4_250"`

**Example**: `"DICT_4X4_250"`

**Meaning**: ArUco marker dictionary specification

**Note**: Must match markers printed on pattern

**Validation**:
```json
{
  "type": "string",
  "enum": [
    "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
    "DICT_5X5_100", "DICT_5X5_250",
    "DICT_6X6_250", "DICT_6X6_1000"
  ]
}
```

---

### capture_config Object

Settings used during dataset capture.

#### image_width (Integer: Pixels)

**Typical Value**: 1280

**Example**: `1280`

**Unit**: Pixels

**Meaning**: Image resolution width (HD format)

**Note**: Images downsampled from 1456 to 1280 for processing

**Validation**:
```json
{
  "type": "integer",
  "minimum": 320,
  "maximum": 1920
}
```

#### image_height (Integer: Pixels)

**Typical Value**: 720

**Example**: `720`

**Unit**: Pixels

**Meaning**: Image resolution height (HD format)

**Note**: Images downsampled from 1088 to 720 for processing

**Validation**:
```json
{
  "type": "integer",
  "minimum": 240,
  "maximum": 1440
}
```

#### capture_delay_seconds (Integer: Seconds)

**Typical Value**: 5

**Example**: `5`

**Unit**: Seconds

**Meaning**: Delay between consecutive frame captures

**Purpose**: Allows user to reposition pattern between captures

**Validation**:
```json
{
  "type": "integer",
  "minimum": 1,
  "maximum": 30
}
```

#### target_image_pairs (Integer: Count)

**Typical Value**: 50

**Example**: `50`

**Meaning**: Number of image pair captures planned

**Note**: System will attempt to capture this many pairs

**Standard Values**:
- 30: Minimum for valid calibration
- 50: Standard recommendation
- 100: For extra-robust calibration

**Validation**:
```json
{
  "type": "integer",
  "minimum": 30,
  "maximum": 200
}
```

#### captured_image_pairs (Integer: Count)

**Example**: `50`

**Meaning**: Number of image pairs actually captured

**Note**: May be < target_image_pairs if user stopped early

**Validation**:
```json
{
  "type": "integer",
  "minimum": 0,
  "maximum": 200
}
```

#### vcsel_enabled (Boolean)

**Valid Values**: `true` or `false`

**Example**: `true`

**Meaning**: Whether VCSEL LED was enabled during capture

**Note**: Should always be `true` for calibration

**Validation**:
```json
{
  "type": "boolean"
}
```

#### vcsel_current_ma (Integer: Milliamps)

**Typical Value**: 280

**Example**: `280`

**Unit**: Milliamps

**Meaning**: VCSEL LED current setting

**Typical Range**: 200-350 mA

**Validation**:
```json
{
  "type": "integer",
  "minimum": 100,
  "maximum": 500
}
```

---

### image_pairs Array

List of captured image pair information.

#### image_pairs[n] Object

Individual image pair metadata.

**Structure:**
```json
{
  "index": 0,
  "left_filename": "left/frame_000.png",
  "right_filename": "right/frame_000.png",
  "timestamp": "2025-01-04T15:30:05+01:00",
  "corners_detected_left": 70,
  "corners_detected_right": 70,
  "quality_score": 0.95
}
```

#### index (Integer)

**Range**: 0 to (captured_image_pairs - 1)

**Example**: `0`

**Meaning**: Sequential index of image pair

**Validation**:
```json
{
  "type": "integer",
  "minimum": 0
}
```

#### left_filename (String)

**Format**: Relative path to left image

**Example**: `"left/frame_000.png"`

**Meaning**: Filename of left camera image

**Constraints**:
- Must be PNG format
- Must exist in dataset directory
- Naming: `left/frame_XXX.png` (3-digit zero-padded index)

**Validation**:
```json
{
  "type": "string",
  "pattern": "^left/frame_\\d{3}\\.png$"
}
```

#### right_filename (String)

**Format**: Relative path to right image

**Example**: `"right/frame_000.png"`

**Meaning**: Filename of right camera image

**Constraints**:
- Must be PNG format
- Must exist in dataset directory
- Naming: `right/frame_XXX.png` (3-digit zero-padded index)

**Validation**:
```json
{
  "type": "string",
  "pattern": "^right/frame_\\d{3}\\.png$"
}
```

#### timestamp (String: ISO 8601)

**Example**: `"2025-01-04T15:30:05+01:00"`

**Meaning**: Capture timestamp of this image pair

**Purpose**: Track when each pair was captured

**Validation**:
```json
{
  "type": "string",
  "format": "date-time"
}
```

#### corners_detected_left (Integer)

**Range**: 0 to (rows × cols)

**Example**: `70`

**Meaning**: Number of pattern corners detected in left image

**Expected Value**: 70 for ChArUco 7×10 pattern

**Interpretation**:
- 70: Perfect detection
- 60-69: Good detection (minor occlusion)
- 40-59: Partial detection (significant occlusion)
- <40: Poor detection (pattern not fully visible)

**Validation**:
```json
{
  "type": "integer",
  "minimum": 0,
  "maximum": 100
}
```

#### corners_detected_right (Integer)

**Range**: 0 to (rows × cols)

**Example**: `70`

**Meaning**: Number of pattern corners detected in right image

**Expected Value**: 70 for ChArUco 7×10 pattern

**Interpretation**: Same as corners_detected_left

**Validation**:
```json
{
  "type": "integer",
  "minimum": 0,
  "maximum": 100
}
```

#### quality_score (Number: 0.0-1.0)

**Range**: 0.0 to 1.0

**Example**: `0.95`

**Meaning**: Quality score of pattern detection

**Interpretation**:
- 1.0: Perfect detection (70/70 corners)
- 0.9-0.99: Excellent detection
- 0.8-0.89: Good detection
- 0.7-0.79: Acceptable detection
- 0.5-0.69: Marginal detection
- <0.5: Poor detection (should not be used)

**Calculation**:
```
quality_score = (corners_detected_left + corners_detected_right) /
                (2 * rows * cols)

For ChArUco 7×10:
- Both cameras detect all 70 corners:
  (70 + 70) / (2 * 70) = 1.0

- Left detects 70, right detects 60:
  (70 + 60) / (2 * 70) = 0.93
```

**Validation**:
```json
{
  "type": "number",
  "minimum": 0.0,
  "maximum": 1.0,
  "multipleOf": 0.01
}
```

---

### quality_summary Object

Overall quality statistics for the dataset.

#### total_pairs (Integer)

**Example**: `50`

**Meaning**: Total number of image pairs in dataset

**Equivalent to**: `captured_image_pairs`

**Validation**:
```json
{
  "type": "integer",
  "minimum": 0
}
```

#### valid_pairs (Integer)

**Example**: `48`

**Meaning**: Number of pairs with successful pattern detection

**Interpretation**:
- valid_pairs >= 30: Sufficient for calibration
- valid_pairs < 30: Insufficient (calibration will fail)

**Quality Assessment**:
```
valid_pairs / total_pairs percentage:

>95%: Excellent dataset
90-95%: Very good
80-90%: Good
70-80%: Acceptable
<70%: Poor (may need recapture)
```

**Validation**:
```json
{
  "type": "integer",
  "minimum": 0,
  "maximum": 200
}
```

#### mean_corners_detected (Number)

**Example**: `68.5`

**Meaning**: Average corners detected across all image pairs

**Calculation**:
```
mean = sum(corners_detected_left + corners_detected_right) /
       (2 * total_pairs)

Example:
- Total 50 pairs
- Sum of detections: 48×70 + 48×70 = 6720
- mean = 6720 / (2 * 50) = 67.2
```

**Interpretation**:
- >69: Excellent (very high detection rate)
- 65-69: Good (most patterns detected)
- 60-65: Acceptable
- <60: Poor (many patterns partially detected)

**Validation**:
```json
{
  "type": "number",
  "minimum": 0.0,
  "maximum": 100.0,
  "multipleOf": 0.1
}
```

#### mean_quality_score (Number: 0.0-1.0)

**Example**: `0.92`

**Meaning**: Average quality score across all image pairs

**Calculation**:
```
mean = sum(quality_score) / total_pairs
```

**Interpretation**:
- >0.95: Excellent quality
- 0.90-0.95: Very good
- 0.80-0.90: Good
- 0.70-0.80: Acceptable
- <0.70: Poor (should recapture)

**Validation**:
```json
{
  "type": "number",
  "minimum": 0.0,
  "maximum": 1.0,
  "multipleOf": 0.01
}
```

---

## Complete Schema Example

**Minimal Valid Dataset:**

```json
{
  "dataset_info": {
    "timestamp": "20250104_153000",
    "creation_date": "2025-01-04T15:30:00+01:00",
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
      "timestamp": "2025-01-04T15:30:05+01:00",
      "corners_detected_left": 70,
      "corners_detected_right": 70,
      "quality_score": 0.95
    },
    {
      "index": 1,
      "left_filename": "left/frame_001.png",
      "right_filename": "right/frame_001.png",
      "timestamp": "2025-01-04T15:30:10+01:00",
      "corners_detected_left": 70,
      "corners_detected_right": 69,
      "quality_score": 0.99
    }
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

## JSON Validation

### Using Python jsonschema

```python
import json
import jsonschema

# Define schema
schema = {
    "type": "object",
    "properties": {
        "dataset_info": {
            "type": "object",
            "properties": {
                "timestamp": {
                    "type": "string",
                    "pattern": "^\\d{8}_\\d{6}$"
                },
                "creation_date": {
                    "type": "string",
                    "format": "date-time"
                },
                "dataset_path": {
                    "type": "string"
                }
            },
            "required": ["timestamp", "creation_date", "dataset_path"]
        },
        "pattern_config": {
            "type": "object",
            "properties": {
                "type": {
                    "type": "string",
                    "enum": ["charuco", "checkerboard", "circle_grid"]
                },
                "rows": {"type": "integer", "minimum": 4, "maximum": 20},
                "cols": {"type": "integer", "minimum": 4, "maximum": 20},
                "square_size_mm": {"type": "number", "minimum": 5.0, "maximum": 100.0},
                "aruco_marker_size_mm": {"type": "number", "minimum": 5.0, "maximum": 50.0},
                "aruco_dictionary": {"type": "string"}
            },
            "required": ["type", "rows", "cols", "square_size_mm"]
        }
    },
    "required": ["dataset_info", "pattern_config", "capture_config", "image_pairs", "quality_summary"]
}

# Validate
with open('/unlook_calib_dataset/dataset_XXX/dataset_info.json') as f:
    data = json.load(f)

try:
    jsonschema.validate(instance=data, schema=schema)
    print("Valid dataset!")
except jsonschema.ValidationError as e:
    print(f"Invalid dataset: {e.message}")
```

### Using C++ nlohmann/json

```cpp
#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;

// Load and validate
std::ifstream ifs("/unlook_calib_dataset/dataset_XXX/dataset_info.json");
json data = json::parse(ifs);

// Access with type checking
try {
    std::string timestamp = data.at("dataset_info").at("timestamp");
    int rows = data.at("pattern_config").at("rows");
    int valid_pairs = data.at("quality_summary").at("valid_pairs");

    // Validate critical fields
    if (valid_pairs < 30) {
        std::cerr << "Error: insufficient valid pairs\n";
    }

    auto image_pairs = data.at("image_pairs");
    for (const auto& pair : image_pairs) {
        int index = pair.at("index");
        float quality = pair.at("quality_score");
    }

} catch (const json::exception& e) {
    std::cerr << "JSON error: " << e.what() << "\n";
}
```

---

## File Structure

### Dataset Directory Layout

```
/unlook_calib_dataset/
├── dataset_20250104_153000/
│   ├── dataset_info.json          <- This file
│   ├── left/
│   │   ├── frame_000.png
│   │   ├── frame_001.png
│   │   ├── ...
│   │   └── frame_049.png          <- 50 frames total
│   └── right/
│       ├── frame_000.png
│       ├── frame_001.png
│       ├── ...
│       └── frame_049.png          <- 50 frames total
└── dataset_20250105_091500/
    └── ...
```

### File Access Permissions

```bash
# Dataset directory should be readable and writable
ls -ld /unlook_calib_dataset/dataset_YYYYMMDD_HHMMSS/
# drwxr-xr-x user user ...

# JSON file should be readable
ls -l /unlook_calib_dataset/dataset_YYYYMMDD_HHMMSS/dataset_info.json
# -rw-r--r-- user user ...

# Image files should be readable
ls -l /unlook_calib_dataset/dataset_YYYYMMDD_HHMMSS/left/frame_000.png
# -rw-r--r-- user user ...
```

---

## Data Integrity

### Checksums (Optional)

Some datasets may include SHA256 checksums:

```json
{
  "checksum_info": {
    "algorithm": "sha256",
    "dataset_info_sha256": "abc123...",
    "image_pairs_count": 50,
    "images_verified": true
  }
}
```

### Consistency Checks

When loading a dataset, verify:

1. All image files exist
2. `captured_image_pairs` matches number of files
3. `valid_pairs` is within range [0, captured_image_pairs]
4. All image_pairs have valid indices [0, captured_image_pairs-1]
5. Pattern configuration is self-consistent

---

## Migration and Backwards Compatibility

### Version 1.0 (Current)

Current schema version for all new datasets.

### Handling Missing Fields

If processing legacy datasets with missing fields:

```python
# Provide defaults for missing optional fields
data = json.load(f)
vcsel_enabled = data.get("capture_config", {}).get("vcsel_enabled", True)
vcsel_current = data.get("capture_config", {}).get("vcsel_current_ma", 280)
```

---

## Common Issues

### Invalid Pattern Configuration

**Issue**: Pattern type mismatch
```json
"type": "charuco",
"aruco_dictionary": "DICT_4X4_250"
```

**Validation**: For ChArUco patterns, must specify aruco_dictionary

### Wrong Image Path Format

**Issue**: Incorrect path format
```json
"left_filename": "/unlook_calib_dataset/dataset_XXX/left/frame_000.png"  // WRONG
"left_filename": "left/frame_000.png"  // CORRECT
```

### Missing Timestamps

**Issue**: dataset_info.json missing creation_date
```json
{
  "dataset_info": {
    "timestamp": "20250104_153000",
    // ERROR: missing "creation_date"
    "dataset_path": "..."
  }
}
```

### Quality Score Inconsistency

**Issue**: Quality score doesn't match corner counts
```json
{
  "corners_detected_left": 35,  // Only 50% detected
  "corners_detected_right": 35,
  "quality_score": 0.99  // ERROR: should be ~0.5
}
```

**Correct**: Quality score should be:
```
(35 + 35) / (2 * 70) = 0.5
```

---

## Tools and Utilities

### JSON Pretty-Print (Linux/macOS)

```bash
# Format JSON for readability
python3 -m json.tool /unlook_calib_dataset/dataset_XXX/dataset_info.json

# Validate JSON syntax
python3 -c "import json; json.load(open('dataset_info.json'))"
```

### Statistics Extraction (Python)

```python
import json

with open('/unlook_calib_dataset/dataset_XXX/dataset_info.json') as f:
    data = json.load(f)

summary = data['quality_summary']
print(f"Total pairs: {summary['total_pairs']}")
print(f"Valid pairs: {summary['valid_pairs']}")
print(f"Success rate: {100*summary['valid_pairs']/summary['total_pairs']:.1f}%")
print(f"Mean quality: {summary['mean_quality_score']:.2f}")
```

---

**Document Version**: 1.0
**Last Updated**: 2025-01-04
**Status**: Production Ready
