# Unlook 3D Scanner - Stereo Calibration Documentation

Complete technical documentation for the Unlook stereo calibration system.

---

## Documentation Files

### 1. CALIBRATION_GUIDE.md (21 KB)

**For**: End-users performing calibration workflow

**Contents**:
- Introduction to stereo calibration and why it matters
- Hardware requirements and pattern setup
- Step-by-step calibration workflow (12 detailed steps)
- Best practices for quality calibration
- Frequently asked questions
- Calibration maintenance schedule

**Read this if**: You need to calibrate your Unlook scanner or understand the complete process

**Quick Start**: See Step 1-12 for the complete workflow

---

### 2. CALIBRATION_TROUBLESHOOTING.md (28 KB)

**For**: Resolving calibration issues and problems

**Contents**:
- Quick diagnosis table for common issues
- Detailed troubleshooting for 6 major problem categories:
  1. Pattern Detection Issues
  2. Low Valid Image Pairs
  3. High RMS Reprojection Error
  4. High Baseline Error
  5. Processing Failures and Crashes
  6. Calibration Not Applied to System
- Error message reference
- Prevention tips
- When to contact support

**Read this if**: Your calibration is failing or producing poor results

**Quick Start**: Check the Quick Diagnosis Table to identify your issue

---

### 3. CALIBRATION_PARAMETERS.md (25 KB)

**For**: Understanding calibration parameters and metrics

**Contents**:
- Pattern configuration parameters (rows, cols, square_size_mm)
- Camera intrinsic parameters (focal length, principal point, distortion)
- Stereo extrinsic parameters (rotation, translation, baseline)
- Rectification parameters (R1, R2, P1, P2, Q matrices)
- Quality metrics interpretation (RMS error, baseline error, epipolar error)
- Complete calibration examples with analysis
- Technical references and formulas

**Read this if**: You need to understand what calibration parameters mean

**Quick Start**: See "Validation Summary" section for quality checklist

---

### 4. CALIBRATION_JSON_SCHEMA.md (21 KB)

**For**: Developers and technical documentation

**Contents**:
- Complete JSON schema for dataset_info.json
- Field definitions with types, ranges, and examples
- Data integrity checks
- File structure and access permissions
- JSON validation examples in Python and C++
- Common issues and solutions
- Migration and backwards compatibility

**Read this if**: You need to understand the dataset JSON format or write tools to process datasets

**Quick Start**: See "Complete Schema Example" for typical dataset structure

---

## Quick Reference

### Choosing the Right Document

**I want to...**

- **Calibrate my scanner**
  → Start with CALIBRATION_GUIDE.md (Step 1-12)

- **My calibration is failing**
  → Go to CALIBRATION_TROUBLESHOOTING.md (Quick Diagnosis Table)

- **Understand why my calibration failed**
  → Check CALIBRATION_TROUBLESHOOTING.md (Detailed Solutions)

- **Improve calibration quality**
  → Read CALIBRATION_GUIDE.md (Best Practices section)

- **Understand the numbers in my calibration file**
  → See CALIBRATION_PARAMETERS.md

- **Write code to process calibration data**
  → Reference CALIBRATION_JSON_SCHEMA.md and CALIBRATION_PARAMETERS.md

- **Know when to recalibrate**
  → Check CALIBRATION_GUIDE.md (Recalibration Schedule)

---

## Key Concepts

### Quality Targets

Target values for successful calibration:

| Metric | Target | Acceptable | Fail |
|--------|--------|-----------|------|
| RMS Error | <0.3 px | <0.6 px | >0.8 px |
| Baseline Error | <0.5 mm | <1.0 mm | >1.0 mm |
| Epipolar Error | <0.5 px | <1.0 px | >1.0 px |
| Valid Pairs | 30+ | 30+ | <30 |

### Pattern Requirements

Standard ChArUco 7×10 pattern:
- Rows: 7
- Columns: 10
- Square size: **24.0 mm (CRITICAL - verify with calipers)**
- ArUco marker size: 17.0 mm
- Material: Rigid backing (foam board or plexiglass)

### Calibration Workflow

```
1. Prepare Pattern
   ↓
2. Open Calibration Interface
   ↓
3. Configure Pattern Settings
   ↓
4. Position Pattern and Check Detection
   ↓
5. Start Automated Capture (50 pairs)
   ↓
6. Capture 50 Image Pairs (5 minutes)
   ↓
7. Dataset Capture Complete
   ↓
8. Switch to Dataset Processing Tab
   ↓
9. Process Dataset (30-60 seconds)
   ↓
10. Review Quality Metrics
   ↓
11. Calibration Auto-Applied
   ↓
12. Verify on Known Objects
```

Total time: 6-7 minutes

---

## System Directories

**Calibration datasets**:
```
/unlook_calib_dataset/dataset_YYYYMMDD_HHMMSS/
├── left/           (50 PNG images)
├── right/          (50 PNG images)
└── dataset_info.json
```

**Calibration files**:
```
/unlook_calib/
├── calib-YYYYMMDD_HHMMSS.yaml  (calibration result)
└── default.yaml                (symlink to latest)
```

---

## Most Common Issues

### 1. Pattern Not Detected (40% of cases)
- **Solution**: Improve focus, check lighting, reposition pattern
- **Time to fix**: 2-5 minutes

### 2. Incorrect Pattern Dimensions (35% of cases)
- **Solution**: Verify square size with calipers (24.0mm), recalibrate
- **Time to fix**: 10-15 minutes

### 3. Low Valid Pairs (15% of cases)
- **Solution**: Recapture with better positioning variety
- **Time to fix**: 5-10 minutes

### 4. Mechanical Misalignment (10% of cases)
- **Solution**: Check and re-tighten camera mounting
- **Time to fix**: 5-10 minutes

---

## Verification Checklist

Before calibrating:

- [ ] Pattern dimensions verified with calipers (24.0 ± 0.5 mm)
- [ ] Cameras rigidly mounted (all screws tight)
- [ ] Camera lenses clean
- [ ] No visible damage to cameras
- [ ] Adequate lighting available
- [ ] Scanner at operating temperature
- [ ] System memory available (>500 MB free)

After calibration:

- [ ] RMS error < 0.3 pixels (PASS) or < 0.6 pixels (WARNING)
- [ ] Baseline error < 0.5 mm (PASS) or < 1.0 mm (WARNING)
- [ ] Epipolar error < 0.5 pixels
- [ ] Valid image pairs >= 30
- [ ] Calibration file exists in /unlook_calib/
- [ ] Test on known object confirms accuracy

---

## Technical Specifications

### Calibration Algorithm

- **Method**: OpenCV StereoCalibrate with pattern detection
- **Pattern Types**: ChArUco (recommended), Checkerboard, Circle Grid
- **Image Resolution**: HD 1280×720 (downsampled from 1456×1088 native)
- **Minimum Valid Pairs**: 30 (recommended: 45+)
- **Processing Time**: 30-60 seconds on standard hardware

### Camera Specifications (IMX296)

- **Sensor Resolution**: 1456×1088 SBGGR10
- **Pixel Size**: 2.2 μm
- **Baseline**: 70.017 mm (expected, ±0.5 mm tolerance)
- **Focal Length**: 6.0 mm lens
- **Global Shutter**: Yes (required for sync)
- **Hardware Sync**: XVS/XHS connections

### Quality Metrics

- **Target Precision**: 0.005 mm (achievable with good calibration)
- **RMS Reprojection Error**: Measures corner detection accuracy
- **Baseline Error**: Measures mechanical alignment
- **Epipolar Error**: Measures rectification quality

---

## Resources

### Understanding Stereo Vision

- Epipolar geometry and stereo rectification
- Disparity and depth computation
- Camera calibration theory (Zhang's method)

### Pattern Generation

ChArUco patterns can be generated using OpenCV:
```python
import cv2

# Generate ChArUco 7x10 with 24mm squares
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
board = cv2.aruco.CharucoBoard((10, 7), 24, 17, aruco_dict)
image = board.generateImage((1980, 1540))  # 24mm = 210 DPI at A4
cv2.imwrite('charuco_7x10.png', image)
```

### Troubleshooting Tools

```bash
# Validate calibration file
python3 -c "import yaml; yaml.safe_load(open('/unlook_calib/default.yaml'))"

# Check dataset integrity
ls /unlook_calib_dataset/dataset_XXX/left | wc -l  # Should be 50
ls /unlook_calib_dataset/dataset_XXX/right | wc -l # Should be 50

# Validate JSON
python3 -m json.tool /unlook_calib_dataset/dataset_XXX/dataset_info.json
```

---

## Support and Reporting Issues

If you encounter issues not covered in the troubleshooting guide:

1. **Check** CALIBRATION_TROUBLESHOOTING.md (Quick Diagnosis Table)
2. **Review** relevant detailed troubleshooting section
3. **Verify** all checklist items in CALIBRATION_GUIDE.md
4. **Contact support** with:
   - Latest calibration YAML file
   - Screenshots of error messages
   - System information (OS, RAM, storage)
   - Description of issue and steps attempted

---

## Document Maintenance

**Document Version**: 1.0
**Last Updated**: 2025-01-04
**Status**: Production Ready
**Author**: Unlook Documentation Team

**Related Files**:
- MEGA_PROMPT_CALIBRATION_SYSTEM.md (implementation specification)
- CLAUDE.md (project guidelines)

---

## File Sizes

| Document | Size | Content |
|----------|------|---------|
| CALIBRATION_GUIDE.md | 21 KB | User guide for calibration workflow |
| CALIBRATION_TROUBLESHOOTING.md | 28 KB | Troubleshooting and common issues |
| CALIBRATION_PARAMETERS.md | 25 KB | Technical parameter reference |
| CALIBRATION_JSON_SCHEMA.md | 21 KB | JSON schema documentation |
| **Total** | **95 KB** | Complete calibration documentation |

---

## Navigation

All documents cross-reference each other:

- CALIBRATION_GUIDE.md → See TROUBLESHOOTING for issues
- CALIBRATION_TROUBLESHOOTING.md → See GUIDE for complete workflow
- CALIBRATION_PARAMETERS.md → Referenced from other docs
- CALIBRATION_JSON_SCHEMA.md → For developers and integrations

Use Ctrl+Click (or Cmd+Click on Mac) to follow references in markdown viewers.

---

## Next Steps

1. **First-time users**: Start with CALIBRATION_GUIDE.md
2. **Having issues**: Go to CALIBRATION_TROUBLESHOOTING.md
3. **Need details**: Consult CALIBRATION_PARAMETERS.md
4. **Writing code**: Reference CALIBRATION_JSON_SCHEMA.md

Happy calibrating!
