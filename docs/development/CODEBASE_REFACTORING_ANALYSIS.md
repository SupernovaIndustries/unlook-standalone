# Codebase Refactoring Analysis

## Executive Summary

This document presents a comprehensive analysis of the Unlook codebase to identify refactoring opportunities, code redundancies, naming inconsistencies, and structural improvements. The goal is to prepare the codebase for multi-team collaboration and gesture recognition system integration.

**Codebase Metrics**:
- Total source files: 86 (C++/H/HPP)
- Total documentation files: 37 (MD)
- Main directories: 13 (api, calibration, camera, core, face, gui, hardware, mesh, pointcloud, realtime, stereo, utils, validation)

---

## Critical Issues Identified

### 1. Documentation Organization (CRITICAL - HIGH PRIORITY)

#### Problem
**37 Markdown documentation files** scattered in the root directory, creating severe documentation clutter:

```
./AMBIENT_SGBM_FIXES.md
./AMBIENT_SUBTRACTION_FIX.md
./AMBIENT_SUBTRACTION_IMPLEMENTATION.md
./API_IMPLEMENTATION_SUMMARY.md
./AS1170_CM5_UPDATE_SUMMARY.md
./AS1170_DEBUG_SYSTEM_SUMMARY.md
./AS1170_DUAL_VCSEL_IMPLEMENTATION_SUMMARY.md
./AUTO_EXPOSURE_OPTIMIZATION.md
./BUILD_SYSTEM_OVERVIEW.md
./CAMERA_SYNC_SOLUTION.md
./CAMERA_SYSTEM.md
./CLAUDE.md
./COMMIT_STATE_SUMMARY.md
./DIRECT_DISPARITY_FIX.md
./DUAL_AS1170_FLOOD_CONFIGURATION.md
./DUAL_VCSEL_QUICK_REFERENCE.md
./DUAL_VCSEL_TEMPORAL_MATCHING.md
./FRAME_AVERAGING_IMPLEMENTATION.md
./GESTURE_RECOGNITION_ANALYSIS.md
./GUI_CONVERSION_SUMMARY.md
./HARDWARE_SETUP.md
./HIGH_QUALITY_SGBM_PARAMS.md
./INSTALL_DEPENDENCIES.md
./ML_DEPTH_REFINEMENT_RESEARCH.md
./NEXT_SESSION_PROMPT.md
./PLY_EXPORT_IMPLEMENTATION_SUMMARY.md
./PROJECT_GUIDELINES.md
./QT_DESIGN_STUDIO_SETUP.md
./README_API.md
./README.md
./SBGGR10_FIX_SUMMARY.md
./STEREO_OPTIMIZATION_REPORT.md
./SYNC_FIX_SUMMARY.md
./TEMPORAL_STEREO_IMPLEMENTATION_COMPLETE.md
./TROUBLESHOOTING.md
./VCSEL_INTEGRATION_SUMMARY.md
./VCSEL_PARAMS_IMPLEMENTATION.md
```

#### Impact
- ❌ **Difficult navigation**: Hard to find relevant documentation
- ❌ **Maintenance burden**: Unclear which docs are current vs obsolete
- ❌ **Poor developer experience**: New team members overwhelmed
- ❌ **Redundancy**: Multiple files cover overlapping topics (VCSEL, stereo, AS1170)

#### Solution

**Create organized documentation structure:**

```
docs/
├── README.md                    # Documentation index
├── architecture/
│   ├── system-overview.md
│   ├── camera-system.md
│   └── stereo-processing.md
├── hardware/
│   ├── setup.md
│   ├── vcsel-system.md
│   ├── as1170-controller.md
│   └── dual-vcsel-configuration.md
├── implementation/
│   ├── api-guide.md
│   ├── stereo-optimization.md
│   ├── temporal-processing.md
│   ├── ambient-subtraction.md
│   └── frame-averaging.md
├── troubleshooting/
│   ├── build-issues.md
│   ├── sync-problems.md
│   └── sbggr10-format.md
├── development/
│   ├── PROJECT_GUIDELINES.md
│   ├── CLAUDE.md
│   └── build-system.md
└── history/                      # Archive for historical summaries
    ├── commit-summaries/
    └── implementation-notes/
```

**Actions**:
1. Create `docs/` directory structure
2. Categorize and move existing MD files
3. Merge redundant documents:
   - **VCSEL files**: Merge `VCSEL_INTEGRATION_SUMMARY.md`, `VCSEL_PARAMS_IMPLEMENTATION.md`, `DUAL_VCSEL_QUICK_REFERENCE.md`, `DUAL_VCSEL_TEMPORAL_MATCHING.md` → `docs/hardware/vcsel-system.md`
   - **AS1170 files**: Merge `AS1170_CM5_UPDATE_SUMMARY.md`, `AS1170_DEBUG_SYSTEM_SUMMARY.md`, `DUAL_AS1170_FLOOD_CONFIGURATION.md` → `docs/hardware/as1170-controller.md`
   - **Stereo files**: Merge `STEREO_OPTIMIZATION_REPORT.md`, `HIGH_QUALITY_SGBM_PARAMS.md` → `docs/implementation/stereo-optimization.md`
   - **Ambient/Frame**: Merge `AMBIENT_SGBM_FIXES.md`, `AMBIENT_SUBTRACTION_FIX.md`, `AMBIENT_SUBTRACTION_IMPLEMENTATION.md`, `FRAME_AVERAGING_IMPLEMENTATION.md` → `docs/implementation/ambient-subtraction.md`
4. Keep only **essential files** in root:
   - `README.md` (project overview)
   - `CLAUDE.md` (Claude Code guidance)
   - `PROJECT_GUIDELINES.md` (development standards)
5. Add deprecation notices to moved files
6. Create `docs/README.md` index with links to all documentation

---

### 2. Code Duplication - TemporalStereoProcessor (HIGH PRIORITY)

#### Problem
**Two versions of TemporalStereoProcessor** exist:

```
src/stereo/TemporalStereoProcessor.cpp
src/stereo/TemporalStereoProcessor_fixed.cpp
```

#### Analysis
- `_fixed` suffix indicates a bug fix or refactored version
- Unclear which version is current/active
- Both files maintained = double maintenance burden
- Risk of using wrong version accidentally

#### Solution

**Option A**: Remove old version entirely (RECOMMENDED)
```bash
# If _fixed version is stable and tested
git rm src/stereo/TemporalStereoProcessor.cpp
git mv src/stereo/TemporalStereoProcessor_fixed.cpp src/stereo/TemporalStereoProcessor.cpp
```

**Option B**: Add clear deprecation warning
```cpp
// TemporalStereoProcessor.cpp
#warning "DEPRECATED: Use TemporalStereoProcessor_fixed.cpp instead. This file will be removed in next release."
```

**Option C**: Archive old version
```bash
mkdir -p archive/deprecated/
git mv src/stereo/TemporalStereoProcessor.cpp archive/deprecated/
```

**Actions**:
1. Verify `_fixed` version is stable and tested
2. Check CMakeLists.txt to confirm which version is compiled
3. Run tests with `_fixed` version
4. Remove old version after verification
5. Update include guards and header references
6. Document changes in commit message

---

### 3. Class Naming Inconsistency - Processor Classes (MEDIUM PRIORITY)

#### Problem
**Inconsistent naming conventions** for processor classes:

```cpp
// api/ namespace
src/api/depth_processor.cpp           // lowercase with underscore

// stereo/ namespace
src/stereo/DepthProcessor.cpp         // PascalCase (correct C++ convention)

// pointcloud/ namespace
src/pointcloud/PointCloudProcessor.cpp  // PascalCase (correct)

// face/ namespace
src/face/FacialPointCloudProcessor.cpp  // PascalCase (correct)
```

#### Impact
- ❌ **Confusing for developers**: Mixed naming conventions
- ❌ **Violates C++ best practices**: Class filenames should match class names
- ❌ **Risk of errors**: Easy to confuse `depth_processor` vs `DepthProcessor`

#### Analysis
Looking at the pattern:
- **`src/api/depth_processor.cpp`**: Likely a C-style API wrapper (lowercase)
- **`src/stereo/DepthProcessor.cpp`**: Main C++ class (PascalCase)

**Hypothesis**: `api/depth_processor.cpp` provides C++ API facade, while `stereo/DepthProcessor.cpp` implements actual processing logic.

#### Solution

**Option A**: Rename API file to match class name (RECOMMENDED)
```bash
# If class is named DepthProcessorAPI or similar
git mv src/api/depth_processor.cpp src/api/DepthProcessorAPI.cpp
```

**Option B**: Keep distinction if intentional
```cpp
// api/depth_processor.cpp - C API wrapper
namespace unlook {
namespace api {
    // C-style API functions
    extern "C" {
        depth_processor_t* depth_processor_create();
        void depth_processor_process(depth_processor_t* dp, ...);
    }
}
}

// stereo/DepthProcessor.cpp - C++ implementation
namespace unlook {
namespace stereo {
    class DepthProcessor {
        // C++ implementation
    };
}
}
```

**Actions**:
1. Read both files to understand their roles
2. Check if `api/depth_processor.cpp` is C API or C++ API
3. If C++ API: Rename to match class name (PascalCase)
4. If C API: Add clear documentation explaining the distinction
5. Ensure consistent pattern across all API files

---

### 4. Directory Structure - Gesture Recognition Integration (HIGH PRIORITY)

#### Problem
**No dedicated directory** for gesture recognition system, which will need:
- MediaPipe wrapper
- ONNX classifier
- IPC communication
- Gesture event handling

#### Current Structure
```
src/
├── api/
├── calibration/
├── camera/
├── core/
├── face/
├── gui/
├── hardware/
├── mesh/
├── pointcloud/
├── realtime/
├── stereo/
├── utils/
└── validation/
```

#### Solution

**Add new gesture directory with modular structure:**

```
src/
├── gesture/                    # NEW: Gesture recognition system
│   ├── GestureRecognitionSystem.cpp
│   ├── GestureRecognitionSystem.hpp
│   ├── MediaPipeWrapper.cpp
│   ├── MediaPipeWrapper.hpp
│   ├── ONNXClassifier.cpp
│   ├── ONNXClassifier.hpp
│   ├── IPCBridge.cpp
│   ├── IPCBridge.hpp
│   ├── GestureTypes.hpp
│   └── GestureConfig.hpp
```

**Separation of concerns:**
```cpp
namespace unlook {
namespace gesture {
    // Main system
    class GestureRecognitionSystem;

    // Hand detection (MediaPipe wrapper)
    class MediaPipeWrapper;

    // Gesture classification (ONNX Runtime)
    class ONNXClassifier;

    // C++/Python IPC
    class IPCBridge;

    // Data types
    struct Hand {
        std::array<cv::Point3f, 21> landmarks;
        float confidence;
    };

    struct GestureResult {
        GestureType type;
        float confidence;
        cv::Point2f position;
    };

    enum class GestureType {
        OPEN_PALM,
        CLOSED_FIST,
        SWIPE_LEFT,
        SWIPE_RIGHT,
        POINT_UP,
        POINT_DOWN,
        UNKNOWN
    };
}
}
```

---

### 5. Code Organization - Java Directory in C++ Project (LOW PRIORITY)

#### Problem
**Java directory exists** in C++ project:

```
src/java/
src/java/unlook/
src/java/unlook/stereo/
```

#### Analysis
- Likely used for BoofCV integration (Java-based library)
- JNI wrapper for calling Java from C++
- Not well-integrated with main codebase

#### Solution

**Option A**: Keep but reorganize
```
src/
├── bindings/              # NEW: Language bindings
│   ├── java/
│   │   └── unlook/stereo/
│   └── python/ (future)
```

**Option B**: Move to third-party
```
third-party/
└── boofcv-wrapper/
    └── java/
```

**Actions**:
1. Check if Java code is actively used
2. If used: Move to `src/bindings/java/`
3. If obsolete: Remove or archive
4. Document purpose in README

---

### 6. Naming Conventions - File Extensions (LOW PRIORITY)

#### Problem
**Inconsistent header file extensions**:

```
src/unlook.h           # .h extension
src/stereo/*.hpp       # .hpp extension
include/unlook/*.h     # .h extension
```

#### Analysis
C++ projects typically use one of:
- `.h` for C-style or simple headers
- `.hpp` for C++-only headers
- `.hxx` for template implementations

#### Solution

**Standardize on `.hpp` for C++ headers:**

```bash
# Rename all .h files in src/ to .hpp
find src -name "*.h" -exec sh -c 'git mv "$1" "${1%.h}.hpp"' _ {} \;
```

**Exception**: Keep `.h` for C-compatible headers:
- Public API headers (if C compatibility needed)
- Third-party integrations

**Actions**:
1. Decide on standard (.h vs .hpp)
2. Update PROJECT_GUIDELINES.md with decision
3. Gradually migrate files (low priority)
4. Update CMakeLists.txt patterns

---

### 7. Test Organization (MEDIUM PRIORITY)

#### Problem
**Test files scattered** across repository:

```
tests/                      # Unit tests directory
examples/test_*.cpp         # Test examples mixed with examples
src/gui/ui/*.ui             # Qt UI test files
```

#### Solution

**Centralize test organization:**

```
tests/
├── unit/                   # Unit tests (Google Test)
│   ├── core/
│   ├── camera/
│   ├── stereo/
│   ├── gesture/        # NEW
│   └── ...
├── integration/            # Integration tests
│   ├── camera_sync/
│   ├── stereo_pipeline/
│   └── gesture_system/  # NEW
├── hardware/               # Hardware-specific tests
│   ├── camera_hardware/
│   ├── vcsel_hardware/
│   └── ...
└── performance/            # Performance benchmarks
    ├── stereo_benchmark/
    ├── gesture_benchmark/  # NEW
    └── ...
```

**Actions**:
1. Move `examples/test_*.cpp` to appropriate `tests/` subdirectory
2. Create `tests/gesture/` for new system
3. Update CMakeLists.txt test discovery
4. Add test documentation

---

## Refactoring Priority Matrix

| Priority | Issue | Impact | Effort | Timeline |
|----------|-------|--------|--------|----------|
| 🔴 **CRITICAL** | Documentation organization (37 MD files) | Very High | Medium | Week 1 |
| 🔴 **HIGH** | TemporalStereoProcessor duplication | High | Low | Week 1 |
| 🔴 **HIGH** | Gesture directory structure | High | Low | Week 1 |
| 🟡 **MEDIUM** | Processor class naming | Medium | Low | Week 2 |
| 🟡 **MEDIUM** | Test organization | Medium | Medium | Week 3 |
| 🟢 **LOW** | Java directory location | Low | Low | Week 4 |
| 🟢 **LOW** | Header file extensions | Low | High | Backlog |

---

## Refactoring Action Plan

### Phase 1: Critical Issues (Week 1)

#### 1.1 Documentation Organization
```bash
# Create documentation structure
mkdir -p docs/{architecture,hardware,implementation,troubleshooting,development,history}

# Move and merge files (detailed plan in issue #1)
# Priority: Merge redundant VCSEL/AS1170/Stereo files first

# Update README.md with documentation links
```

#### 1.2 Remove TemporalStereoProcessor Duplication
```bash
# Verify _fixed version is active
grep -r "TemporalStereoProcessor_fixed" CMakeLists.txt

# Test _fixed version
./build_and_test_performance.sh

# Remove old version
git rm src/stereo/TemporalStereoProcessor.cpp

# Rename _fixed to standard name
git mv src/stereo/TemporalStereoProcessor_fixed.cpp \
      src/stereo/TemporalStereoProcessor.cpp
```

#### 1.3 Create Gesture Directory
```bash
# Create gesture directory structure
mkdir -p src/gesture
mkdir -p include/unlook/gesture
mkdir -p tests/gesture

# Create initial files (headers only for now)
touch src/gesture/GestureRecognitionSystem.cpp
touch include/unlook/gesture/GestureRecognitionSystem.hpp
# ... (see section #4 for full file list)
```

### Phase 2: Medium Priority (Week 2-3)

#### 2.1 Fix Processor Class Naming
```bash
# Investigate api/depth_processor.cpp
cat src/api/depth_processor.cpp | grep "class\|namespace"

# If C++ API: Rename to PascalCase
# If C API: Document distinction clearly
```

#### 2.2 Reorganize Tests
```bash
# Create test directory structure
mkdir -p tests/{unit,integration,hardware,performance}

# Move test files from examples/
mv examples/test_*.cpp tests/integration/

# Update CMakeLists.txt
```

### Phase 3: Low Priority (Week 4+)

#### 3.1 Reorganize Java/Bindings
```bash
# Move Java code to bindings
mkdir -p src/bindings
mv src/java src/bindings/java
```

#### 3.2 Header Extensions (Backlog)
- Decide on standard (.h vs .hpp)
- Update PROJECT_GUIDELINES.md
- Gradually migrate (automated script)

---

## Code Quality Improvements

### Naming Conventions (from PROJECT_GUIDELINES.md)

**Current Status**: Mixed adherence
**Target**: Strict adherence to C++ best practices

#### Classes and Structs
- ✅ **Good**: `GestureRecognitionSystem`, `DepthProcessor`, `PointCloudProcessor`
- ❌ **Bad**: `depth_processor` (lowercase file for class)

#### Files
- ✅ **Good**: `TemporalStereoProcessor.cpp` (matches class name)
- ❌ **Bad**: `TemporalStereoProcessor_fixed.cpp` (temporary suffix)

#### Namespaces
- ✅ **Good**: `unlook::gesture`, `unlook::stereo`, `unlook::camera`
- ✅ **Consistent**: All lowercase with `::`

#### Functions
- ✅ **Good**: `captureSynchronizedFrames()`, `detectGesture()`
- ✅ **Consistent**: camelCase for methods

### Documentation Standards

**Current Status**: Inconsistent
**Target**: Doxygen-style comments for all public APIs

```cpp
/**
 * @brief Detects hand gestures in real-time from camera frames
 *
 * This class integrates MediaPipe for hand detection and ONNX Runtime
 * for gesture classification, providing a complete gesture recognition
 * pipeline optimized for Raspberry Pi CM5.
 *
 * @note Requires MediaPipe Python wrapper and ONNX Runtime C++
 * @see MediaPipeWrapper, ONNXClassifier
 *
 * Example usage:
 * @code
 * GestureRecognitionSystem gesture;
 * gesture.initialize(cameraSystem);
 * auto result = gesture.detectGesture(frame);
 * if (result.type == GestureType::OPEN_PALM) {
 *     openWindow();
 * }
 * @endcode
 */
class GestureRecognitionSystem {
    // ...
};
```

---

## Post-Refactoring Checklist

### Code Organization
- [ ] All MD files moved to `docs/` directory
- [ ] Redundant MD files merged
- [ ] `docs/README.md` index created
- [ ] TemporalStereoProcessor duplication removed
- [ ] Gesture directory structure created
- [ ] Processor class naming standardized

### Build System
- [ ] CMakeLists.txt updated for new structure
- [ ] All targets compile successfully
- [ ] Tests run and pass
- [ ] Documentation generation works (Doxygen)

### Documentation
- [ ] All moved files have deprecation notices
- [ ] New structure documented in README.md
- [ ] PROJECT_GUIDELINES.md updated
- [ ] CLAUDE.md updated with new structure

### Testing
- [ ] All unit tests pass
- [ ] Integration tests pass
- [ ] Hardware tests verified (if available)
- [ ] Performance benchmarks maintained

### Git History
- [ ] All moves done with `git mv` (preserves history)
- [ ] Clear commit messages explaining changes
- [ ] Branch `gesture-recognition` updated
- [ ] Ready for review before merge to main

---

## Estimated Impact

### Developer Experience
- **Before**: 😓 Confused by 37 MD files, duplicate classes, inconsistent naming
- **After**: 😊 Clear documentation structure, consistent patterns, easy navigation

### Maintenance
- **Before**: High effort to maintain duplicate files and scattered documentation
- **After**: Single source of truth, clear ownership, easy updates

### New Team Members
- **Before**: Overwhelming, unclear where to start
- **After**: Clear documentation index, logical structure, quick onboarding

### Gesture Integration
- **Before**: Unclear where gesture code should live
- **After**: Dedicated `src/gesture/` with clear boundaries

---

## Conclusion

This refactoring analysis identifies **7 major issues** requiring attention before gesture recognition integration:

1. **Documentation organization** (37 scattered MD files) - CRITICAL
2. **Code duplication** (TemporalStereoProcessor) - HIGH
3. **Naming inconsistency** (Processor classes) - MEDIUM
4. **Missing structure** (Gesture directory) - HIGH
5. **Java code location** (src/java) - LOW
6. **Header extensions** (mixed .h/.hpp) - LOW
7. **Test organization** (scattered tests) - MEDIUM

**Recommended Approach**:
- **Week 1**: Address CRITICAL and HIGH priority items
- **Week 2-3**: Medium priority refactoring
- **Week 4+**: Low priority and continuous improvement

**Expected Outcome**:
- Clean, maintainable codebase ready for multi-team collaboration
- Clear structure for gesture recognition integration
- Improved developer experience and onboarding
- Reduced technical debt

---

**Document Version**: 1.0
**Date**: 2025-10-13
**Author**: Unlook Gesture Recognition Team
**Status**: Analysis Complete - Ready for Refactoring Implementation
