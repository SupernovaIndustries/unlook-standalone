# Gesture Recognition System - Technical Analysis

## Executive Summary

This document presents a comprehensive analysis of hand gesture recognition solutions for integration into the Unlook 3D scanner system for window/door control applications.

**Target Hardware**: Raspberry Pi CM5 (16GB RAM)
**Requirements**: Real-time gesture recognition, C++ implementation, headless operation, integration with existing stereo camera system
**Use Case**: Window/door control via hand gestures after FaceID authentication

---

## Repository Analysis

### 1. jesture-ai/jesture-sdk

**Repository**: https://github.com/jesture-ai/jesture-sdk

**Technology Stack**:
- Python-based (96.1%)
- MediaPipe framework for hand tracking
- Binary library (.dylib) for performance

**Pros**:
- Real-time on-device processing
- Recognizes both dynamic and static gestures
- Apache-2.0 license

**Cons**:
- ❌ macOS only (not yet available for Linux/Windows)
- ❌ No explicit C++ API support
- ❌ No Raspberry Pi support documentation

**Verdict**: ❌ NOT SUITABLE - Platform limitations prevent use on Raspberry Pi CM5

---

### 2. guillaumephd/deep_learning_hand_gesture_recognition

**Repository**: https://github.com/guillaumephd/deep_learning_hand_gesture_recognition

**Technology Stack**:
- Keras/PyTorch implementations
- Skeletal data (22 joints × 3 channels)
- Deep learning on temporal evolution of hand joints

**Pros**:
- Research-backed (IEEE FG 2018)
- High accuracy on skeletal data
- Google Colab notebooks for training

**Cons**:
- ❌ Requires skeletal data preprocessing (not raw camera input)
- ❌ Research-oriented (not production-ready)
- ❌ Python-based with no C++ migration path
- ❌ Requires significant adaptation for embedded deployment

**Verdict**: ❌ NOT SUITABLE - Too complex for embedded integration, requires extensive preprocessing

---

### 3. ahmetgunduz/Real-time-GesRec

**Repository**: https://github.com/ahmetgunduz/Real-time-GesRec
**Project Page**: https://ahmetgunduz.github.io/Real-time-GesRec/

**Technology Stack**:
- PyTorch-based
- Two-stage architecture: Detector + Classifier
- ResNet-10, ResNetL-10, ResNeXt-101, C3D v1 models
- Tested on EgoGesture, NvGesture, Jester, Kinetics, UCF101

**Performance**:
- 94.04% accuracy on EgoGesture
- 83.82% accuracy on NVIDIA benchmarks
- Published in IEEE Transactions on Biometrics (2020)

**Pros**:
- Excellent accuracy
- Well-tested on multiple datasets
- Strong community (669 stars, 174 forks)
- Proven two-stage architecture

**Cons**:
- ⚠️ PyTorch-based (requires porting to C++)
- ⚠️ Complex model architecture (ResNeXt-101)
- ⚠️ Trained on specific datasets (may require retraining)
- ❌ No direct Raspberry Pi/embedded deployment documentation

**Verdict**: ⚠️ POSSIBLE BUT CHALLENGING - Excellent architecture but requires significant porting effort

---

### 4. MediaPipe (Google)

**Source**: Google AI Edge MediaPipe
**Documentation**: https://ai.google.dev/edge/mediapipe/

**Technology Stack**:
- C++ and Python APIs
- Cross-platform (Linux, macOS, Windows, Android, iOS)
- Hand Tracking and Gesture Recognition solutions
- Bazel-based build system

**Raspberry Pi Support**:
- ✅ Python API well-documented and tested on RPi 3/4/5
- ✅ Pre-built wheels available for ARM64 (PINTO0309/mediapipe-bin)
- ⚠️ C++ API less documented for ARM64
- ⚠️ Cross-compilation challenges with Bazel

**Pros**:
- ✅ Real-time hand landmark detection (21 landmarks)
- ✅ Gesture recognition built-in
- ✅ Excellent performance on embedded devices
- ✅ Apache-2.0 license
- ✅ Industrial-grade quality from Google

**Cons**:
- ⚠️ Bazel build system complexity
- ⚠️ C++ API documentation limited for ARM64
- ⚠️ Requires compilation from source for custom C++ integration

**Verdict**: ✅ HIGHLY SUITABLE - Best balance of features and compatibility, with Python wrapper option

---

### 5. ONNX Runtime + Pre-trained Models

**Key Repository**: https://github.com/PINTO0309/hand-gesture-recognition-using-onnx

**Technology Stack**:
- ONNX Runtime (C++ and Python APIs)
- Pre-trained ONNX models for hand detection and gesture classification
- Multi-Layer Perceptron (MLP) architecture
- OpenCV for preprocessing

**Architecture**:
1. **Hand Detection**: Detects hands in frame
2. **Palm Landmark Detection**: Extracts 21 hand landmarks
3. **Keypoint Classifier**: Recognizes hand signs (palm, fist, pointing, etc.)
4. **Point History Classifier**: Tracks finger gesture trajectories

**ARM64 Support**:
- ✅ ONNX Runtime officially supports ARM64
- ✅ C++ API is mature and well-documented
- ✅ Lightweight models suitable for embedded systems
- ✅ Pre-trained models available

**Performance**:
- Real-time processing on webcam
- Lightweight MLP models (fast inference)
- Configurable detection confidence thresholds

**Pros**:
- ✅ Complete pipeline with pre-trained models
- ✅ Native C++ API support
- ✅ ARM64 optimized builds available
- ✅ Modular architecture (easy to customize)
- ✅ Active development and community support

**Cons**:
- ⚠️ Python-based implementation (requires C++ porting)
- ⚠️ Models may need retraining for specific gestures
- ⚠️ No explicit Raspberry Pi CM5 benchmarks

**Verdict**: ✅ HIGHLY SUITABLE - Best option for C++ integration with proven performance

---

## Recommended Solution: Hybrid MediaPipe + ONNX Runtime

### Architecture Overview

```
┌──────────────────────────────────────────────────────────────┐
│                    Unlook Gesture System                      │
└──────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────┐
│           Existing Unlook Camera System (C++)                │
│  - Hardware-synchronized stereo cameras (IMX296)             │
│  - Calibration: calib_boofcv_test3.yaml                      │
│  - Real-time capture pipeline                                │
└──────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────┐
│              Hand Detection & Tracking Layer                  │
│                                                               │
│  Option A: MediaPipe Python (easier integration)             │
│    - Python wrapper using MediaPipe Hands                    │
│    - C++ callable via subprocess/shared memory               │
│                                                               │
│  Option B: MediaPipe C++ (better performance)                │
│    - Direct C++ integration                                  │
│    - Build from source with ARM64 toolchain                  │
└──────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────┐
│           Gesture Classification Layer (C++)                 │
│                                                               │
│  - ONNX Runtime C++ API                                      │
│  - Pre-trained models from PINTO0309 repository              │
│  - Custom MLP classifier for window/door gestures            │
│  - Real-time inference on CM5                                │
└──────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────┐
│              Gesture Recognition API (C++)                   │
│                                                               │
│  - Headless library (production)                             │
│  - GUI demo mode (client presentation)                       │
│  - Event callbacks for gesture detection                     │
│  - Integration with window/door control API                  │
└──────────────────────────────────────────────────────────────┘
```

### Component Selection Rationale

#### 1. Hand Detection: MediaPipe

**Why MediaPipe?**
- Proven real-time performance on Raspberry Pi
- 21-landmark hand tracking (sufficient for gesture recognition)
- Python API is stable and well-documented for RPi
- Can be wrapped in C++ via IPC or shared memory
- Apache-2.0 license (commercial use allowed)

**Implementation Strategy**:
- **Phase 1**: Python MediaPipe wrapper (fastest time to market)
  - C++ process launches Python subprocess
  - Communication via shared memory (zero-copy for performance)
  - Python process runs MediaPipe Hands solution
  - Returns landmark coordinates to C++ main process

- **Phase 2** (optional): Native C++ MediaPipe integration
  - Build MediaPipe from source for ARM64
  - Direct C++ API integration
  - Maximum performance (eliminates Python overhead)

#### 2. Gesture Classification: ONNX Runtime C++

**Why ONNX Runtime?**
- Native C++ API (no Python dependency)
- Excellent ARM64 support with optimized builds
- Pre-trained models available (PINTO0309 repository)
- Flexible: can use custom trained models
- Microsoft-backed with industrial-grade support

**Implementation Strategy**:
- Use ONNX Runtime C++ API for inference
- Load pre-trained gesture classification models (ONNX format)
- Extract hand landmarks from MediaPipe
- Feed landmarks to ONNX classifier
- Return gesture classification result

**Supported Gestures** (from pre-trained models):
- Open palm (window open)
- Closed fist (window close)
- Pointing (directional control)
- Swipe left/right (open/close action)
- Custom gestures (via retraining)

### Integration with Existing Unlook System

#### Camera System Integration

The existing Unlook camera system already provides:
- ✅ Hardware-synchronized stereo cameras
- ✅ Calibrated stereo setup (70.017mm baseline)
- ✅ Real-time capture pipeline
- ✅ YUV420/RGB conversion
- ✅ Thread-safe camera access

**Integration Approach**:
```cpp
// Existing Unlook camera system
namespace unlook {
namespace camera {
    class SynchronizedCameraSystem {
        // Already implemented
        bool captureSynchronizedFrames(cv::Mat& left, cv::Mat& right);
    };
}

// New gesture recognition system
namespace unlook {
namespace gesture {
    class GestureRecognitionSystem {
    public:
        bool initialize(std::shared_ptr<camera::SynchronizedCameraSystem> camera);
        GestureResult detectGesture(const cv::Mat& frame);
    };
}
}
```

#### Workflow Integration

```
1. System Standby
   ↓
2. FaceID Recognition (existing Unlook system)
   ↓
3. User Authenticated → Activate Gesture Recognition
   ↓
4. Gesture Detection Loop:
   - Capture frame from left camera (or both for depth)
   - Run MediaPipe hand detection
   - Extract hand landmarks
   - Run ONNX gesture classification
   - Trigger window/door action
   ↓
5. User leaves → Return to Standby
```

---

## Implementation Roadmap

### Phase 1: Research & Prototyping (Week 1-2)
- [x] Research gesture recognition solutions
- [x] Evaluate compatibility with Raspberry Pi CM5
- [x] Select MediaPipe + ONNX Runtime architecture
- [ ] Set up development environment in isolated folder
- [ ] Clone and test PINTO0309 ONNX models
- [ ] Test MediaPipe Python on Raspberry Pi CM5

### Phase 2: Core Implementation (Week 3-4)
- [ ] Implement MediaPipe Python wrapper
- [ ] Integrate ONNX Runtime C++ inference
- [ ] Create C++/Python IPC bridge (shared memory)
- [ ] Integrate with existing Unlook camera system
- [ ] Implement gesture classification pipeline

### Phase 3: Headless Library (Week 5-6)
- [ ] Design C++ API for headless operation
- [ ] Implement event-driven gesture callbacks
- [ ] Create configuration system for gesture types
- [ ] Add logging and error handling
- [ ] Performance optimization for real-time operation

### Phase 4: GUI Demo (Week 7)
- [ ] Implement Qt5-based demo GUI
- [ ] Real-time gesture visualization
- [ ] Debug mode with hand landmark overlay
- [ ] Gesture recognition statistics display
- [ ] Client presentation mode

### Phase 5: Testing & Optimization (Week 8-9)
- [ ] Performance benchmarking on CM5
- [ ] Latency optimization (<100ms target)
- [ ] Memory usage optimization
- [ ] Integration testing with FaceID system
- [ ] Client demo preparation

### Phase 6: Documentation & Deployment (Week 10)
- [ ] Complete API documentation
- [ ] User manual for demo GUI
- [ ] Integration guide for client API
- [ ] Deployment package creation
- [ ] Client presentation materials

---

## Technical Specifications

### Performance Targets

| Metric | Target | Raspberry Pi CM5 Expected |
|--------|--------|---------------------------|
| Gesture detection latency | <100ms | 50-80ms |
| Frame rate | ≥20 FPS | 25-30 FPS |
| Hand detection accuracy | >90% | 92-95% (MediaPipe) |
| Gesture classification accuracy | >85% | 88-92% (ONNX MLP) |
| Memory usage | <500MB | 300-400MB |
| CPU usage | <50% | 35-45% |

### Gesture Set (Initial Implementation)

| Gesture | Action | Description |
|---------|--------|-------------|
| Open Palm | Open Window | Hand fully open, palm facing camera |
| Closed Fist | Close Window | Hand fully closed |
| Swipe Right | Open Action | Hand moves right to left |
| Swipe Left | Close Action | Hand moves left to right |
| Point Up | Stop/Pause | Index finger pointing up |
| Point Down | Resume | Index finger pointing down |

### Dependencies

#### Core Libraries
- **ONNX Runtime** (v1.17.0+): C++ inference engine
- **MediaPipe** (v0.10.0+): Hand detection and tracking
- **OpenCV** (v4.5.0+): Already present in Unlook system
- **Qt5** (v5.15.0+): GUI demo (already present)

#### Optional Libraries
- **Protocol Buffers**: For MediaPipe communication
- **Abseil**: MediaPipe dependency

### Build System Updates

```cmake
# CMakeLists.txt additions for gesture recognition
option(BUILD_GESTURE_RECOGNITION "Build gesture recognition system" ON)

if(BUILD_GESTURE_RECOGNITION)
    find_package(ONNXRuntime REQUIRED)
    find_package(MediaPipe)  # Optional, use Python wrapper if not found

    add_executable(unlook_gesture
        src/gesture/main.cpp
        src/gesture/GestureRecognitionSystem.cpp
        src/gesture/ONNXClassifier.cpp
        src/gesture/MediaPipeWrapper.cpp
    )

    target_link_libraries(unlook_gesture
        unlook_camera  # Existing camera system
        ONNXRuntime::ONNXRuntime
        ${OpenCV_LIBS}
        Qt5::Widgets
    )
endif()
```

---

## Risk Analysis

### Technical Risks

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| MediaPipe C++ build complexity | High | Medium | Use Python wrapper initially, C++ later |
| ONNX model accuracy insufficient | Low | High | Retrain models with custom dataset |
| Real-time performance on CM5 | Low | High | Optimize pipeline, use NEON instructions |
| Integration with existing camera system | Low | Medium | Modular design with clear interfaces |
| Python/C++ IPC overhead | Medium | Medium | Use shared memory for zero-copy |

### Mitigation Strategies

1. **MediaPipe Build Complexity**
   - Use pre-built Python wheels for RPi ARM64
   - Python wrapper approach for Phase 1
   - Defer C++ build to Phase 2 (optional)

2. **Model Accuracy**
   - Use pre-trained models as baseline
   - Collect custom dataset if needed
   - Retrain using ONNX-compatible frameworks (PyTorch → ONNX)

3. **Performance Optimization**
   - Profile critical path with gprof/perf
   - Optimize with ARM NEON instructions
   - Multi-threading for parallel processing
   - Consider using only one camera (left) instead of stereo

4. **Integration Challenges**
   - Design clear API boundaries
   - Use dependency injection for testing
   - Mock camera system for development

---

## Alternative Approaches Considered

### Alternative 1: Full Python Implementation

**Description**: Implement entire system in Python using MediaPipe + ONNX Runtime Python API

**Pros**:
- Fastest development time
- All libraries have mature Python APIs
- Easier prototyping and testing

**Cons**:
- ❌ Violates project requirement (C++ only)
- ❌ Higher memory footprint
- ❌ Slower performance than C++
- ❌ Python runtime dependency

**Verdict**: ❌ REJECTED - Does not meet C++ requirement

### Alternative 2: TensorFlow Lite C++

**Description**: Use TensorFlow Lite C++ API instead of ONNX Runtime

**Pros**:
- Native C++ API
- ARM64 optimizations
- Google-backed support

**Cons**:
- More complex than ONNX Runtime C++ API
- Requires TFLite model conversion
- Larger binary size

**Verdict**: ⚠️ SECONDARY OPTION - ONNX Runtime is simpler and sufficient

### Alternative 3: OpenCV DNN Module

**Description**: Use OpenCV's DNN module for inference

**Pros**:
- Already integrated in project
- C++ native
- No additional dependencies

**Cons**:
- Limited gesture recognition model support
- Slower than specialized inference engines
- No pre-trained gesture models

**Verdict**: ❌ REJECTED - Insufficient features for gesture recognition

---

## Conclusion

The recommended solution combines **MediaPipe for hand detection** and **ONNX Runtime for gesture classification**, providing:

✅ **Real-time performance** on Raspberry Pi CM5
✅ **C++ integration** with Python wrapper for Phase 1
✅ **Industrial-grade quality** from proven libraries
✅ **Modular architecture** for easy integration
✅ **Pre-trained models** for rapid deployment
✅ **Scalability** for custom gesture training

This approach balances **development speed**, **performance**, and **maintainability**, while meeting all project requirements for the window/door control gesture recognition system.

---

## References

1. MediaPipe Hands: https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker
2. ONNX Runtime: https://onnxruntime.ai/
3. PINTO0309 ONNX Hand Gesture: https://github.com/PINTO0309/hand-gesture-recognition-using-onnx
4. Real-time-GesRec Paper: G. Kopuklu et al., "Real-time Hand Gesture Detection and Classification Using Convolutional Neural Networks," IEEE Transactions on Biometrics, Behavior, and Identity Science, 2020
5. MediaPipe ARM64 Wheels: https://github.com/PINTO0309/mediapipe-bin

---

**Document Version**: 1.0
**Date**: 2025-10-13
**Author**: Unlook Gesture Recognition Team
**Status**: Technical Analysis Complete - Ready for Implementation
