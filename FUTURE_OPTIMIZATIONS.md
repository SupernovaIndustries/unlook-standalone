# Future Performance Optimizations

This document tracks potential optimizations for the Unlook 3D Scanner real-time performance.

## Current Performance Baseline (2025-11-19)

- **Total processing time**: ~10 seconds
  - Capture: ~2-3 seconds (burst mode)
  - Processing: ~7-8 seconds (OpenMP parallelized)
- **Hardware**: Raspberry Pi 5 (4x Cortex-A76 cores @ 2.4 GHz)
- **Resolution**: 680x420 (center crop from 1280x720)
- **Algorithm**: SGM-Census stereo matching with 384 disparities

## Implemented Optimizations (2025-11-19)

### ✅ OpenMP Multi-threading (4 cores)
- Census Transform: row parallelization
- Matching Cost: row parallelization
- SGM 8-path aggregation: path parallelization
- WTA Selection: row parallelization
- Border/Statistical filters: parallelized
- **Speedup achieved**: 2.5-3x

### ✅ BURST CAPTURE MODE
- Continuous capture without stop/start overhead
- Single stabilization (300ms vs 250ms × frames)
- Skip frames once (3 frames vs 6 × frames)
- **Speedup achieved**: ~10x capture time (20-30s → 2-3s)

### ✅ Micro-optimizations
- CLAHE object reuse (created once, not per frame)
- Disparity fusion parallelized
- **Speedup achieved**: ~5-10% additional

---

## Future Optimizations (Priority Order)

### 1. NEON SIMD Intrinsics (ARM64) 🔥 **HIGH PRIORITY**

**Target**: Census Transform and Hamming Distance computation
**Expected speedup**: 2-4x for these specific functions
**Complexity**: Medium

#### Implementation Strategy:

**Census Transform with NEON:**
```cpp
// Current: scalar bitwise operations
// Future: Process 8 pixels simultaneously with NEON
void computeCensusTransformNEON(const cv::Mat& image, cv::Mat& census) {
    // Use uint8x16_t for 16 pixels at once
    // Use vcltq_u8 for comparison (pixel < center)
    // Use vshlq_n_u64 for bit shifting
    // Reduce Census time: ~50ms → ~15ms
}
```

**Hamming Distance with NEON:**
```cpp
// Current: __builtin_popcountll (good, but can be better)
// Future: NEON vectorized popcount
inline int hammingWeightNEON(uint64_t val) {
    // Use vcnt_u8 (count bits) on ARM64
    // Process multiple descriptors in parallel
    // Reduce Hamming cost: ~100ms → ~30ms
}
```

**Files to modify:**
- `src/stereo/SGMCensus.cpp` (add NEON variants)
- `include/unlook/stereo/SGMCensus.hpp` (compile-time detection)
- `CMakeLists.txt` (add `-march=armv8-a+simd` flags)

**References:**
- ARM NEON Intrinsics Guide: https://developer.arm.com/architectures/instruction-sets/intrinsics/
- OpenCV NEON examples: `modules/core/src/arithm_neon.cpp`

---

### 2. Memory Pooling 🔥 **HIGH PRIORITY**

**Target**: Reduce dynamic allocations in hot paths
**Expected speedup**: 10-20% reduction in total time
**Complexity**: Medium

#### Current Bottlenecks:

1. **Cost Volume allocations** (every frame):
   - `std::vector<uint8_t> costVolume(height * width * D)` → 680×420×384 = 110 MB
   - `std::vector<uint32_t> aggregatedCost(height * width * D)` → 440 MB
   - **Total per frame: ~550 MB allocated/freed**

2. **Path costs** (SGM aggregation):
   - 8 × `std::vector<uint16_t>(height * width * D)` → 8 × 220 MB = 1.76 GB

#### Solution: Pre-allocated Memory Pools

```cpp
class SGMCensusMemoryPool {
    std::vector<uint8_t> costVolume_;
    std::vector<uint32_t> aggregatedCost_;
    std::vector<std::vector<uint16_t>> pathCosts_;

    void resize(int width, int height, int D) {
        // Resize ONCE, reuse for all frames
        costVolume_.resize(height * width * D);
        aggregatedCost_.resize(height * width * D);
        for (auto& path : pathCosts_) {
            path.resize(height * width * D);
        }
    }
};
```

**Files to modify:**
- `src/stereo/SGMCensus.cpp` (add memory pool member)
- `src/api/HandheldScanPipeline.cpp` (reuse SGMCensus instance)

---

### 3. Pipeline Overlap ⚡ **MEDIUM PRIORITY**

**Target**: Process frame N while capturing frame N+1
**Expected speedup**: 20-30% reduction in total time
**Complexity**: High (threading complexity)

#### Current Sequential Pipeline:
```
Capture frames [2-3s] → Process all frames [7-8s] → Total: 10s
```

#### Proposed Overlapped Pipeline:
```
Frame 1: Capture [300ms] → Process [700ms]
Frame 2: Capture [300ms] → Process [700ms]  ← Overlapped!
Frame 3: Capture [300ms] → Process [700ms]  ← Overlapped!
...
Total: ~5-7s (instead of 10s)
```

#### Implementation Strategy:

```cpp
// Producer-Consumer pattern with lock-free queue
std::queue<StereoFrame> captureQueue;
std::mutex queueMutex;

// Thread 1: Capture (runs continuously)
void captureThread() {
    while (frames_captured < TARGET_FRAMES) {
        auto frame = camera_system_->captureFrame();
        {
            std::lock_guard lock(queueMutex);
            captureQueue.push(frame);
        }
    }
}

// Thread 2: Process (runs in parallel)
void processThread() {
    while (true) {
        StereoFrame frame;
        {
            std::lock_guard lock(queueMutex);
            if (captureQueue.empty()) continue;
            frame = captureQueue.front();
            captureQueue.pop();
        }
        processFrame(frame);  // SGM-Census + rectification
    }
}
```

**Challenges:**
- Need lock-free queue for zero-copy frame transfer
- Careful memory management (who owns the frame buffer?)
- Synchronization complexity

**Files to modify:**
- `src/api/HandheldScanPipeline.cpp` (add pipeline threading)
- `src/gui/handheld_scan_widget.cpp` (coordinate threads)

---

### 4. GPU Acceleration (Vulkan) 💎 **LOW PRIORITY**

**Target**: Offload SGM aggregation to GPU
**Expected speedup**: 3-5x for SGM (not total pipeline)
**Complexity**: Very High

#### Why Vulkan (not CUDA/OpenCL):
- Raspberry Pi 5 has VideoCore VII GPU (Vulkan 1.3 support)
- Cross-platform (works on desktop too)
- Modern compute shaders

#### Candidate Operations for GPU:

1. **SGM Cost Aggregation** (most compute-intensive):
   - 8 directional paths → 8 GPU compute dispatches
   - Each path: parallel processing of scanlines
   - Expected: 200ms → 40ms

2. **Census Transform** (good for GPU):
   - Highly parallel (each pixel independent)
   - Expected: 50ms → 10ms

3. **Matching Cost** (moderate for GPU):
   - Hamming distance can be vectorized
   - Expected: 100ms → 30ms

#### Implementation Notes:
- Keep CPU fallback for systems without Vulkan
- Use compute shaders (not graphics pipeline)
- Minimize CPU↔GPU transfers (use staging buffers)

**Files to create:**
- `src/stereo/SGMCensusVulkan.cpp` (Vulkan backend)
- `shaders/census_transform.comp` (GLSL compute shader)
- `shaders/sgm_aggregation.comp` (GLSL compute shader)

**NOT recommended for initial implementation** due to:
- Development complexity
- CPU version already very fast (10s is acceptable)
- Vulkan driver stability on Raspberry Pi

---

## Additional Optimization Ideas

### 5. Adaptive Disparity Range
- **Idea**: Reduce `numDisparities` from 384 to ~128 based on scene depth
- **Speedup**: 3x for matching cost
- **Challenge**: Need robust depth estimation from previous frame

### 6. Multi-resolution Pyramid
- **Idea**: Coarse-to-fine SGM (start at 1/4 resolution, refine at full)
- **Speedup**: 2-3x overall
- **Challenge**: Complexity in parameter tuning

### 7. Reduced Vertical Search Range
- **Idea**: Reduce from ±2px to ±1px if epipolar error < 3px
- **Speedup**: 40% for matching cost
- **Risk**: May lose valid disparities on edges

---

## Performance Profiling Tools

When implementing future optimizations, use these tools to measure impact:

```bash
# CPU profiling with perf
perf record -g ./build/src/gui/unlook_scanner
perf report

# Memory profiling with valgrind
valgrind --tool=massif ./build/src/gui/unlook_scanner

# ARM PMU counters
perf stat -e cycles,instructions,cache-misses,cache-references \
    ./build/src/gui/unlook_scanner
```

---

## Benchmark Results Template

When implementing optimizations, document results here:

```markdown
### [Optimization Name] - [Date]

**Hardware**: Raspberry Pi 5
**Baseline**: X seconds
**Optimized**: Y seconds
**Speedup**: Z%

**Changes**:
- File1.cpp: description
- File2.cpp: description

**Profiling**:
- Function A: before Xms → after Yms
- Function B: before Xms → after Yms

**Side effects**:
- Quality impact: [none/minor/significant]
- Memory impact: [reduced/same/increased]
```

---

## References

- OpenCV Performance Guide: https://docs.opencv.org/4.6.0/dc/d71/tutorial_py_optimization.html
- ARM NEON Programming Guide: https://developer.arm.com/documentation/102467/latest
- Vulkan Compute Tutorial: https://www.khronos.org/blog/vulkan-tutorial
- Real-time Stereo Survey: Scharstein & Szeliski (2002)
- SGM Original Paper: Hirschmueller (2008)
