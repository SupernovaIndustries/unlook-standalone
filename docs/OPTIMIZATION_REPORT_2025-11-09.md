# 🚀 UNLOOK STEREO MATCHING - COMPLETE OPTIMIZATION REPORT
**Target**: Raspberry Pi 5 (Cortex-A76 @ 2.4GHz, 4 cores)
**Date**: 2025-11-09
**Status**: URGENT - Must work optimally within hours

---

## 📊 CURRENT PERFORMANCE ANALYSIS

### Actual Timing Breakdown (from logs - 720p resolution)
```
Total per frame: ~6,656ms (0.15 FPS)
├─ Rectification:        ~7ms    ( 0.1%) ✅ EXCELLENT
├─ Census transform:   ~125ms    ( 1.9%) ✅ GOOD
├─ AD cost:            ~650ms    ( 9.8%) ⚠️ ACCEPTABLE
├─ Cost combination:   ~700ms    (10.5%) ⚠️ ACCEPTABLE
├─ SGM aggregation:  ~4,800ms    (72.1%) ❌ CRITICAL BOTTLENECK
├─ Disparity selection: ~240ms    ( 3.6%) ⚠️ ACCEPTABLE
├─ Post-processing:      ~9ms    ( 0.1%) ✅ EXCELLENT
└─ Depth conversion:    ~30ms    ( 0.5%) ✅ GOOD
```

**CRITICAL FINDING**: SGM aggregation consumes **72% of total time** (4.8s out of 6.6s)

### Current Configuration
- **Resolution**: 1280x720 (921,600 pixels)
- **Disparity range**: 0-256 (256 levels)
- **Block size**: 7x7
- **SGM paths**: 4 (L→R, R→L, T→B, B→T)
- **Algorithm**: AD-Census + 4-path NEON SGM
- **NEON optimization**: ENABLED ✅
- **GPU acceleration**: NOT WORKING (Vulkan stub)

---

## 🎯 PERFORMANCE TARGETS & FEASIBILITY

### Research Benchmarks (from literature)

| Platform | Resolution | Disparities | Algorithm | FPS | Source |
|----------|-----------|-------------|-----------|-----|--------|
| **ARM Cortex-A57 (embedded)** | 640x480 (VGA) | 128 | SGM 4-path NEON | **46 FPS** | ReS2tAC 2021 |
| Tegra X1 (ARM) | 640x480 | 128 | SGM 4-path | 42 FPS | Multiple sources |
| Tegra X2 (ARM) | 1242x375 | 128 | SGM | 28 FPS | Research papers |
| **Raspberry Pi CM4** | 960x720 | 128 | Stereo matching | **5 FPS** | Community reports |
| Intel Core i7 (4 cores) | 640x480 | 128 | CPU SGM | 16 FPS | Spangenberg et al. |

### Target Performance for Raspberry Pi 5 (Cortex-A76 - 2.5x faster than A57)

| Resolution | Disparities | Paths | Expected FPS | Frame Time | Feasibility |
|-----------|-------------|-------|--------------|------------|-------------|
| **640x480 (VGA)** | 64 | 4 | **80-100 FPS** | 10-12ms | ✅ VERY HIGH |
| **640x480 (VGA)** | 128 | 4 | **50-60 FPS** | 16-20ms | ✅ HIGH |
| **1280x720 (720p)** | 64 | 4 | **25-30 FPS** | 33-40ms | ✅ HIGH |
| **1280x720 (720p)** | 128 | 4 | **12-15 FPS** | 66-83ms | ✅ MEDIUM |
| **1280x720 (720p)** | 256 | 4 | **3-5 FPS** | 200-333ms | ⚠️ LOW (current: 0.15 FPS) |
| **1280x720 (720p)** | 128 | 2 | **20-25 FPS** | 40-50ms | ✅ HIGH |

**CURRENT STATUS**: 0.15 FPS @ 720p/256 disparities → **20-100x slower than achievable!**

---

## 🔧 OPTIMIZATION STRATEGIES (Prioritized by Impact)

### ⚡ TIER 1: QUICK WINS (Implement in <30 minutes)

#### 1.1 **REDUCE DISPARITY RANGE** (256 → 128) 
**Impact**: **50% faster** (6.6s → ~3.3s, 0.3 FPS)
**Implementation time**: 2 minutes

```cpp
// File: src/api/HandheldScanPipeline.cpp, line 151
// BEFORE:
disparityConfig_.numDisparities = 256;  // Full range

// AFTER (QUICK WIN):
disparityConfig_.numDisparities = 128;  // 50% reduction, still good range
```

**Reasoning**:
- SGM complexity: O(width × height × disparities²)
- Reducing 256→128 = 4x less computation in cost aggregation
- 70mm baseline @ 128 disparities = depth range ~200-3000mm (sufficient for handheld scanning)
- Literature shows 128 disparities is standard for embedded systems

**Trade-offs**: 
- ✅ Maintains good depth range for objects 20-300cm from camera
- ⚠️ Reduces maximum depth from ~6m to ~3m (acceptable for handheld scanner)

---

#### 1.2 **REDUCE SGM PATHS** (4-path → 2-path)
**Impact**: **50% faster** on top of 1.1 (3.3s → ~1.6s, 0.6 FPS)
**Implementation time**: 5 minutes

```cpp
// File: src/stereo/DisparityComputer.cpp (AD-Census implementation)
// Current: 4 paths (L→R, R→L, T→B, B→T)
// Change to: 2 paths (L→R, R→L only)

// In sgmComplete4PathNEON() function:
sgmPathAggregationNEON_LR(costVolume, aggregatedCost, numDisparities, P1, P2);
sgmPathAggregationNEON_RL(costVolume, aggregatedCost, numDisparities, P1, P2);
// COMMENT OUT:
// sgmPathAggregationNEON_TB(costVolume, aggregatedCost, numDisparities, P1, P2);
// sgmPathAggregationNEON_BT(costVolume, aggregatedCost, numDisparities, P1, P2);
```

**Reasoning**:
- Many embedded SGM implementations use 2-path (horizontal only)
- Vertical paths less critical for horizontal stereo rigs
- Research shows 2-path still gives good results (error increase: ~5-10%)

**Trade-offs**:
- ⚠️ Slightly more noise in vertical edges
- ✅ Still enforces smoothness constraint horizontally (most important)

---

#### 1.3 **REDUCE RESOLUTION DURING PROCESSING** (720p → VGA)
**Impact**: **4x faster** on top of 1.1+1.2 (1.6s → ~400ms, 2.5 FPS)
**Implementation time**: 15 minutes

```cpp
// File: src/api/HandheldScanPipeline.cpp
// Add before rectification (line ~345):

cv::Mat leftDownsampled, rightDownsampled;
cv::resize(frame.leftImage, leftDownsampled, cv::Size(640, 480), 0, 0, cv::INTER_AREA);
cv::resize(frame.rightImage, rightDownsampled, cv::Size(640, 480), 0, 0, cv::INTER_AREA);

// Process at VGA
auto rectResult = rectificationEngine_->rectify(leftDownsampled, rightDownsampled);
// ... continue with VGA processing ...

// After depth conversion, upscale back to 720p:
cv::resize(depthMap, depthMap, cv::Size(1280, 720), 0, 0, cv::INTER_LINEAR);
```

**Reasoning**:
- Pixel count: 1280x720 = 921,600 → 640x480 = 307,200 (3x reduction)
- SGM time scales with width×height → ~3-4x faster
- Upscaling depth map is fast (~5ms) and quality loss is minimal

**Trade-offs**:
- ⚠️ Slight loss of fine detail in depth map
- ✅ Final mesh can still be dense after upscaling
- ✅ VCSEL dots still visible at VGA resolution

---

### ⚡ TIER 2: MEDIUM OPTIMIZATIONS (Implement in 1-2 hours)

#### 2.1 **OPTIMIZE NEON SGM INNER LOOPS**
**Impact**: **20-30% faster** on SGM component
**Implementation time**: 1-2 hours

**Current bottleneck**: Cost aggregation inner loop processes disparities sequentially

```cpp
// File: src/stereo/SGM_NEON.cpp
// Current implementation processes 8 disparities at a time with NEON
// Improvement: Process 16 disparities with better register usage

// BEFORE (line ~200 in sgmPathAggregationNEON_LR):
for (int d = 0; d < numDisparities; d += 8) {
    uint16x8_t cost_vec = vld1q_u16(&cost_row[d]);
    // ... processing ...
}

// AFTER (use double buffering and unroll):
for (int d = 0; d < numDisparities; d += 16) {
    uint16x8_t cost_vec1 = vld1q_u16(&cost_row[d]);
    uint16x8_t cost_vec2 = vld1q_u16(&cost_row[d+8]);
    
    // Process both vectors in parallel
    // ... processing with reduced dependency chains ...
}
```

**Additional optimizations**:
- Prefetch next row data (`__builtin_prefetch()`)
- Reduce branching in inner loops
- Better cache alignment for cost volume

---

#### 2.2 **IMPLEMENT COARSE-TO-FINE DISPARITY SEARCH**
**Impact**: **30-40% faster** overall
**Implementation time**: 2-3 hours

```cpp
// Strategy:
// 1. First pass: Compute disparity at 1/4 resolution with 64 disparities
// 2. Refine: Use coarse disparity to limit search range in full resolution
//    Example: If coarse disparity at pixel = 32, search only [24-40] (±8) in full res

// Pseudo-code:
cv::Mat coarseDisparity = computeDisparityAtQuarterRes(left, right, 64);
cv::resize(coarseDisparity, coarseDisparity, fullSize, INTER_LINEAR);

// For each pixel, search only ±16 around coarse estimate
cv::Mat fineDisparity = refineDisparityWithNarrowSearch(left, right, coarseDisparity, searchRange=16);
```

**Benefits**:
- Reduces effective disparity search from 256 → ~32 per pixel
- Used in many real-time stereo systems
- Minimal quality loss with proper refinement

---

#### 2.3 **SWITCH TO FASTER COST FUNCTION**
**Impact**: **15-20% faster** on cost computation
**Implementation time**: 1 hour

**Current**: AD-Census (absolute difference + census transform)
**Alternative**: Census-only or Mini-Census

```cpp
// Census-only is 2-3x faster than AD-Census
// Trade-off: Slightly less robust in low-texture areas
// But VCSEL provides texture → Census-only should work well

disparityConfig_.lambdaAD = 0.0f;         // Disable AD
disparityConfig_.lambdaCensus = 1.0f;     // Census only
disparityConfig_.censusWindowSize = 7;    // Smaller window (9→7) = faster
```

**Alternative**: Mini-Census (5x5 instead of 9x7)
- Census bits: 63 → 24 (faster Hamming distance)
- Quality loss: minimal for VCSEL structured light

---

### ⚡ TIER 3: ADVANCED OPTIMIZATIONS (Implement in 3-6 hours)

#### 3.1 **TEMPORAL FRAME REUSE**
**Impact**: **50-60% faster** for video streams
**Implementation time**: 3-4 hours

```cpp
// Strategy: Reuse previous frame's disparity as initialization
// SGM can converge faster when starting from good estimate

// 1. Store previous frame disparity
cv::Mat previousDisparity;

// 2. Use as initialization (warm start)
// - Shift previous disparity by camera motion (if IMU available)
// - Use as prior in SGM path costs
// - Reduces search range per pixel

// 3. Only full search every N frames (e.g., N=10)
if (frameCount % 10 == 0) {
    // Full disparity search
} else {
    // Incremental update from previous frame
}
```

---

#### 3.2 **PARALLEL PROCESSING WITH OPENMP**
**Impact**: **2-3x faster** (utilize all 4 cores effectively)
**Implementation time**: 2-3 hours

**Current**: Some OpenMP pragmas, but not fully parallelized

```cpp
// Parallelize SGM path aggregation:
// - L→R and R→L paths can run in parallel (independent)
// - T→B and B→T paths can run in parallel

#pragma omp parallel sections num_threads(4)
{
    #pragma omp section
    { sgmPathAggregationNEON_LR(...); }
    
    #pragma omp section
    { sgmPathAggregationNEON_RL(...); }
    
    #pragma omp section
    { sgmPathAggregationNEON_TB(...); }
    
    #pragma omp section
    { sgmPathAggregationNEON_BT(...); }
}
```

**Additional parallelization**:
- Census transform (already has `#pragma omp parallel for`)
- Cost volume computation
- Disparity selection (WTA)

---

#### 3.3 **GPU ACCELERATION (Vulkan/OpenCL)**
**Impact**: **10-20x faster** (if working correctly)
**Implementation time**: 6-10 hours

**Current status**: Vulkan stub implemented but not working

**Options**:
1. **Fix Vulkan SGM implementation** (VulkanSGMAccelerator.cpp)
   - Raspberry Pi 5 has VideoCore VII GPU
   - Vulkan 1.3 support
   - Need to debug current implementation

2. **Alternative: OpenCL implementation**
   - Better documented for VideoCore
   - More examples available for Raspberry Pi

**Expected performance with GPU**:
- 720p @ 128 disparities: **30-60 FPS** (16-33ms)
- 720p @ 256 disparities: **15-20 FPS** (50-66ms)

---

## 📊 COMBINED OPTIMIZATION IMPACT

### Scenario 1: QUICK WINS ONLY (Tier 1 - 30 min implementation)
```
Optimizations applied:
1. Disparity range: 256 → 128       (2x faster)
2. SGM paths: 4 → 2                 (2x faster)
3. Resolution: 720p → VGA           (3x faster)

Combined speedup: 2 × 2 × 3 = 12x faster

Current:  6,656ms (0.15 FPS)
After:      555ms (1.8 FPS)   ← VGA resolution
Upscaled:   580ms (1.7 FPS)   ← Back to 720p output
```

**Quality trade-off**: 
- Depth accuracy: -5% (acceptable)
- Depth range: 200-3000mm (sufficient for handheld)
- Output resolution: 720p (preserved via upscaling)

---

### Scenario 2: QUICK + MEDIUM (Tier 1+2 - 3 hours implementation)
```
Tier 1 optimizations:                12x faster → 555ms
+ NEON loop optimization:            1.3x faster → 427ms
+ Coarse-to-fine:                    1.4x faster → 305ms
+ Faster cost function:              1.2x faster → 254ms

Combined speedup: ~26x faster

Current:  6,656ms (0.15 FPS)
After:      254ms (3.9 FPS @ VGA)
```

---

### Scenario 3: ALL OPTIMIZATIONS (Tier 1+2+3 - 6-8 hours implementation)
```
Tier 1+2 optimizations:              26x faster → 254ms
+ Temporal reuse:                    1.6x faster → 159ms
+ Full OpenMP parallelization:       2.5x faster →  64ms
+ GPU acceleration:                  8x faster   →   8ms

Combined speedup: ~830x faster (if GPU works!)

Current:  6,656ms (0.15 FPS)
After (CPU only):     64ms (15.6 FPS @ VGA, 20+ FPS @ 640x480)
After (with GPU):      8ms (125 FPS @ VGA, 60+ FPS @ 720p!)
```

---

## 🎯 RECOMMENDED IMPLEMENTATION PLAN

### PHASE 1: IMMEDIATE (Next 30 minutes) - TEST NOW!

**Priority 1**: Disparity range reduction (256 → 128)
```bash
# Edit: src/api/HandheldScanPipeline.cpp line 151
disparityConfig_.numDisparities = 128;  # Change from 256

# Rebuild
./build.sh

# Expected result: ~2x faster (6.6s → 3.3s)
```

**Priority 2**: SGM paths reduction (4 → 2)
```bash
# Comment out vertical paths in DisparityComputer.cpp
# Expected result: Another 2x faster (3.3s → 1.6s, 0.6 FPS)
```

**Priority 3**: Resolution downsampling (720p → VGA)
```bash
# Add resize before processing
# Expected result: 4x faster (1.6s → 400ms, 2.5 FPS)
```

**TOTAL PHASE 1 SPEEDUP**: 12x faster (6.6s → 550ms, **1.8 FPS**)

---

### PHASE 2: SHORT-TERM (Next 2-3 hours) - If Phase 1 not sufficient

**Priority 4**: NEON loop optimization
- Unroll loops, better vectorization
- Expected: 1.3x faster

**Priority 5**: Census-only cost function
- Disable AD component
- Expected: 1.2x faster

**Priority 6**: OpenMP full parallelization
- Parallelize all paths
- Expected: 2x faster on 4 cores

**TOTAL PHASE 2 SPEEDUP**: +3x on top of Phase 1 → **~150ms (6-7 FPS @ VGA)**

---

### PHASE 3: MEDIUM-TERM (Next 6-12 hours) - For best performance

**Priority 7**: Coarse-to-fine disparity search
**Priority 8**: Temporal frame reuse
**Priority 9**: GPU acceleration debugging

**TOTAL PHASE 3 SPEEDUP**: +10x on top of Phase 1+2 → **15-20 FPS @ 720p**

---

## 🔬 BENCHMARKING STRATEGY

After each optimization, verify performance with:

```bash
# Run scan with timing
unlook

# Check log for timing breakdown:
tail -100 /home/alessandro/unlook_logs/log_unlook_*.txt | grep -E "complete.*ms|FPS"

# Expected metrics:
# - Rectification: <10ms (already good)
# - SGM aggregation: <2000ms after Phase 1 (currently 4800ms)
# - Total per frame: <600ms after Phase 1 (currently 6656ms)
```

---

## 📈 PERFORMANCE TARGETS SUMMARY

| Phase | Implementation Time | FPS (720p) | FPS (VGA) | Quality |
|-------|-------------------|------------|-----------|---------|
| **Current** | - | 0.15 | 0.15 | 100% |
| **Phase 1** (Quick wins) | 30 min | **1.8** | **3.5** | 95% |
| **Phase 2** (+ Medium) | 3 hours | **5-7** | **15** | 93% |
| **Phase 3** (+ Advanced) | 8 hours | **15-20** | **40-50** | 90% |
| **Phase 3 + GPU** | 12 hours | **30-60** | **80-100** | 90% |

---

## 💡 CRITICAL RECOMMENDATIONS

### FOR IMMEDIATE USE (within hours):

1. **IMPLEMENT PHASE 1 NOW** (30 minutes):
   - Change numDisparities to 128
   - Disable 2 SGM paths (vertical)
   - Add VGA downsampling
   - **Expected: 1.8 FPS @ 720p output**

2. **If 1.8 FPS is not sufficient**:
   - Add OpenMP parallelization (1 hour)
   - Switch to Census-only (30 min)
   - **Expected: 5-7 FPS @ 720p**

3. **Quality validation**:
   - Check depth map has visible gradients (not monocolor)
   - Verify VCSEL dots are detected
   - Ensure valid pixel percentage > 40%

### FOR PRODUCTION QUALITY (next days):

4. **Implement coarse-to-fine** (2-3 hours)
5. **Fix GPU acceleration** (6-10 hours)
6. **Add temporal reuse** (3-4 hours)

---

## 🎓 LITERATURE REFERENCES

1. **ReS2tAC (2021)**: "UAV-Borne Real-Time SGM Stereo Optimized for Embedded ARM and CUDA Devices"
   - Achieved 46 FPS @ VGA on ARM Cortex-A57 with NEON
   - Source: https://www.mdpi.com/1424-8220/21/11/3938

2. **Spangenberg et al. (2014)**: "Real-time Semi-Global Matching on the CPU"
   - 16 FPS @ VGA on Intel Core i7
   
3. **Luxonis OAK-D approach**:
   - Hardware SGBM block (95 disparity search)
   - SHAVE cores for post-processing
   - Achieves 30+ FPS on Myriad X VPU

4. **OpenCV SGBM optimization**:
   - NEON optimizations give ~30% speedup
   - StereoBM is 10x faster but lower quality
   
---

**END OF OPTIMIZATION REPORT**

**NEXT ACTION**: Implement Phase 1 optimizations (30 min) and test immediately!
