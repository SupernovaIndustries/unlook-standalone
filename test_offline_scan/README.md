# Offline Scan Test

Quick test tool to iterate on stereo processing optimizations without using live cameras.

## Purpose

This test replicates the **exact same processing pipeline** as `HandheldScanWidget` but uses pre-captured raw frames instead of live camera feed. This allows for:

- Fast iteration on optimizations
- Consistent test data
- Easy performance comparison
- No need for camera hardware

## Usage

### 1. Build

```bash
cd /home/alessandro/unlook-standalone/test_offline_scan
chmod +x build.sh run.sh
./build.sh
```

### 2. Run

```bash
# Use default frames from last scan
./run.sh

# Or specify custom frames directory
./run.sh /path/to/frames /path/to/calibration.yaml
```

### 3. Check Results

Results are saved in: `/home/alessandro/unlook_debug/test_offline_scan_YYYYMMDD_HHMMSS/`

Output includes:
- Rectified images
- Raw disparity map (TIFF + visualization)
- Depth map (TIFF + visualization)
- Detailed timing breakdown

## What It Tests

The test processes all frames (00-09) from the specified directory and:

1. **Rectification**: Using RectificationEngine with BORDER_REPLICATE fix
2. **Disparity**: Using DisparityComputer with AD-Census + NEON SGM
3. **Depth conversion**: Using Q matrix reprojection

**Configuration matches HandheldScanPipeline exactly**:
- Disparity range: 256 (configurable in code for testing optimizations)
- Block size: 7
- P1: 392, P2: 1568
- AD-Census lambdas: 0.25/0.75
- 4-path SGM (L→R, R→L, T→B, B→T)

## Testing Optimizations

To test optimization changes:

1. **Edit the code** (e.g., change `numDisparities` to 128)
2. **Rebuild**: `./build.sh`
3. **Run**: `./run.sh`
4. **Compare**: Check timing in output vs baseline

### Example: Test Disparity Range Reduction

Edit `test_offline_scan.cpp` line 90:
```cpp
// Change from:
dispConfig.numDisparities = 256;

// To:
dispConfig.numDisparities = 128;  // Test 2x speedup
```

Then:
```bash
./build.sh && ./run.sh
```

## Frame Source

Default frames from: `/home/alessandro/unlook_debug/scan_20251109_023628/`

These are the raw frames from the last scan with:
- VCSEL enabled (350mA)
- Auto-calibrated camera params (16900us, 1.9x gain)
- 10 frames captured

## Output Format

```
=== OFFLINE SCAN TESTER ===
Frames directory: /home/alessandro/unlook_debug/scan_20251109_023628
Calibration file: /unlook_calib/default.yaml
Output directory: /home/alessandro/unlook_debug/test_offline_scan_20251109_031530

--- INITIALIZATION ---
Loading calibration...
✓ Calibration loaded:
  - Image size: 1280x720
  - Baseline: 70.017000 mm
Initializing RectificationEngine...
✓ RectificationEngine initialized
Initializing DisparityComputer...
✓ DisparityComputer initialized
  - Method: AUTO (GPU if available)
  - Disparities: 256
  - Block size: 7
  - P1: 392
  - P2: 1568

--- LOADING FRAMES ---
✓ Loaded frame 0 (1280x720)
...
Total frames loaded: 10

--- PROCESSING FRAMES ---
Processing 10 frames...

╔══════════════════════════════════════════════════╗
║  PROCESSING FRAME 0/10                           ║
╚══════════════════════════════════════════════════╝
Stage 1: Rectification...
✓ Rectification complete (7.2 ms)
  - Valid pixels: 82.0%
Stage 2: Disparity computation...
✓ Disparity complete (6450.5 ms)
  - Method: CPU
  - Valid pixels: 0.078%
Stage 3: Depth conversion...
✓ Depth conversion complete (30.1 ms)

Frame 0 complete:
  - Rectification: 7.2 ms
  - Disparity: 6450.5 ms
  - Depth conversion: 30.1 ms
  - Total: 6487.8 ms
  - Valid pixels: 0.078%

...

╔══════════════════════════════════════════════════════════════╗
║                    PROCESSING SUMMARY                        ║
╚══════════════════════════════════════════════════════════════╝

Frames processed: 10

Average timing per frame:
  Rectification:        7.0 ms
  Disparity:         6500.0 ms  ← BOTTLENECK
  Depth conversion:     30.0 ms
  ────────────────────────────
  TOTAL:             6537.0 ms

Performance:
  Average FPS:     0.153 fps
  Valid pixels:    0.078%

Output directory: /home/alessandro/unlook_debug/test_offline_scan_20251109_031530

✓ Test complete!
```

## Integration with Optimization Report

This test is designed to validate the optimizations described in:
`/home/alessandro/unlook-standalone/docs/OPTIMIZATION_REPORT_2025-11-09.md`

**Quick Win Tests**:

1. **Disparity 256→128**: Change line 90, expect 2x speedup
2. **4-path→2-path**: Comment out TB/BT paths in DisparityComputer, expect 2x speedup
3. **720p→VGA**: Add resize before processing, expect 3x speedup

**Expected Results After Phase 1**:
- Baseline: ~6500ms per frame (0.15 FPS)
- After 128 disparities: ~3250ms (0.31 FPS)
- After 2-path: ~1625ms (0.62 FPS)
- After VGA: ~540ms (1.85 FPS)

## Notes

- First run may show GPU initialization warnings (Vulkan not fully working yet)
- Valid pixel percentage should be >40% after BORDER_REPLICATE fix
- Disparity visualization should show color gradients (not solid blue)
- Depth map should show realistic values (not all 10000mm)
