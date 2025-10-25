# 🔧 SGBM Parameters Optimization for Ambient Light Stereo (NO VCSEL)

**Date**: 2025-10-25
**Context**: Testing stereo vision WITHOUT VCSEL structured light (all VCSEL units broken)
**Baseline**: 70.017mm (from calib_boofcv_test3.yaml)
**Resolution**: 1456x1088 SBGGR10
**Calibration RMS**: 0.242px (WARNING: Too high, target <0.15px)

---

## 📊 CURRENT PARAMETERS ANALYSIS

### Current SGBMStereoMatcher Configuration (Lines 14-74)

```cpp
// CURRENT: Optimized for VCSEL dot pattern matching
params_.minDisparity = 0;
params_.numDisparities = 448;      // ✅ GOOD: Maximum range
params_.blockSize = 3;             // ⚠️ TOO SMALL for ambient light
params_.P1 = 72;                   // ⚠️ TOO LOW for natural scenes
params_.P2 = 288;                  // ⚠️ TOO LOW for natural scenes
params_.uniquenessRatio = 22;      // ⚠️ TOO LOW (accepts poor matches)
params_.textureThreshold = 3;      // ⚠️ TOO LOW for ambient light
params_.preFilterCap = 15;         // ⚠️ TOO LOW
params_.speckleWindowSize = 15;    // ⚠️ TOO SMALL
params_.speckleRange = 128;        // ⚠️ TOO WIDE
params_.wlsLambda = 5500.0;        // ✅ GOOD
params_.wlsSigma = 1.0;            // ✅ GOOD
params_.leftRightCheck = true;     // ✅ GOOD
params_.disp12MaxDiff = 1;         // ✅ GOOD
params_.mode = MODE_SGBM_3WAY;     // ✅ GOOD
```

---

## ⚠️ PROBLEMI IDENTIFICATI

### 1. **blockSize = 3 (TROPPO PICCOLO per luce ambiente)**

**Problema**:
- blockSize=3 è ottimizzato per **15K VCSEL dots** (pattern ad alta frequenza)
- Con luce ambiente naturale: troppo sensibile a rumore, bassa robustezza

**Effetto**:
- Matching instabile in aree low-texture
- Troppi falsi match
- Disparity map rumorosa

**Soluzione**:
```cpp
params_.blockSize = 7;  // OPTIMAL for natural scenes (was 3)
```

**Rationale**:
- OpenCV recommendation: 5-21 for natural scenes
- 7 = good balance (speed vs quality)
- 9-11 = better quality, slower
- **DO NOT go below 5** for ambient light

---

### 2. **P1/P2 Troppo Bassi (Smoothness Insufficiente)**

**Problema**:
```cpp
P1 = 72;   // 8 * 1 * 3 * 3 (for blockSize=3, VCSEL dots)
P2 = 288;  // 32 * 1 * 3 * 3
```

**Effetto**:
- Insufficient smoothness penalty
- Depth discontinuities troppo aggressive
- Rumore in superfici continue

**Soluzione per blockSize=7**:
```cpp
params_.P1 = 8 * 1 * 7 * 7;   // = 392  (was 72)
params_.P2 = 32 * 1 * 7 * 7;  // = 1568 (was 288)
```

**Formula Standard SGBM**:
```
P1 = 8 * cn * blockSize^2    (small smoothness penalty)
P2 = 32 * cn * blockSize^2   (large smoothness penalty)
cn = number of channels (1 for grayscale)
```

**Ratio P2/P1 = 4**: Optimal per superfici naturali

---

### 3. **uniquenessRatio = 22 (TROPPO PERMISSIVO)**

**Problema**:
- uniquenessRatio=22 accetta match con solo 22% differenza dal secondo miglior match
- Con VCSEL: va bene (dots ben definiti)
- Con luce ambiente: troppi falsi positivi

**Effetto**:
- Matching ambiguo in aree low-texture
- Outliers in disparity map

**Soluzione**:
```cpp
params_.uniquenessRatio = 10;  // STRICT (was 22)
```

**Raccomandazione OpenCV**:
- 5-15: Strict (meno match, più accurati)
- 15-25: Balanced
- 25+: Permissive (più match, meno affidabili)

**Per ambient light**: 10-15 optimal

---

### 4. **textureThreshold = 3 (TROPPO BASSO)**

**Problema**:
- textureThreshold=3 accetta aree quasi completamente untextured
- Ottimizzato per VCSEL dots (texture implicita)
- Con luce ambiente: genera disparity in aree senza informazione

**Effetto**:
- False matches su superfici uniformi (pareti bianche, plastica)
- Depth instabile in low-texture regions

**Soluzione**:
```cpp
params_.textureThreshold = 10;  // BALANCED (was 3)
```

**Raccomandazione**:
- 10: Minimum texture for reliable matching
- 20+: Very strict (solo aree high-texture)

---

### 5. **preFilterCap = 15 (TROPPO BASSO)**

**Problema**:
- preFilterCap limita il range del preprocessing Sobel filter
- 15 = minimal preprocessing (buono per VCSEL high-contrast dots)
- Ambient light: serve più preprocessing per normalizzare

**Effetto**:
- Matching instabile con variazioni di illuminazione
- Poor performance in ombre/highlights

**Soluzione**:
```cpp
params_.preFilterCap = 31;  // BALANCED (was 15)
```

**Range**: 1-63 (più alto = più normalizzazione)
**Raccomandazione**: 31-63 per natural scenes

---

### 6. **speckleWindowSize = 15 (TROPPO PICCOLO)**

**Problema**:
- speckleWindowSize=15 rimuove solo cluster <15 pixels
- Con ambient light: più rumore → servono filtri più aggressivi

**Soluzione**:
```cpp
params_.speckleWindowSize = 100;  // AGGRESSIVE (was 15)
```

**Raccomandazione**:
- 50-100: Standard per natural scenes
- 100-200: High quality (più lento)

---

### 7. **speckleRange = 128 (TROPPO AMPIO)**

**Problema**:
- speckleRange=128 permette disparity difference fino a 128 pixels in un cluster
- Troppo permissivo → preserva rumore

**Soluzione**:
```cpp
params_.speckleRange = 32;  // STRICT (was 128)
```

**Raccomandazione**:
- 16-32: Strict (rimuove più speckles)
- 32-64: Balanced

---

## ✅ PARAMETRI CORRETTI - STEREO AMBIENT LIGHT

### Configurazione Ottimizzata per Luce Naturale

```cpp
// AMBIENT LIGHT OPTIMIZED PARAMETERS - Natural Scene Matching
// Target: Robust stereo without structured light
// Scene: Indoor/outdoor with natural texture and illumination

// Disparity range (KEEP as is - good coverage)
params_.minDisparity = 0;         // ✅ CORRECT
params_.numDisparities = 448;     // ✅ CORRECT (maximum range)

// CHANGED: Larger block for ambient light matching
params_.blockSize = 7;            // OPTIMAL for natural scenes (was 3)
                                  // 5-9 range recommended for non-VCSEL

// CHANGED: P1/P2 recalculated for blockSize=7
params_.P1 = 392;                 // 8 * 1 * 7 * 7 (was 72)
params_.P2 = 1568;                // 32 * 1 * 7 * 7 (was 288)
                                  // P2/P1 ratio = 4 (optimal)

// CHANGED: Stricter uniqueness for reliable matching
params_.uniquenessRatio = 10;     // STRICT (was 22)
                                  // Reject ambiguous matches

// CHANGED: Higher texture threshold
params_.textureThreshold = 10;    // BALANCED (was 3)
                                  // Require minimum scene texture

// CHANGED: More preprocessing normalization
params_.preFilterCap = 31;        // BALANCED (was 15)
                                  // Better illumination handling

// CHANGED: Aggressive speckle filtering
params_.speckleWindowSize = 100;  // AGGRESSIVE (was 15)
params_.speckleRange = 32;        // STRICT (was 128)
                                  // Remove small isolated regions

// WLS filter (KEEP as is - already optimal)
params_.useWLSFilter = true;      // ✅ CORRECT
params_.wlsLambda = 5500.0;       // ✅ CORRECT
params_.wlsSigma = 1.0;           // ✅ CORRECT

// Left-right check (KEEP as is - essential)
params_.leftRightCheck = true;    // ✅ CORRECT
params_.disp12MaxDiff = 1;        // ✅ CORRECT

// Mode (KEEP as is - best quality)
params_.mode = cv::StereoSGBM::MODE_SGBM_3WAY;  // ✅ CORRECT
```

---

## 🔧 IMPLEMENTAZIONE - Come Applicare le Modifiche

### Option 1: Modifica Diretta in SGBMStereoMatcher.cpp

```cpp
// In SGBMStereoMatcher::SGBMStereoMatcher() constructor
// Replace lines 14-52 with:

SGBMStereoMatcher::SGBMStereoMatcher() {
    // AMBIENT LIGHT OPTIMIZED - Natural scene stereo matching
    // NO VCSEL structured light, rely on scene texture only

    // Maximum disparity range for comprehensive depth coverage
    params_.minDisparity = 0;
    params_.numDisparities = 448;  // Support 25cm-2m at 70mm baseline

    // AMBIENT LIGHT: Larger block for robust matching
    params_.blockSize = 7;         // Optimal for natural texture (NOT VCSEL dots)

    // P1/P2 recalculated for blockSize=7
    params_.P1 = 392;              // 8 * 1 * 7 * 7
    params_.P2 = 1568;             // 32 * 1 * 7 * 7 (ratio = 4)

    // Stricter matching criteria for ambient light
    params_.uniquenessRatio = 10;  // Reject ambiguous matches
    params_.textureThreshold = 10; // Require minimum texture
    params_.preFilterCap = 31;     // Better illumination normalization

    // Aggressive speckle filtering
    params_.speckleWindowSize = 100;
    params_.speckleRange = 32;

    // WLS filter (optimal as is)
    params_.useWLSFilter = true;
    params_.wlsLambda = 5500.0;
    params_.wlsSigma = 1.0;

    // Strict left-right consistency
    params_.leftRightCheck = true;
    params_.disp12MaxDiff = 1;

    // 3-WAY mode for maximum quality
    params_.mode = cv::StereoSGBM::MODE_SGBM_3WAY;

    // Create matcher
    sgbm_ = cv::StereoSGBM::create(
        params_.minDisparity,
        params_.numDisparities,
        params_.blockSize
    );

    updateSGBMParameters();

    // Create right matcher for left-right check
    if (params_.leftRightCheck) {
        rightMatcher_ = cv::ximgproc::createRightMatcher(sgbm_);
    }

    // Create WLS filter
    if (params_.useWLSFilter) {
        createWLSFilter();
    }
}
```

---

### Option 2: Runtime Configuration (GUI Tuning)

Se vuoi testare parametri diversi senza ricompilare:

```cpp
// In GUI DepthTestWidget or main application
auto* sgbmMatcher = dynamic_cast<SGBMStereoMatcher*>(depthProcessor->getStereoMatcher());
if (sgbmMatcher) {
    StereoMatchingParams params = sgbmMatcher->getParameters();

    // Apply ambient light optimizations
    params.blockSize = 7;
    params.P1 = 392;
    params.P2 = 1568;
    params.uniquenessRatio = 10;
    params.textureThreshold = 10;
    params.preFilterCap = 31;
    params.speckleWindowSize = 100;
    params.speckleRange = 32;

    sgbmMatcher->setParameters(params);
}
```

---

## 📈 PERFORMANCE IMPACT

### Processing Time Estimate

```
blockSize=3 (VCSEL):     ~40ms VGA, ~150ms HD
blockSize=7 (Ambient):   ~90ms VGA, ~350ms HD
```

**Trade-off**: 2-2.5x slower, ma **matching molto più robusto**

**Ottimizzazione futura**: GPU acceleration con Vulkan compute shaders

---

## 🎯 EXPECTED IMPROVEMENTS

### Con Parametri Ottimizzati:

1. **Disparity Coverage**: 20-40% → 60-80% (stima)
2. **Match Reliability**: Fewer false positives in low-texture areas
3. **Depth Stability**: Less noise, smoother surfaces
4. **Outlier Reduction**: Aggressive speckle filtering removes isolated errors

### ⚠️ LIMITAZIONE CRITICA: Calibration RMS Error

**IMPORTANTE**: Anche con parametri ottimali, la **calibration quality** è il bottleneck:

```yaml
rms_error: 0.242 pixels  # TOO HIGH (target <0.15px)
```

**Impact**:
- At 500mm distance: ~1-2mm depth error
- 10-20x OVER target precision (0.1mm)

**Solution**: Recalibrate con target RMS <0.15px dopo testing parametri SGBM

---

## 🧪 TESTING PROTOCOL

### Step 1: Test Parametri Base

```bash
# Compile with ambient light parameters
./build.sh

# Test GUI
unlook
```

**Verify**:
- Disparity map coverage (should be >60%)
- Valid points in point cloud (should be >500K for VGA)
- Median disparity >0 (check /tmp/sgbm_disparity.log)

---

### Step 2: Fine-Tuning (se necessario)

**If disparity too sparse**:
```cpp
uniquenessRatio = 15;      // Less strict (accept more matches)
textureThreshold = 5;      // Lower threshold
```

**If disparity too noisy**:
```cpp
speckleWindowSize = 150;   // More aggressive
wlsLambda = 8000.0;        // Stronger smoothing
```

**If depth discontinuities too smooth**:
```cpp
P2 = 1176;                 // Lower P2 (P2/P1 = 3 instead of 4)
```

---

### Step 3: Validate Depth Accuracy

Scansiona oggetto con dimensioni note (es. cubo 100x100x100mm):

```bash
# Export point cloud
# Measure in MeshLab/CloudCompare
```

**Expected accuracy** (con calibration RMS=0.242px):
- At 500mm: ±1-2mm (acceptable per demo)
- At 300mm: ±0.5-1mm (good)
- At 1000mm: ±3-5mm (degraded)

---

## 📝 CONCLUSIONI E RACCOMANDAZIONI

### Modifiche IMMEDIATE per Testing Ambient Light

1. ✅ **Change blockSize: 3 → 7**
2. ✅ **Recalculate P1/P2: 72/288 → 392/1568**
3. ✅ **Stricter uniqueness: 22 → 10**
4. ✅ **Higher texture threshold: 3 → 10**
5. ✅ **More preprocessing: 15 → 31**
6. ✅ **Aggressive speckle: 15/128 → 100/32**

### Dopo Testing Parametri SGBM

1. 🔄 **Recalibrate cameras** (target RMS <0.15px)
2. 🔄 **MATLAB calibration integration** (superior quality available)
3. 🔄 **Epipolar validation** (verify <0.3px mean error)

### Per Demo Success

1. ✅ Optimize SGBM parameters (THIS DOCUMENT)
2. ✅ Implement Artec mesh processing (MISSION_CRITICAL_IMPLEMENTATION.md)
3. 🔄 Test on demo objects
4. 🔄 Fine-tune if needed
5. 🎯 **Demo ready**

---

## 🚀 NEXT STEPS

**IMMEDIATE** (oggi):
```bash
# 1. Apply SGBM parameter changes (Option 1 or 2 above)
# 2. Rebuild
./build.sh

# 3. Test
unlook

# 4. Verify coverage in /tmp/sgbm_disparity.log
# 5. Export point cloud and inspect quality
```

**After validation** (domani):
- Proceed with Artec mesh processing implementation
- Test complete pipeline (SGBM → Point Cloud → Mesh)
- Fine-tune for demo objects

---

**⚠️ CRITICAL**: Test parametri SGBM PRIMA di implementare mesh processing.
Se disparity map è scarsa (low coverage), anche il miglior mesh processing non può aiutare!
