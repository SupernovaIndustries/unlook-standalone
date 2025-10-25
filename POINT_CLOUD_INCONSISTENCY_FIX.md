# 🔧 Point Cloud Inconsistency Fix - Z Sballate e Skew

**Data**: 2025-10-25
**Problema**: Punti inconsistenti, alcuni con Z sballate, possibile skew
**Status**: Rettifica OK, Epipolari OK, ma point cloud deformato
**Cause**: Outliers in disparity map, matching errors, edge artifacts

---

## 🎯 DIAGNOSI DEL PROBLEMA

### Sintomi:

1. ✅ **Rettifica OK** (epipolar lines orizzontali)
2. ✅ **Calibrazione caricata** (baseline 70mm, focal 1772px)
3. ❌ **Point cloud inconsistente**:
   - Alcuni punti con Z sballate (outliers)
   - Possibile skew/deformazione
   - Distribuzione non uniforme

### Cause Probabili:

#### 1. **Outliers in Disparity Map** (MOST COMMON)
```
Problema: SGBM genera false matches in aree:
- Low texture (pareti uniformi)
- Occlusions (bordi oggetti)
- Specular reflections (superfici lucide)
- Repetitive patterns (texture ripetitiva)

Effetto: Punti con d = 1-5 pixels → Z = 25-125 metri! ❌
```

#### 2. **Edge Artifacts**
```
Problema: Bordi oggetti hanno disparity gradients elevati
→ SGBM smoothness constraint (P1/P2) non riesce
→ Disparity discontinuities creano "flying pixels"

Effetto: Point cloud con "frange" intorno agli oggetti
```

#### 3. **Speckle Noise**
```
Problema: Piccoli cluster di disparity sbagliata
→ Isolated points con Z casuale

Effetto: "Salt and pepper" noise in point cloud
```

#### 4. **Invalid Disparity Handling**
```
Problema: SGBM setta invalid pixels a d=0 o d=-1
→ Conversione Z = (f×B)/0 → inf o NaN

Effetto: Punti all'infinito o degenerate
```

---

## 🔬 DIAGNOSIS TOOLS

### 1. Analizza Disparity Distribution (GIÀ IMPLEMENTATO!)

Controlla `/tmp/sgbm_disparity.log`:

```bash
cat /tmp/sgbm_disparity.log
```

**Cosa cercare**:
```
[SGBM] Disparity distribution analysis:
  Valid pixels: 450000/1580608 (28.5%)  ← ⚠️ Se <50%, problema grave
  Range: [0.5, 448.0] pixels            ← Check min/max sensati
  Mean: 45.2 pixels                     ← Dovrebbe essere 20-150px
  Median: 38.5 pixels                   ← Se median ≈ 0, PROBLEMA CRITICO
  Q1=25.3, Q3=58.7                      ← Quartili per vedere distribuzione

⚠️ WARNING: Median near zero (2.3) while mean is 45.2
   → Highly skewed distribution!
   → Most pixels have invalid/zero disparity
```

**Diagnosi**:
- **Median near zero**: SGBM matching fallisce sulla maggior parte dell'immagine
- **Coverage <30%**: Texture insufficiente o parametri troppo strict
- **Range includes 0-5**: Invalid disparities incluse

---

### 2. Visualizza Disparity Map

```cpp
// In depth_test_widget.cpp, after SGBM compute
cv::Mat disparity_vis;
cv::normalize(disparity, disparity_vis, 0, 255, cv::NORM_MINMAX, CV_8U);
cv::applyColorMap(disparity_vis, disparity_vis, cv::COLORMAP_JET);
cv::imwrite("/tmp/disparity_colormap.png", disparity_vis);

// Also save raw disparity
cv::FileStorage fs("/tmp/disparity_raw.yml", cv::FileStorage::WRITE);
fs << "disparity" << disparity;
fs.release();
```

**Cosa cercare nell'immagine**:
- ❌ **Large black areas**: No disparity (texture insufficiente)
- ❌ **Speckles**: Isolated colored pixels (noise)
- ❌ **Stripes/bands**: Matching errors
- ✅ **Smooth gradients**: Good matching

---

### 3. Point Cloud Quality Check

```cpp
// In PointCloudProcessor, dopo generazione point cloud
void analyzePointCloudQuality(const PointCloud& cloud) {
    std::vector<float> z_values;
    z_values.reserve(cloud.points.size());

    for (const auto& pt : cloud.points) {
        z_values.push_back(pt.z);
    }

    std::sort(z_values.begin(), z_values.end());

    float z_min = z_values.front();
    float z_max = z_values.back();
    float z_median = z_values[z_values.size() / 2];
    float z_q25 = z_values[z_values.size() / 4];
    float z_q75 = z_values[3 * z_values.size() / 4];
    float z_mean = std::accumulate(z_values.begin(), z_values.end(), 0.0f) / z_values.size();

    std::cout << "[PointCloud] Z distribution analysis:" << std::endl;
    std::cout << "  Points: " << cloud.points.size() << std::endl;
    std::cout << "  Z range: [" << z_min << ", " << z_max << "] mm" << std::endl;
    std::cout << "  Z mean: " << z_mean << " mm" << std::endl;
    std::cout << "  Z median: " << z_median << " mm (Q1=" << z_q25 << ", Q3=" << z_q75 << ")" << std::endl;

    // Detect outliers (Z > 3 std deviations from mean)
    float std_dev = 0;
    for (float z : z_values) {
        std_dev += (z - z_mean) * (z - z_mean);
    }
    std_dev = std::sqrt(std_dev / z_values.size());

    int outliers = 0;
    for (float z : z_values) {
        if (std::abs(z - z_mean) > 3 * std_dev) outliers++;
    }

    std::cout << "  Z std dev: " << std_dev << " mm" << std::endl;
    std::cout << "  Outliers (>3σ): " << outliers << " ("
              << (100.0 * outliers / z_values.size()) << "%)" << std::endl;

    // Warning if too many outliers
    if (outliers > 0.05 * z_values.size()) {
        std::cout << "  ⚠️ WARNING: >5% outliers detected! Check disparity filtering." << std::endl;
    }
}
```

---

## ✅ SOLUZIONI (In Ordine di Efficacia)

### **Solution 1: Statistical Outlier Removal** (MASSIMA PRIORITÀ!)

Questa è la soluzione **Artec-grade** che DEVI implementare SUBITO.

#### A. **Durante Disparity Processing** (SGBMStereoMatcher)

```cpp
// In SGBMStereoMatcher::computeDisparity, DOPO WLS filtering

// Step 1: Remove invalid disparities
cv::Mat valid_mask = (disparity > params_.minDisparity) &
                     (disparity < params_.numDisparities);

// Step 2: Statistical filtering on disparity map
cv::Mat disparity_filtered = disparity.clone();

int kernel_size = 5;  // Local neighborhood
float sigma_threshold = 2.0;  // Standard deviations

for (int y = kernel_size; y < disparity.rows - kernel_size; y++) {
    for (int x = kernel_size; x < disparity.cols - kernel_size; x++) {
        if (!valid_mask.at<uchar>(y, x)) continue;

        float center_d = disparity.at<float>(y, x);

        // Collect neighboring valid disparities
        std::vector<float> neighbors;
        for (int dy = -kernel_size; dy <= kernel_size; dy++) {
            for (int dx = -kernel_size; dx <= kernel_size; dx++) {
                if (dx == 0 && dy == 0) continue;
                if (valid_mask.at<uchar>(y+dy, x+dx)) {
                    neighbors.push_back(disparity.at<float>(y+dy, x+dx));
                }
            }
        }

        if (neighbors.size() < 5) {
            disparity_filtered.at<float>(y, x) = 0;  // Isolated point
            continue;
        }

        // Compute local statistics
        float mean = std::accumulate(neighbors.begin(), neighbors.end(), 0.0f) / neighbors.size();
        float variance = 0;
        for (float d : neighbors) {
            variance += (d - mean) * (d - mean);
        }
        float std_dev = std::sqrt(variance / neighbors.size());

        // Remove if center is outlier
        if (std::abs(center_d - mean) > sigma_threshold * std_dev) {
            disparity_filtered.at<float>(y, x) = 0;  // Mark as invalid
            qDebug() << "[SGBM] Outlier removed at (" << x << "," << y
                     << "): d=" << center_d << ", local_mean=" << mean
                     << ", std_dev=" << std_dev;
        }
    }
}

disparity = disparity_filtered;
```

**Aggiungi questo in `SGBMStereoMatcher.cpp` linea ~240** (dopo WLS filter, prima di return)

---

#### B. **Durante Point Cloud Generation** (Open3D)

```cpp
// In PointCloudProcessor::generatePointCloud, DOPO creazione point cloud

#ifdef OPEN3D_ENABLED
// Statistical outlier removal (Artec method)
auto [inlier_cloud, inlier_indices] = pointCloud->RemoveStatisticalOutliers(
    20,    // nb_neighbors (K nearest neighbors)
    2.0    // std_ratio (standard deviation threshold)
);

std::cout << "[PointCloud] Statistical outlier removal:" << std::endl;
std::cout << "  Input points: " << pointCloud->points_.size() << std::endl;
std::cout << "  Output points: " << inlier_cloud->points_.size() << std::endl;
std::cout << "  Removed: " << (pointCloud->points_.size() - inlier_cloud->points_.size())
          << " (" << (100.0 * (pointCloud->points_.size() - inlier_cloud->points_.size()) / pointCloud->points_.size())
          << "%)" << std::endl;

pointCloud = inlier_cloud;
#endif
```

**Aggiungi in `PointCloudProcessor.cpp`** dopo la generazione iniziale del point cloud.

---

### **Solution 2: Disparity Range Validation** (QUICK WIN!)

Filtra disparities che generano Z non sensati:

```cpp
// In DepthProcessor::generatePointCloudFromDisparity

// Calcola Z limits da min/max depth config
float z_min = config.minDepthMm;  // 300mm
float z_max = config.maxDepthMm;  // 2000mm

// Calcola disparity limits corrispondenti
// Z = (f × B) / d  →  d = (f × B) / Z
float d_min = (focal_length * baseline) / z_max;  // d for far objects
float d_max = (focal_length * baseline) / z_min;  // d for close objects

std::cout << "[DepthProcessor] Valid disparity range: [" << d_min << ", " << d_max << "] pixels" << std::endl;
std::cout << "  → Z range: [" << z_min << ", " << z_max << "] mm" << std::endl;

// Filter disparity map
for (int y = 0; y < disparity.rows; y++) {
    for (int x = 0; x < disparity.cols; x++) {
        float d = disparity.at<float>(y, x);

        // Validate disparity range
        if (d < d_min || d > d_max) {
            disparity.at<float>(y, x) = 0;  // Invalid
        }
    }
}
```

**Calcolo con i tuoi parametri**:
```
f = 1772.98 px
B = 70.017 mm
z_min = 300mm, z_max = 2000mm

d_min = (1772.98 × 70.017) / 2000 = 62.1 pixels  (far)
d_max = (1772.98 × 70.017) / 300 = 414.0 pixels  (close)

Valid disparity range: [62, 414] pixels
```

**Aggiungi in `DepthProcessor.cpp`** PRIMA di convertire disparity to point cloud.

---

### **Solution 3: Bilateral Filtering on Depth Map** (Edge-Preserving)

Filtra outliers preservando edges:

```cpp
// In DepthProcessor, dopo conversione disparity → depth map

if (config.applyBilateralFilter) {
    cv::Mat depth_filtered;
    cv::bilateralFilter(
        depth_map,
        depth_filtered,
        5,        // Spatial diameter (was config.bilateralD)
        25.0,     // Sigma color (depth similarity) - REDUCED from 50
        25.0      // Sigma space - REDUCED from 50
    );

    // Replace only valid pixels (preserve masked regions)
    cv::Mat valid_mask = depth_map > 0;
    depth_filtered.copyTo(depth_map, valid_mask);

    std::cout << "[DepthProcessor] Bilateral filtering applied (sigma=25)" << std::endl;
}
```

**Modifica in `DepthProcessor.cpp`** linea ~60-75 (config initialization) → riduci sigma da 50 a 25.

---

### **Solution 4: Confidence-Based Filtering**

Usa la confidence map di WLS filter per rimuovere punti incerti:

```cpp
// In SGBMStereoMatcher::computeDisparity, dopo WLS filter

cv::Mat confidence_map;
if (params_.useWLSFilter && wlsFilter_) {
    // WLS filter genera automaticamente confidence map
    confidence_map = wlsFilter_->getConfidenceMap();

    // Filtra disparity basandosi su confidence
    float confidence_threshold = 0.5;  // 0.0-1.0

    for (int y = 0; y < disparity.rows; y++) {
        for (int x = 0; x < disparity.cols; x++) {
            float conf = confidence_map.at<float>(y, x);
            if (conf < confidence_threshold) {
                disparity.at<float>(y, x) = 0;  // Low confidence → invalid
            }
        }
    }

    std::cout << "[SGBM] Confidence filtering (threshold=" << confidence_threshold << ")" << std::endl;
}
```

---

### **Solution 5: Radius Outlier Removal** (Alternative to Statistical)

Rimuove punti isolati nello spazio 3D:

```cpp
#ifdef OPEN3D_ENABLED
// In PointCloudProcessor, alternativa a Statistical Outlier Removal

auto [inlier_cloud, inlier_indices] = pointCloud->RemoveRadiusOutliers(
    10,    // nb_points: minimo 10 neighbors entro radius
    5.0    // radius: 5mm sphere
);

std::cout << "[PointCloud] Radius outlier removal (10pts in 5mm):" << std::endl;
std::cout << "  Removed: " << (pointCloud->points_.size() - inlier_cloud->points_.size())
          << " points" << std::endl;

pointCloud = inlier_cloud;
#endif
```

---

## 🎯 IMPLEMENTATION PRIORITY

### **OGGI - Quick Fixes (30 min)**:

#### 1. **Disparity Range Validation** (HIGHEST PRIORITY!)
```cpp
// Add in DepthProcessor.cpp line ~400 (in generatePointCloudFromDisparity)

float d_min = 62.0;   // (f×B)/z_max = (1772.98×70.017)/2000
float d_max = 414.0;  // (f×B)/z_min = (1772.98×70.017)/300

// Filter invalid disparities
cv::Mat valid_mask = (disparity > d_min) & (disparity < d_max);
disparity.setTo(0, ~valid_mask);

std::cout << "[DepthProcessor] Disparity range filter: [" << d_min << ", " << d_max << "] px" << std::endl;
int filtered = cv::countNonZero(~valid_mask);
std::cout << "  Filtered " << filtered << " invalid disparities" << std::endl;
```

**Test**: Rebuild, capture, check `/tmp/sgbm_disparity.log` per reduction in outliers.

---

#### 2. **Bilateral Filter Tuning** (5 min)
```cpp
// Modify DepthProcessor.cpp line ~68-73

config.bilateralD = 5;           // Keep small window
config.bilateralSigmaColor = 25.0;  // REDUCE from 50 (less aggressive)
config.bilateralSigmaSpace = 25.0;  // REDUCE from 50
```

**Test**: Rebuild, verify smoother depth map without over-smoothing edges.

---

### **DOMANI - Outlier Removal (1-2 ore)**:

#### 3. **Statistical Outlier Removal** (Point Cloud Level)
```cpp
// Add in PointCloudProcessor.cpp after point cloud generation

#ifdef OPEN3D_ENABLED
auto [inlier_cloud, indices] = pointCloud->RemoveStatisticalOutliers(20, 2.0);
std::cout << "[PointCloud] Removed "
          << (pointCloud->points_.size() - inlier_cloud->points_.size())
          << " outliers" << std::endl;
pointCloud = inlier_cloud;
#endif
```

**Test**: Export PLY, verify cleaner point cloud in MeshLab.

---

#### 4. **Disparity-Level Statistical Filtering** (Advanced)

Add the full kernel-based filtering in `SGBMStereoMatcher.cpp` (see Solution 1A above).

**Test**: Check reduction in isolated disparities in `/tmp/disparity_colormap.png`.

---

## 📊 VALIDATION CHECKLIST

### Before Fixes:
```bash
# 1. Capture frame
unlook → Depth Test → Capture

# 2. Check logs
cat /tmp/sgbm_disparity.log

# 3. Export PLY
Export Point Cloud

# 4. Open in MeshLab
meshlab /tmp/unlook_pointcloud_*.ply
```

**Aspettati**:
- ❌ Many outlier points (Z > 10 metri)
- ❌ Sparse, inconsistent cloud
- ❌ "Flying pixels" around edges

---

### After Fixes:
```bash
# Same test procedure

# Expected improvements:
- ✅ Outliers reduced by 80-90%
- ✅ Denser, more uniform cloud
- ✅ Clean edges (no fringes)
- ✅ Z distribution compact (std_dev <100mm)
```

---

## 🔬 DEBUG VISUALIZATION

Aggiungi in DepthTestWidget per vedere problema visivamente:

```cpp
// In depth_test_widget.cpp, after depth processing

// Save debug images
cv::Mat disparity_vis;
cv::normalize(current_result_.disparity_map, disparity_vis, 0, 255, cv::NORM_MINMAX, CV_8U);
cv::applyColorMap(disparity_vis, disparity_vis, cv::COLORMAP_JET);
cv::imwrite("/tmp/disparity_colormap.png", disparity_vis);

// Save depth map
cv::Mat depth_vis;
cv::normalize(current_result_.depth_map, depth_vis, 0, 255, cv::NORM_MINMAX, CV_8U);
cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_JET);
cv::imwrite("/tmp/depth_colormap.png", depth_vis);

addStatusMessage("Debug images saved: /tmp/disparity_colormap.png, /tmp/depth_colormap.png");
```

**Visualizza**:
```bash
eog /tmp/disparity_colormap.png &
eog /tmp/depth_colormap.png &
```

---

## 💡 QUICK TEST NOW

### Test Immediato (5 min):

```cpp
// TEMPORARY FIX - Add in DepthProcessor.cpp line ~400

// BEFORE point cloud generation, filter disparity:
float d_min = 62.0;
float d_max = 414.0;
cv::Mat valid = (disparity > d_min) & (disparity < d_max);
disparity.setTo(0, ~valid);

int removed = cv::countNonZero(~valid);
std::cout << "[DepthProcessor] Range filter removed " << removed << " invalid disparities" << std::endl;
```

**Rebuild**:
```bash
./build.sh
```

**Test**:
```bash
unlook
# Depth Test → Capture → Export PLY
meshlab /tmp/unlook_pointcloud_*.ply
```

**Se funziona**: Point cloud molto più pulito! 🎉

---

## 🎯 SUMMARY

### Problema: Punti Inconsistenti

**Cause**:
1. ❌ Disparity outliers (SGBM matching errors)
2. ❌ Edge artifacts (depth discontinuities)
3. ❌ No outlier filtering (punti con Z sballate passano)

### Soluzioni (Priority Order):

1. ✅ **Disparity range validation** (NOW - 5 min fix!)
2. ✅ **Bilateral filter tuning** (NOW - 1 line change)
3. ✅ **Statistical outlier removal** (TOMORROW - 30 min)
4. ✅ **Confidence filtering** (LATER - if needed)

### Expected Result:

```
BEFORE: 500K points, 20% outliers, Z range 0-50m ❌
AFTER:  400K points, <2% outliers, Z range 300-800mm ✅
```

---

**Vuoi che implemento il fix immediato (disparity range validation) adesso?** 🚀