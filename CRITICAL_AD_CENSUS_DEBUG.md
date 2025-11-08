# 🚨 CRITICAL: AD-CENSUS DEBUGGING & FIX (INVESTOR DEMO IN FEW HOURS)

## ⚠️ SITUAZIONE CRITICA

**DEADLINE:** Demo con investitore tra poche ore - DOBBIAMO avere scansione funzionante!
**TARGET:** Calibrazione valida + Scansione 3D + PLY con geometria corretta (anche se non 0.1mm precision)

---

## 🔴 PROBLEMI ATTUALI (CRITICAL ISSUES)

### 1. PLY COMPLETAMENTE PIATTA
- Point cloud generata è PIATTA - nessuna variazione 3D
- Oggetto (mano) appare come superficie 2D invece di volume 3D
- Profondità Z praticamente COSTANTE su tutta la superficie

### 2. DISPARITY MAPS CON FORME GEOMETRICHE CASUALI
**QUESTO È IL BUG PRINCIPALE!**

Invece di mostrare la geometria della scena (mano), le disparity PNG salvate mostrano:
- ❌ Cerchi/ellissi casuali
- ❌ Triangoli geometrici
- ❌ Pattern strani che NON corrispondono agli oggetti reali
- ❌ Bordi verticali artificiali

**Esempio dalla scansione `scan_20251108_013807`:**
```
File: 02_disparity_frame05.png
ASPETTATO: Contorno della mano con variazione graduale di intensità
OTTENUTO: Forme geometriche astratte (cerchi bianchi, triangoli grigi)
```

### 3. DEPTH MAPS UNIFORMI (NO 3D GEOMETRY)
**Analisi RAW values (`02_disparity_frame05_raw.tiff`):**
```
Valid pixels: 96.5% (matching funziona!)
Disparity range: 0.14 - 173.00 pixels

DISTRIBUZIONE:
- 40% pixel: disparity 0-17 (background lontano) ✓
- 28% pixel: disparity 156-173 (MANO) ✗
- Centro mano: disparity = 169.9 COSTANTE

PROBLEMA: Mano intera ha STESSO valore disparity!
Nessuna differenza tra dita/palmo/polso → NO 3D!
```

### 4. LOG METRICHE SEMBRANO OK MA RISULTATI SBAGLIATI
```
[DEBUG] VCSELStereoMatcher processing complete: Valid: 96-97% ✓
[DEBUG] Disparity normalization: min=0.0, max=173.0, range=173.0 ✓
[INFO] Processed 10 depth maps ✓

MA le immagini salvate sono COMPLETAMENTE SBAGLIATE!
```

### 5. RETTIFICAZIONE EPIPOLARE È CORRETTA
**Verificato con test visivo:**
- Feature allineate orizzontalmente tra left/right ✓
- Epipolar lines perfette ✓
- R1/R2 non swappate ✓
- Calibration matrices caricate correttamente ✓

**QUINDI IL PROBLEMA NON È NELLA CALIBRAZIONE O RETTIFICAZIONE!**

---

## 📋 CONTESTO IMPLEMENTAZIONE

### Implementazione Basata su Ricerca Approfondita
L'implementazione AD-CENSUS è stata realizzata seguendo il documento:
**`MEGA_PROMPT_AD_CENSUS_HANDHELD.md`**

Questo documento contiene:
- ✅ Ricerca su competitor (Intel RealSense D400)
- ✅ Analisi repository GitHub: libSGM (Fixstars), opencv_contrib
- ✅ Algoritmo AD-Census completo (AD + Census fusion)
- ✅ ARM NEON optimizations
- ✅ Parametri ottimizzati per 400-1000mm range

**Paper di riferimento:**
- "On building an accurate stereo matching system on graphics hardware" (Xing Mei et al., 2011)

### Files Implementati
```
src/stereo/VCSELStereoMatcher.cpp      # AD-Census algorithm principale
include/unlook/stereo/VCSELStereoMatcher.hpp
src/api/HandheldScanPipeline.cpp       # Multi-frame pipeline
src/gui/handheld_scan_widget.cpp       # GUI handheld scanning
```

### Parametri AD-CENSUS Attuali
```cpp
// PARAMETRI OTTIMIZZATI (scan_20251108_013807):
blockSize = 7
minDisparity = 0
numDisparities = 256
P1 = 8
P2 = 32
uniquenessRatio = 15
speckleWindowSize = 100
disp12MaxDiff = 2
preFilterCap = 63

// RESOLUTION:
Capture: 1456x1088 (native IMX296)
Processing: 1280x720 HD (downsampled per performance)
```

---

## 🔍 DIAGNOSI RICHIESTA (COMPLETE SYSTEM ANALYSIS)

### STEP 1: ANALISI CALIBRAZIONE
**File calibrazione attuale dai log:**
```
[INFO] Calibration matrices extracted: Q=4x4, Baseline=69.924179mm
Calibration file: /unlook_calib/default.yaml → calib-20251106_010224.yaml
```

**TASK:**
1. ✅ Leggere `/unlook_calib/default.yaml` (symlink)
2. ✅ Verificare che punti a calibrazione VALIDATA (nuova da MATLAB in arrivo)
3. ✅ Analizzare TUTTI i campi calibrazione:
   - `image_width`, `image_height` (DEVE essere 1280x720)
   - `camera_matrix_left/right`, `distortion_coeffs_left/right`
   - `rotation_matrix`, `translation_vector`
   - `rectification_transform_left/right` (R1/R2)
   - `projection_matrix_left/right` (P1/P2)
   - **`disparity_to_depth_matrix` (Q matrix) ← CRITICO!**
4. ✅ Verificare che Q matrix sia MATEMATICAMENTE CORRETTA:
   ```
   Q = [[1, 0, 0, -cx],
        [0, 1, 0, -cy],
        [0, 0, 0,  f],
        [0, 0, -1/Tx, (cx-cx')/Tx]]

   Dove: Tx = baseline_mm (dovrebbe essere ~70mm)
   ```
5. ✅ Controllare se remap maps binarie esistono e corrispondono a 1280x720

### STEP 2: ANALISI CARICAMENTO CALIBRAZIONE
**File da analizzare:**
```
src/calibration/CalibrationManager.cpp
src/api/HandheldScanPipeline.cpp (lines 140-172)
```

**VERIFICARE:**
1. ✅ Caricamento corretto di `disparity_to_depth_matrix` vs `Q`
2. ✅ Clonazione matrici: `Q_ = calibData.Q.clone()`
3. ✅ Nessuna modifica accidentale a Q dopo caricamento
4. ✅ Q matrix passata correttamente a `cv::reprojectImageTo3D()`
5. ✅ Remap maps caricate o computate correttamente
6. ✅ Rettificazione usa le map giuste (non swappate L/R)

### STEP 3: ANALISI COMPLETA AD-CENSUS ALGORITHM
**File critico da DEBUGGARE linea per linea:**
```
src/stereo/VCSELStereoMatcher.cpp
```

**CERCARE QUESTI BUG COMUNI:**

#### 3A. CENSUS TRANSFORM BUG
```cpp
// Verificare che Census transform generi BINARY descriptor corretto
// BUG POTENZIALE: Bit order invertito, window size sbagliata

// CORRETTO (da libSGM):
for (int dy = -radius; dy <= radius; dy++) {
    for (int dx = -radius; dx <= radius; dx++) {
        if (dx == 0 && dy == 0) continue;
        census_bits = (census_bits << 1) | (neighbor > center);
    }
}

// VERIFICARE: census_bits è uint64_t o uint32_t?
// VERIFICARE: Ordine bit corretto?
```

#### 3B. HAMMING DISTANCE BUG
```cpp
// Verificare POPCOUNT corretto
// BUG POTENZIALE: XOR sbagliato, count incompleto

// CORRETTO:
uint32_t xor_result = left_census ^ right_census;
int hamming = __builtin_popcount(xor_result);  // O vcntq_u8() su NEON

// VERIFICARE: Hamming distance accumulato correttamente nel cost volume?
```

#### 3C. AD-CENSUS FUSION BUG
```cpp
// Verificare fusion weights corretta
// BUG POTENZIALE: Lambda values invertiti, normalizzazione sbagliata

// CORRETTO:
float lambda_AD = 0.3f;      // Peso AD (texture)
float lambda_Census = 0.7f;  // Peso Census (robustness)
cost = lambda_AD * ad_cost + lambda_Census * census_cost;

// VERIFICARE: Normalizzazione costi prima di fusion?
// VERIFICARE: Range AD vs Census compatibili?
```

#### 3D. SGM AGGREGATION BUG **← PROBABILE CAUSA!**
```cpp
// SGM è COMPLESSO - moltissimi bug possibili!

// VERIFICARE path directions:
// Devono essere: 0° (→), 45° (↗), 90° (↑), 135° (↖)
const int paths[4][2] = {{0, 1}, {1, 1}, {1, 0}, {1, -1}};

// VERIFICARE accumulation formula:
L(p, d) = C(p, d) + min(
    L(p-r, d),           // Same disparity
    L(p-r, d-1) + P1,    // Small change
    L(p-r, d+1) + P1,    // Small change
    min_L(p-r) + P2      // Large change
) - min_L(p-r)

// BUG COMUNI SGM:
// 1. Path direction invertita
// 2. Min propagation sbagliata
// 3. P1/P2 applicati male
// 4. Overflow in accumulation (usare int32_t, non uint16_t!)
```

#### 3E. DISPARITY SELECTION BUG **← ALTRA CAUSA PROBABILE!**
```cpp
// Dopo SGM aggregation, trovare disparity con MINIMUM cost

// VERIFICARE winner-takes-all:
int best_disp = 0;
float min_cost = aggregated_cost[0];
for (int d = 1; d < num_disparities; d++) {
    if (aggregated_cost[d] < min_cost) {
        min_cost = aggregated_cost[d];
        best_disp = d;
    }
}
disparity_map(y, x) = best_disp + minDisparity;

// BUG POSSIBILI:
// 1. Dimenticare +minDisparity
// 2. best_disp mai aggiornato (rimane 0!)
// 3. Cost array indexing sbagliato
```

#### 3F. SUBPIXEL REFINEMENT BUG
```cpp
// Parabolic fitting per subpixel accuracy

// VERIFICARE formula corretta:
float C0 = cost[d-1];
float C1 = cost[d];
float C2 = cost[d+1];
float denom = 2.0f * (C2 + C0 - 2.0f * C1);
float delta = (C0 - C2) / denom;

disparity = d + delta;  // NOT d + delta/16!

// BUG POSSIBILI:
// 1. Delta invertito
// 2. Scaling sbagliato
// 3. Check bounds mancante (d deve essere > 0 e < max-1)
```

#### 3G. POST-PROCESSING BUG **← CAUSA DELL'OVER-SMOOTHING!**
```cpp
// Speckle filter, median filter, bilateral filter

// VERIFICARE che non DISTRUGGA variazione 3D:
// PARAMETRI ATTUALI:
speckleWindowSize = 100  // TROPPO GRANDE!
speckleRange = 16

// BUG: Speckle filter rimuove TUTTA la variazione sulla mano!
// Soluzione: Ridurre drasticamente o disabilitare
```

### STEP 4: RICERCA ONLINE - CORRETTA INTEGRAZIONE

**CERCARE SU:**
1. GitHub Issues di `opencv/opencv_contrib` per `StereoBinarySGBM`
2. Stack Overflow: "AD-Census OpenCV flat disparity"
3. Intel RealSense SDK source code per AD-Census reference
4. libSGM Fixstars: confrontare con implementazione corretta

**DOMANDE DA RISOLVERE:**
- Come OpenCV `StereoBinarySGBM` gestisce Census + AD fusion?
- Qual è il cost volume format corretto? (HxWxD o DxHxW?)
- Come deve essere normalizzato AD cost rispetto Census cost?
- SGM aggregation: forward+backward o solo forward?
- Uniqueness check: quando applicarlo?

### STEP 5: VERIFICARE DOWNSAMPLING 1456x1088 → 1280x720
```cpp
// VERIFICARE che downsampling non introduca artefatti

// METODO USATO:
cv::resize(input, output, cv::Size(1280, 720), 0, 0, cv::INTER_AREA);

// ALTERNATIVA da testare:
cv::pyrDown() multipli invece di resize diretto
```

### STEP 6: CONFRONTO CON OPENCV SGBM STANDARD
**TEST DIAGNOSTICO CRITICO:**
```cpp
// Sostituire temporaneamente AD-Census con SGBM standard OpenCV
cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
    minDisparity, numDisparities, blockSize,
    P1, P2, disp12MaxDiff, preFilterCap, uniquenessRatio,
    speckleWindowSize, speckleRange, cv::StereoSGBM::MODE_SGBM_3WAY
);

sgbm->compute(leftRect, rightRect, disparity);

// SE SGBM STANDARD FUNZIONA → bug in AD-Census implementation
// SE SGBM STANDARD FALLISCE → problema in calibrazione/rettificazione
```

---

## 🎯 DELIVERABLES RICHIESTI

### 1. DIAGNOSI COMPLETA
Report che identifichi:
- ✅ Exact line di codice con bug
- ✅ Perché AD-Census genera forme geometriche invece di depth reale
- ✅ Perché depth è piatta (169.9 costante)
- ✅ Se problema è in Census, AD, Fusion, SGM, o Post-processing

### 2. FIX IMPLEMENTATO
Codice corretto con:
- ✅ Bug risolti
- ✅ Parametri ottimizzati per scansione oggetti reali
- ✅ Validazione con test case

### 3. CALIBRAZIONE VALIDATA
- ✅ Nuova calibrazione da MATLAB (in arrivo da utente)
- ✅ Verificata matematicamente corretta
- ✅ Q matrix, R1/R2, P1/P2 tutti corretti
- ✅ Symlink `/unlook_calib/default.yaml` aggiornato

### 4. SCANSIONE FUNZIONANTE PER DEMO
**TARGET PER DEMO INVESTITORE:**
- ✅ Cattura mano/oggetto a ~40-50cm
- ✅ Disparity map mostra VERA geometria oggetto
- ✅ Depth map ha VARIAZIONE 3D (non piatta!)
- ✅ PLY esportata mostra forma 3D corretta
- ✅ Anche se precision non è 0.1mm, la FORMA deve essere riconoscibile!

---

## ⚙️ PARAMETRI DA TESTARE

### Opzione 1: RIDURRE SMOOTHING (più dettagli)
```cpp
blockSize = 3          // Finestre piccole (era 7)
P1 = 4                 // Meno smoothing (era 8)
P2 = 16                // Molto meno smoothing (era 32)
speckleWindowSize = 20  // Non rimuovere variazioni (era 100)
uniquenessRatio = 10   // Più permissivo (era 15)
```

### Opzione 2: DISABILITARE POST-PROCESSING
```cpp
// Testare SENZA speckle filter, median filter
// Per vedere disparity RAW da SGM
```

### Opzione 3: USARE OPENCV SGBM STANDARD
```cpp
// Fallback a cv::StereoSGBM::create() se AD-Census buggy
// Almeno avremo scansione funzionante per demo!
```

---

## 🚀 EXECUTION PLAN

### PRIORITÀ 1 (NEXT 30 MINUTES): DIAGNOSI RAPIDA
1. ✅ Analizzare VCSELStereoMatcher::computeDisparity() linea per linea
2. ✅ Cercare BUG in SGM aggregation (path directions, accumulation)
3. ✅ Verificare disparity selection (winner-takes-all)
4. ✅ Testare OPENCV SGBM standard come baseline

### PRIORITÀ 2 (NEXT 1 HOUR): FIX & TEST
1. ✅ Implementare fix per bug trovati
2. ✅ Ridurre parametri smoothing
3. ✅ Testare con nuova calibrazione validata MATLAB
4. ✅ Verificare disparity map mostra vera geometria

### PRIORITÀ 3 (NEXT 1 HOUR): VALIDAZIONE DEMO
1. ✅ Scan oggetto test (mano, scatola, oggetto meccanico)
2. ✅ Verificare PLY ha forma 3D corretta
3. ✅ Export PLY per visualizzazione
4. ✅ Preparare demo setup

---

## 📊 SUCCESS CRITERIA

### MINIMO per DEMO (CRITICAL):
- [x] Disparity map mostra VERA forma oggetto (non cerchi/triangoli casuali!)
- [x] Depth map ha VARIAZIONE 3D (non 169.9 costante!)
- [x] PLY esportata riconosce forma oggetto (anche se approssimativa)
- [x] Nessun crash durante acquisizione

### IDEALE per DEMO:
- [ ] Precision <1mm @ 500mm
- [ ] Smooth surface reconstruction
- [ ] Real-time preview funzionante
- [ ] Multi-frame averaging per quality

### QUALITÀ CODICE:
- [ ] Bug documentati e risolti
- [ ] Reference a paper/implementazioni corrette
- [ ] Parametri ottimizzati spiegati
- [ ] Test cases per validazione

---

## 📚 REFERENCE IMPLEMENTATIONS

### libSGM (Fixstars) - GOLD STANDARD
```
Repository: https://github.com/fixstars/libSGM
File chiave: src/sgm.cu, src/census_transform.cu
Usare come reference per SGM aggregation corretta
```

### OpenCV contrib StereoBinarySGBM
```
Repository: https://github.com/opencv/opencv_contrib
Path: modules/stereo/src/stereo_binary_sgbm.cpp
Verificare come implementano Census + Hamming
```

### Intel RealSense
```
D400 Series usa AD-Census
Cercare documentazione/white papers implementation details
```

---

## 🔥 FINAL NOTES

**QUESTO È CRITICAL PATH PER DEMO INVESTITORE!**

Non possiamo permetterci:
- ❌ Forme geometriche casuali nelle disparity
- ❌ PLY piatte senza 3D geometry
- ❌ Crash o errori durante scansione

DOBBIAMO avere:
- ✅ Scansione stabile e riproducibile
- ✅ Geometria 3D riconoscibile
- ✅ Export PLY funzionante

**SE AD-CENSUS È TROPPO BUGGY:**
→ Fallback IMMEDIATO a `cv::StereoSGBM` standard!
→ Meglio scansione con SGBM funzionante che AD-Census rotto!

**TEMPO MASSIMO:** 3-4 ore per diagnosi + fix + test
**DEADLINE ASSOLUTA:** Demo investitore

---

## 🎯 ACTION REQUIRED

1. **ANALIZZA** tutto il codice AD-Census come descritto sopra
2. **CERCA** online per implementazioni corrette
3. **IDENTIFICA** exact bug location
4. **IMPLEMENTA** fix
5. **TESTA** con calibrazione validata MATLAB (in arrivo)
6. **VALIDA** scansione pronta per demo

**GO GO GO! 🚀🔥**
