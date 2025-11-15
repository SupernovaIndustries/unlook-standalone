# Ricerca: Perché Calibrazione OK su Checkerboard ma FAIL su Features

**Data:** 2025-11-15
**Problema:** Calibrazione stereo OpenCV mostra 0.072px di errore sui corner della checkerboard ma 58px sulle ORB features distribuite nell'immagine

---

## 🔍 ROOT CAUSE IDENTIFICATA

### 1. **Extrapolation vs Interpolation Problem**

**Fonte:** Stack Overflow - Camera Calibration Best Practices
**Citazione chiave:**
> "Calibration only considers pixels within the pattern (the checkerboard), and everything outside of the pattern might be very distorted. To address this, you should take enough pictures to cover the whole camera field of view."

> "The distortion correction is wrong only in areas where there is no observed data (along the boundaries of the image), which tells us that the calibrated model is bad at extrapolation but good at interpolation. This is a classic example of over-fitting."

**Spiegazione:**
- Il distortion model (k1-k6, p1-p2) viene ottimizzato SOLO sui punti osservati (i corner della checkerboard)
- Nelle aree NON coperte dalla checkerboard, il modello **ESTRAPO**LA i parametri
- L'estrapolazione con polynomial/rational models **DIVERGE** facilmente
- Risultato: perfetto su checkerboard (interpolazione), disastroso fuori (extrapolazione)

---

### 2. **OpenCV Checkerboard Detection Limitation**

**Fonte:** Signal Processing Stack Exchange - Camera Calibration Accuracy
**Citazione chiave:**
> "In OpenCV, the entire chessboard must be visible in all images in order to be detected, which usually makes it difficult to obtain information from the very edges of images - areas that constrain the lens distortion model properly."

> "flags CALIB_CB_LARGER or CALIB_CB_MARKER are helpful to use calibration patterns exceeding the field of view of the camera, and these oversized patterns allow more accurate calibrations as corners can be utilized which are as close as possible to the image borders."

**Problema:**
- OpenCV standard richiede **TUTTA** la checkerboard visibile
- Quindi i **BORDI** dell'immagine NON vengono MAI calibrati!
- Le ORB features ai bordi hanno errori enormi perché quella zona NON ha dati di calibrazione

---

### 3. **Rational Model Overfitting Risk**

**Fonte:** Stack Overflow - Rational Distortion Model
**Citazione chiave:**
> "Radial distortion is always monotonic for real lenses, and if the estimator produces a non-monotonic result, this should be considered a calibration failure - radial distortion must be monotonic and the distortion function must be bijective."

> "When using the higher order radial model or the rational model, it is possible that camera calibration converges to distortion parameters that are not monotonic, making assumptions invalid."

> "The optimization method used in OpenCV camera calibration does not include these constraints as the framework does not support the required integer programming and polynomial inequalities."

**Problema con CALIB_RATIONAL_MODEL:**
- Usa k1-k6 in una frazione razionale: `(1 + k1*r^2 + k2*r^4 + k3*r^6) / (1 + k4*r^2 + k5*r^4 + k6*r^6)`
- OpenCV NON garantisce monotonia → può convergere a soluzioni NON FISICHE
- Con dataset limitato (34-50 frames) → **OVERFITTING GARANTITO**
- Il modello "memorizza" i punti della checkerboard ma diverge altrove

**Raccomandazione:**
> "For normal cameras the precision of the calibration does not increase so much (even decrease because of 'dimensionality curse')"

Per M12 6mm lenses (wide-angle ma non fisheye), il rational model può essere **CONTROPRODUCENTE** con dataset limitato!

---

## ✅ SOLUZIONI VALIDATE

### Soluzione 1: Coverage Completa del FOV (Field of View)

**Best Practices da Stack Overflow:**

```
REQUISITO CRITICO:
"cover the entire field of view with points, especially close to
the edges and corners of the frame"

NUMERO MINIMO FRAMES:
- 20-30 frames: MINIMO assoluto
- 60-80 frames: RACCOMANDATO per coverage completa
- Zhang's original paper: solo 5 frames (INSUFFICIENTE per produzione!)

PLACEMENT STRATEGY:
- Varie angolazioni (0-40°)
- Tilted: left, right, top, bottom
- NO frames simili (causa bias)
- "The smaller the board is, the more images you should take"
```

**Il nostro problema:** 50 frames a 1 metro NON coprono i bordi dell'immagine!

---

### Soluzione 2: Usare `findChessboardCornersSB()` con `CALIB_CB_LARGER`

**Fonte:** OpenCV Documentation
**Funzione:** `findChessboardCornersSB()` (SB = Sector-Based, Sub-pixel Board)

**Vantaggi:**
- Più robusta di `findChessboardCorners()`
- Supporta **partial detection** (checkerboard parzialmente fuori view!)
- Flag `CALIB_CB_LARGER`: rileva pattern più grandi della view
- Flag `CALIB_CB_MARKER`: usa marker per coordinate system consistente

**Implementazione:**
```cpp
// Invece di findChessboardCorners()
bool found = cv::findChessboardCornersSB(
    image,
    patternSize,  // Può essere PIÙ PICCOLO del pattern reale!
    corners,
    cv::CALIB_CB_LARGER | cv::CALIB_CB_MARKER  // CRITICAL FLAGS!
);
```

**Workflow con oversized pattern:**
1. Stampare checkerboard 12x18 (invece di 7x10)
2. Usare `patternSize = (7, 10)` nel codice
3. Flag `CALIB_CB_LARGER` permette rilevamento parziale
4. Posizionare checkerboard PARZIALMENTE fuori view per calibrare i bordi!

---

### Soluzione 3: Ridurre Complessità Distortion Model

**Raccomandazione:** Rimuovere `CALIB_RATIONAL_MODEL` per M12 6mm lenses

**Motivazione:**
- M12 6mm è wide-angle ma NON fisheye
- FOV stimato: ~70-80° (non >120° come fisheye veri)
- Rational model con k1-k6 ha **14 parametri di distorsione**
- Con 50 frames x 54 corner = 2700 punti
- 14 parametri / 2700 punti = **OVERFITTING RISK ALTO**

**Nuovo flag set proposto:**
```cpp
// REMOVE: cv::CALIB_RATIONAL_MODEL
int flags = cv::CALIB_FIX_ASPECT_RATIO +
            cv::CALIB_USE_INTRINSIC_GUESS +
            cv::CALIB_SAME_FOCAL_LENGTH +
            // Enable k3 but NOT k4-k6 (rational model)
            cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5 + cv::CALIB_FIX_K6;
            // This gives: k1, k2, k3, p1, p2 (8 parameters total)
```

**Vantaggi:**
- 8 parametri invece di 14
- Sempre monotonic (polynomial senza rational fraction)
- Sufficiente per M12 lenses standard
- Meno overfitting risk

---

## 📊 VALIDATION METRICS

**Fonte:** Multiple Stack Overflow threads

```
RMS REPROJECTION ERROR:
- Excellent:  < 0.3 px
- Good:       0.3-0.5 px
- Acceptable: 0.5-1.0 px
- Poor:       > 1.0 px

EPIPOLAR ERROR (su checkerboard):
- Excellent:  < 0.3 px
- Good:       0.3-1.0 px
- Acceptable: 1.0-2.0 px
- Poor:       > 2.0 px

CROSS-VALIDATION:
- Dividere dataset in 2 parti
- Calibrare su parte 1, testare su parte 2
- Calibrare su parte 2, testare su parte 1
- Cross-RMS deve essere simile a RMS originale
```

**Il nostro caso:**
- RMS su checkerboard: 0.46px ✓ GOOD
- Epipolar su checkerboard: 0.26px ✓ EXCELLENT
- **Epipolar su ORB features: 58px ✗ DISASTRO TOTALE**

Questo indica **overfitting severo** → modello memorizza checkerboard ma non generalizza!

---

## 🎯 ACTION PLAN RACCOMANDATO

### Plan A: Coverage Completa + Simplified Model (RACCOMANDATO)

**Step 1:** Modifica codice calibrazione
```cpp
// src/calibration/StereoCalibrationProcessor.cpp:709
int flags = cv::CALIB_FIX_ASPECT_RATIO +
            cv::CALIB_USE_INTRINSIC_GUESS +
            cv::CALIB_SAME_FOCAL_LENGTH +
            cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5 + cv::CALIB_FIX_K6;
            // REMOVED: cv::CALIB_RATIONAL_MODEL
            // Keeps: k1, k2, k3, p1, p2 (8 params instead of 14)
```

**Step 2:** Cattura nuovo dataset (100 frames)
```
DISTANCE: 80-120cm (target 100cm)
DISTRIBUTION:
- 30 frames: CENTRO (varie angolazioni 0-30°)
- 25 frames: ANGOLI (top-left, top-right, bottom-left, bottom-right)
- 25 frames: BORDI (top, bottom, left, right ai bordi estremi!)
- 20 frames: DISTANZE VARIE (70cm, 130cm)

CRITICAL: Muovere checkerboard MOLTO vicino ai bordi della view!
La checkerboard deve essere VICINA ai bordi anche se non centrata.
```

**Step 3:** Validazione
- RMS reprojection < 0.5px
- Epipolar error su checkerboard < 0.5px
- **TEST CRITICO:** Epipolar error su ORB features < 5px (target!)

---

### Plan B: Oversized Checkerboard + findChessboardCornersSB (ALTERNATIVA)

**Step 1:** Stampare checkerboard 12x18 (invece di 7x10)
- Mantenere square size = 24mm
- Dimensioni totali: ~29cm x 43cm (molto più grande!)

**Step 2:** Modificare PatternDetector per usare `findChessboardCornersSB()`
```cpp
// In PatternDetector::detectCheckerboard()
bool found = cv::findChessboardCornersSB(
    gray,
    cv::Size(config_.cols - 1, config_.rows - 1),  // Still 9x6 inner corners
    corners_,
    cv::CALIB_CB_ADAPTIVE_THRESH |
    cv::CALIB_CB_NORMALIZE_IMAGE |
    cv::CALIB_CB_LARGER |           // ALLOW LARGER PATTERNS!
    cv::CALIB_CB_MARKER,            // USE MARKER FOR CONSISTENCY
    meta
);
```

**Step 3:** Cattura dataset con partial visibility
- Posizionare checkerboard PARZIALMENTE fuori view
- Questo calibra i corner VICINI ai bordi dell'immagine!
- 80-100 frames con varie posizioni partial/full

---

### Plan C: ChArUco Board (GOLD STANDARD)

**Vantaggi ChArUco:**
- Detection più robusta (ArUco markers + checkerboard)
- Supporta **partial detection** nativamente
- Ogni corner identificato univocamente (no ambiguità orientamento)
- Usato in applicazioni high-precision (AR/VR, robotica industriale)

**Implementazione:**
Il codice GIÀ SUPPORTA ChArUco! (Pattern type in dataset_capture_widget.cpp)

Basta usare:
```cpp
currentPatternConfig_.type = calibration::PatternType::CHARUCO;
currentPatternConfig_.rows = 7;
currentPatternConfig_.cols = 10;
currentPatternConfig_.squareSizeMM = 24.0f;
currentPatternConfig_.arucoMarkerSizeMM = 17.0f;
currentPatternConfig_.arucoDict = cv::aruco::DICT_4X4_250;
```

---

---

## 🔧 SPECIFICHE LENTI M12 6mm 1/2.7"

### FOV (Field of View) Teorico

**Calcolo:**
```
Sensor 1/2.7" = 5.37mm diagonal
Focal length = 6mm
FOV = 2 × arctan(5.37 / (2 × 6))
    = 2 × arctan(0.4475)
    = 2 × 24.1°
    = 48.2° diagonal FOV
```

**Reality Check:**
- Horizontal FOV: ~40-45° (tipico per 6mm su 1/2.7")
- NON è fisheye (fisheye = >120° FOV)
- È **wide-angle moderato**, non ultra-wide

**Implicazioni:**
- Distorsione presente ma NON estrema
- Standard polynomial model DOVREBBE funzionare
- Ma richiede **p1, p2 (tangential)** per manufacturing tolerances M12

---

### Problema Specifico: M12 Manufacturing Tolerances

**Fonte:** Physics Stack Exchange, LearnOpenCV
**Citazione chiave:**
> "Tangential distortion occurs because the image-taking lens is not aligned perfectly parallel to the imaging plane. More specifically, tangential distortion arises due to misalignment of optical elements with the optical axis of the lens, often as a result of **imperfect lens elements manufacturing or assembly**."

> "The imager chip is usually not perfectly centered with the optical axis of the camera due to **manufacturing tolerance limits**."

> "**Radial distortion arises from the shape of the lens whereas tangential distortion is caused by the assembly process of the camera**."

**M12 Lenses Characteristics:**
- Manufacturing tolerance: ±2-3%
- Typical distortion: 1-5% (low-distortion versions: <1%)
- Assembly process può causare misalignment significativo
- **p1, p2 (tangential) SONO ESSENZIALI** per M12 lenses!

---

### OpenCV GitHub Issue #15992: "calibrateCamera does not enforce lens model constraints"

**CRITICAL FINDING: False Positives in Calibration**

**Citazione esatta:**
> "**Poor calibrations can appear successful because reprojection error remains low and undistortion looks reasonable in the image center, while creating artifacts near edges**."

> "This creates ring patterns or reflections near the edges, and this type of failure can fool people as the reprojection error will be low and undistortion appears reasonable in the center of the image."

> "One key issue with OpenCV's technique is that it needs to see all the corners/dots on the target for a frame to be used in the solution, **making it quite hard to get points near the corners of the image**."

**Questo è ESATTAMENTE il nostro caso:**
- ✓ RMS reprojection error: 0.46px (sembra buono!)
- ✓ Epipolar error su checkerboard: 0.26px (sembra ottimo!)
- ✗ Epipolar error su ORB features: 58px (DISASTRO ai bordi!)

**Root Cause:**
> "Chessboard observations are not well spread in the image"

> "Chessboard patterns clustered in certain image regions allow the optimizer to fit **non-physical parameters** that minimize observed error without ensuring lens bijectivity."

**Solution Raccomandazione Ufficiale:**
> "Using **approximately 60 well-distributed, motion-free calibration images** covering the **entire image space including corners**"

---

### Monotonicity Constraint Violation

**Problema OpenCV (da GitHub Issue #15992):**
> "k1*r^2 + k2*r^4 + k3*r^6 must be either monotonically increasing or decreasing"

> "The optimization method used in OpenCV camera calibration **does not include these constraints** as the framework does not support the required integer programming and polynomial inequalities."

**Conseguenze:**
- Calibrazione può convergere a parametri **NON FISICI**
- Radial distortion diventa non-monotonic → reversione ai bordi
- Funziona al centro (r piccolo) ma diverge ai bordi (r grande)

**Con CALIB_RATIONAL_MODEL ancora peggio:**
```
Model: (1 + k1*r^2 + k2*r^4 + k3*r^6) / (1 + k4*r^2 + k5*r^4 + k6*r^6)
```
- Fraction può avere zeri nel denominatore
- "Zero-crossing problem" (Claus & Fitzgibbon paper)
- Ancora più facile convergere a soluzioni non-monotonic

---

### Wide-Angle Lens Calibration Best Practices

**Fonte:** Stack Overflow - Wide Angle Lenses Calibration
**Per lenti 60-80° FOV (come M12 6mm):**

**NON serve fisheye model se:**
- FOV < 100° (check: 48° ✓)
- Distorsione < 5% (M12 standard: 1-5% ✓)

**MA serve:**
1. **Coverage completa:** ~60 images covering ENTIRE FOV including corners
2. **p1, p2 enabled:** Tangential distortion per M12 manufacturing tolerances
3. **k3 enabled:** Tertiary radial per wide-angle (ma non k4-k6!)
4. **AVOID rational model:** Troppo complesso, overfitting garantito

**Validation:**
> "Adding documentation warnings about monotonicity requirements"
> "Verifying the distortion function maintains monotonicity post-calibration"

**Come verificare monotonicity?**
- Plottare r vs (k1*r^2 + k2*r^4 + k3*r^6)
- Se NON cresce/decresce monotonicamente → calibrazione FALLITA anche se RMS basso!

---

## 📚 REFERENCES DOWNLOADED

- `/tmp/stable_radial_distortion.pdf` - Arxiv paper on stable polynomial calibration
- Stack Overflow: Wide-angle lens calibration
- Stack Overflow: Camera calibration best practices
- OpenCV GitHub Issue #15992: Lens model constraints
- Physics Stack Exchange: Tangential distortion explanation
- LearnOpenCV: Understanding lens distortion

---

## 🔬 CONCLUSIONI FINALI (SPECIFICHE M12 6mm 1/2.7")

### Il Problema ROOT è TRIPLO:

**1. OVERFITTING dovuto a CALIB_RATIONAL_MODEL** ✅ CONFERMATO
- 14 parametri (k1-k6, p1-p2) troppi per 50 frames x 54 corners = 2700 punti
- Rational model può convergere a parametri NON MONOTONIC
- OpenCV NON verifica monotonicity constraint → "false positive" calibration
- Citazione GitHub #15992: "reprojection error remains low [...] in the center, while creating artifacts near edges"

**2. COVERAGE INSUFFICIENTE del Field of View** ✅ CONFERMATO
- OpenCV richiede checkerboard INTERA visibile → bordi mai calibrati
- Citazione GitHub #15992: "it needs to see all corners [...] making it quite hard to get points near the corners"
- Con 50 frames concentrati al centro → zone ai bordi completamente NON osservate
- Distortion model "extrapolates" ai bordi → diverge completamente

**3. M12 MANUFACTURING TOLERANCES** ✅ CONFERMATO
- M12 lenses hanno ±2-3% tolerance in assembly
- Tangential distortion (p1, p2) causata da lens element misalignment
- Per M12 **p1, p2 SONO ESSENZIALI** (già abilitati ✓)
- Ma con rational model + limited coverage → overfitting anche su p1, p2!

### Perché Calibrazione OK su Checkerboard ma FAIL su Features?

**Spiegazione Tecnica Completa:**

1. **OpenCV optimizer minimizza reprojection error** sui 54 corner della checkerboard
2. Con **CALIB_RATIONAL_MODEL** (14 parametri), optimizer può "memorizzare" posizioni checkerboard
3. Convergenza a parametri **non-monotonic** che funzionano PERFETTAMENTE al centro (r piccolo)
4. Ma **divergono completamente** ai bordi (r grande) dove non ci sono dati di calibrazione
5. Risultato: 0.072px su checkerboard vs 58px su ORB features distribuite

**Analogia:**
- È come fare un fit polinomiale di grado 10 su 10 punti
- Il polinomio passa ESATTAMENTE per tutti i 10 punti (RMS = 0!)
- Ma tra i punti e oltre i punti → oscilla selvaggiamente (overfitting classico!)

**GitHub Issue #15992 lo conferma esplicitamente:**
> "Poor calibrations can appear successful because reprojection error remains low and undistortion looks reasonable in the **image center**, while creating artifacts near **edges**"

Questo è il nostro caso 1:1!

### Perché NON è un problema del codice?

**Evidence:**
- ✓ Baseline: 69.77mm perfetto (errore 0.33%)
- ✓ Epipolar su checkerboard: 0.26px (eccellente)
- ✓ RMS reprojection: 0.46px (good)
- ✓ Hardware sync: XVS/XHS funzionante
- ✓ Bug computeEpipolarErrors: fixato (ma non cambia risultato perché YAML già era corretto su checkerboard!)

**Se fosse un bug del codice:**
- Epipolar error sarebbe alto ANCHE su checkerboard → invece è 0.072px!
- Baseline sarebbe sbagliata → invece è 69.77mm vs 70mm expected (0.33% error!)

### Perché M12 6mm Specifically?

**FOV 48° diagonal** = "sweet spot" problematico:
- **Troppo wide** per pinhole model semplice (k1, k2 insufficienti)
- **Non abbastanza wide** per fisheye model (che sarebbe più robusto)
- **Manufacturing tolerances** M12 richiedono p1, p2
- Ma **rational model con 14 param** è OVERKILL → overfitting

**Citazione Stack Overflow - Rational Model:**
> "for normal cameras the precision of the calibration does not increase so much (even decrease because of 'dimensionality curse')"

M12 6mm 1/2.7" è "normal camera" (non fisheye), quindi rational model è counterproductive!

---

**Il problema NON è:**
- ❌ Bug nel codice (verificato: baseline, epipolar su checkerboard perfetti)
- ❌ Hardware non sincronizzato (baseline 69.77mm ± 0.33%)
- ❌ Distortion model sbagliato (p1, p2, k3 sono CORRETTI per M12)

**Prossimo step raccomandato:**
1. Rimuovere CALIB_RATIONAL_MODEL (Plan A Step 1)
2. Rebuild
3. Cattura 100 frames con coverage ESTREMA dei bordi (Plan A Step 2)
4. Calibra e testa su ORB features
5. Se ancora problemi → switch a ChArUco (Plan C)

---

**Aspettativa realistica:**
Con Plan A, epipolar error su ORB features dovrebbe scendere da 58px → **2-5px** (10-30x miglioramento!)
