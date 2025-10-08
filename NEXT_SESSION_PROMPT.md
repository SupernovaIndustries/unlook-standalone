# UNLOOK 3D SCANNER - SESSIONE CORREZIONE DEPTH PROCESSING

**LETTURA OBBLIGATORIA**: Leggi @PROJECT_GUIDELINES.md PRIMA di iniziare qualsiasi implementazione.

---

## 📋 CONTESTO PROBLEMA

### Situazione Attuale
Dopo intensive sessioni di debug su depth processing, abbiamo identificato **4 problemi critici** che impediscono il corretto funzionamento del sistema stereo con pattern VCSEL attivo:

**Sintomi osservati**:
- SGBM: Alta coverage MA Z topologicamente sbagliate (porta 3m più vicina di scatola 60cm)
- ADCensus: Z corrette quando matcha MA coverage 0.73% (inutilizzabile)
- Pattern VCSEL BELAGO 1.2 (15K dots) visibile sulle immagini
- Calibrazione stereo corretta (epipolar lines allineate perfettamente)
- Processing time: ~23 secondi per frame

**Conclusione dell'analisi**: Il problema NON è negli algoritmi fondamentali, ma in:
1. Pipeline preprocessing immagini
2. Parametri SGBM non ottimizzati per VCSEL pattern
3. Mancanza di temporal matching (tecnica standard per structured light)
4. Background noise da oggetti lontani

---

## 🎯 AZIONI CORRETTIVE - PIANO OPERATIVO

### FASE 0: PREPARAZIONE AMBIENTE ⭐⭐⭐⭐⭐

**CRITICO**: Prima di qualsiasi modifica al codice:

1. **Git Revert a Commit Pulito**:
   ```bash
   # Torna a commit PRIMA di:
   # - Implementazione ADCensus/Census Transform
   # - Refactoring AdaptiveHybridStereoMatcher
   # Target: commit con SGBM puro funzionante

   git log --oneline | head -20  # Identifica commit target
   git revert <commit-hash> --no-commit
   # Oppure: git reset --hard <commit-hash>
   ```

2. **Verifica stato pulito**:
   - Solo SGBMStereoMatcher presente
   - Nessun Census/ADCensus nel codebase
   - GUI depth widget con parametri SGBM base
   - Build system funzionante

**Razionale**: Ripartiamo da codebase stabile e testiamo incrementalmente ogni modifica.

---

### PROBLEMA 1: COMPRESSIONE/PREPROCESSING IMMAGINI ⭐⭐⭐⭐⭐

**Diagnosi**:
- Immagini 1456×1088 grayscale pesano 522-560KB (teorico 1.51MB)
- Possibile degradazione pattern VCSEL in pipeline preprocessing
- Debayering SBGGR10→Grayscale potrebbe smoothare dots

**SOLUZIONE**: Usa immagini RAW senza preprocessing lossy

**Implementazione**:

1. **Modifica Camera Capture Pipeline**:
   ```cpp
   // In camera capture:
   // - Salva immagini come RAW Bayer SBGGR10 (NO conversione)
   // - Oppure: estrai B channel direttamente (max IR sensitivity)
   // - NO PNG compression, usa formato lossless o binary dump

   // Debug: confronta variance
   double variance_raw = computeVariance(raw_image);
   double variance_processed = computeVariance(processed_image);
   qDebug() << "Variance RAW:" << variance_raw << "vs Processed:" << variance_processed;
   ```

2. **Stereo Matching su RAW**:
   - Passa immagini raw/B-channel direttamente a SGBM
   - Salta conversioni intermedie
   - Preserva massimo dettaglio VCSEL pattern

**Agent da usare**: `camera-sync-manager` per modifiche camera pipeline

**Validazione**:
- Variance deve aumentare significativamente
- Pattern VCSEL più definito in debug images
- Coverage SGBM deve migliorare

---

### PROBLEMA 2: PARAMETRI SGBM NON OTTIMIZZATI ⭐⭐⭐⭐

**Diagnosi**: Parametri attuali probabilmente per texture naturale, non VCSEL pattern denso

**SOLUZIONE**: Ottimizza parametri SGBM per pattern dots

**Parametri CORRETTI per VCSEL dots densi** (validati da letteratura structured light):

```cpp
// SGBMStereoMatcher configuration per VCSEL pattern
StereoMatchingParams params;

// CRITICAL: Blocco piccolo per catturare singolo dot cluster
params.blockSize = 5;  // Era probabilmente 9-15, troppo grande

// NO pre-filtering per preservare dots
params.preFilterCap = 63;  // Massimo, minimal filtering

// Smoothness penalties (smooth surface assumption)
int channels = 1;  // Grayscale
params.P1 = 8 * channels * params.blockSize * params.blockSize;   // = 200
params.P2 = 32 * channels * params.blockSize * params.blockSize;  // = 800

// ALTO uniqueness per evitare match ambigui (dots identici)
params.uniquenessRatio = 15;  // Era 5-10, troppo permissivo

// Speckle filtering moderato
params.speckleWindowSize = 50;  // Era 100, troppo aggressivo
params.speckleRange = 64;

// Disparity range OK
params.minDisparity = 0;
params.numDisparities = 320;  // Range completo close-far

// Mode
params.mode = cv::StereoSGBM::MODE_SGBM;  // Standard, non HH
```

**Implementazione**:
- Modifica SGBMStereoMatcher constructor/setParameters
- Hardcode questi valori come default per VCSEL mode
- Rimuovi slider GUI (vedi FASE 1)

**Agent da usare**: `stereo-vision-optimizer` per ottimizzazione parametri

**Validazione**:
- Test su scena attuale (scatola + tavolo)
- Z devono essere topologicamente corrette
- Coverage 40-60% minimo

---

### PROBLEMA 3: MANCANZA TEMPORAL MATCHING ⭐⭐⭐⭐⭐

**Diagnosi**:
- VCSEL pattern crea ambiguità (dots identici distribuiti random)
- Impossibile distinguere pattern da texture naturale
- Scanner professionali usano TUTTI temporal matching

**SOLUZIONE**: Implementa multi-frame temporal structured light

#### Hardware Setup Nuovo (FONDAMENTALE)

**Configurazione 2 VCSEL**:
```
VCSEL 1 (LED1): Posizionato 2cm da Camera 1 (LEFT/MASTER)
  - 15K dots pattern
  - Controllato da AS1170 LED1
  - Current: 200mA

VCSEL 2 (LED2): Posizionato 2cm da Camera 0 (RIGHT/SLAVE)
  - 15K dots pattern (identico a VCSEL 1)
  - Controllato da AS1170 LED2
  - Current: 200mA

Flood: Disabilitato (non necessario con dual VCSEL)
```

**Razionale**: Dual VCSEL elimina occlusion shadows che singolo VCSEL può creare.

#### Sequenza Capture Tripla

```cpp
// SEQUENZA OTTIMALE per temporal matching

// Frame A: VCSEL Camera Master ON
capture_frame(vcsel1=ON, vcsel2=OFF, flood=OFF);
// Pattern proiettato da sinistra, visibile da entrambe camere

// Frame B: VCSEL Camera Slave ON
capture_frame(vcsel1=OFF, vcsel2=ON, flood=OFF);
// Pattern proiettato da destra, visibile da entrambe camere

// Frame C: Nessuna illuminazione attiva (texture naturale)
capture_frame(vcsel1=OFF, vcsel2=OFF, flood=OFF);
// Solo luce ambiente, texture naturale oggetto

// PROCESSING:
// 1. Pattern isolato A = Frame_A - Frame_C (VCSEL1 puro)
// 2. Pattern isolato B = Frame_B - Frame_C (VCSEL2 puro)
// 3. Pattern combinato = (Pattern_A + Pattern_B) / 2 (media per ridurre noise)
// 4. Stereo matching su Pattern Combinato (zero ambiguità ambiente)
```

#### Implementazione Dettagliata

**Step 1**: Hardware Control Upgrade
```cpp
// In AS1170Controller o VCSELProjector:

struct TripleFrameCapture {
    cv::Mat frame_vcsel1_left, frame_vcsel1_right;  // VCSEL1 ON
    cv::Mat frame_vcsel2_left, frame_vcsel2_right;  // VCSEL2 ON
    cv::Mat frame_ambient_left, frame_ambient_right; // No VCSEL
};

TripleFrameCapture captureTemporalSequence() {
    TripleFrameCapture result;

    // Frame A: VCSEL1 (Master camera side)
    activateLED1(200);  // VCSEL1 ON
    deactivateLED2();   // VCSEL2 OFF
    std::this_thread::sleep_for(50ms);  // Settle time
    camera_system->captureSingle(result.frame_vcsel1_left, result.frame_vcsel1_right);

    // Frame B: VCSEL2 (Slave camera side)
    deactivateLED1();   // VCSEL1 OFF
    activateLED2(200);  // VCSEL2 ON
    std::this_thread::sleep_for(50ms);
    camera_system->captureSingle(result.frame_vcsel2_left, result.frame_vcsel2_right);

    // Frame C: Ambient (no VCSEL)
    deactivateLED1();
    deactivateLED2();
    std::this_thread::sleep_for(50ms);
    camera_system->captureSingle(result.frame_ambient_left, result.frame_ambient_right);

    return result;
}
```

**Step 2**: Pattern Isolation
```cpp
cv::Mat isolateVCSELPattern(const cv::Mat& vcsel_frame, const cv::Mat& ambient_frame) {
    cv::Mat pattern;
    cv::subtract(vcsel_frame, ambient_frame, pattern);

    // Threshold noise (VCSEL dots devono emergere, rumore no)
    cv::threshold(pattern, pattern, 10, 255, cv::THRESH_TOZERO);

    return pattern;
}

cv::Mat combinePatterns(const cv::Mat& pattern1, const cv::Mat& pattern2) {
    cv::Mat combined;
    cv::addWeighted(pattern1, 0.5, pattern2, 0.5, 0, combined);
    return combined;
}
```

**Step 3**: Stereo Matching Temporale
```cpp
bool DepthProcessor::processTemporalStereo(const TripleFrameCapture& frames,
                                           cv::Mat& depthMap) {
    // Isolate patterns
    cv::Mat pattern_left = combinePatterns(
        isolateVCSELPattern(frames.frame_vcsel1_left, frames.frame_ambient_left),
        isolateVCSELPattern(frames.frame_vcsel2_left, frames.frame_ambient_left)
    );

    cv::Mat pattern_right = combinePatterns(
        isolateVCSELPattern(frames.frame_vcsel1_right, frames.frame_ambient_right),
        isolateVCSELPattern(frames.frame_vcsel2_right, frames.frame_ambient_right)
    );

    // SGBM matching su SOLO pattern VCSEL isolato
    cv::Mat disparity;
    sgbmMatcher->computeDisparity(pattern_left, pattern_right, disparity);

    // Convert to depth
    disparityToDepth(disparity, calibration.Q, depthMap);

    return true;
}
```

**Agent da usare**:
- `hardware-interface-controller` per AS1170 dual VCSEL control
- `stereo-vision-optimizer` per temporal matching pipeline
- `camera-sync-manager` per triple-frame capture sequence

**Validazione**:
- Pattern isolato deve mostrare SOLO dots VCSEL, zero texture ambiente
- Variance pattern isolato >> 200 (alta distintività)
- Coverage 60-80% su qualsiasi superficie
- Z topologicamente corrette

---

### PROBLEMA 4: BACKGROUND NOISE ⭐⭐⭐

**Diagnosi**: Porta a 3m crea noise nel matching, rallenta processing

**SOLUZIONE**: Pre-filtering geometrico + Progressive depth layers

#### Approccio Combinato B+C

```cpp
// STAGE 1: Pre-filtering disparity range (Approccio B)
void setDisparityROI(int min_disparity, int max_disparity) {
    // Focus solo su range utile
    // Disparity 50-320 = 38cm-122m (range scanner utile)
    // Scarta disparity < 50 (oltre 122m, certamente background)

    params.minDisparity = 50;   // Skip molto lontano
    params.numDisparities = 270; // 50+270=320 max

    // Risparmio: non calcola cost per disparità inutili
    // Speed: ~30% più veloce
}

// STAGE 2: Progressive depth layers (Approccio C)
cv::Mat progressiveDepthMatching(const cv::Mat& left, const cv::Mat& right) {
    cv::Mat depthMap = cv::Mat::zeros(left.size(), CV_32F);
    cv::Mat confidenceMap = cv::Mat::zeros(left.size(), CV_32F);

    // Layer 1: NEAR objects (high disparity 150-320)
    cv::Mat disparity_near;
    sgbm_params.minDisparity = 150;
    sgbm_params.numDisparities = 170;
    sgbmMatcher->computeDisparity(left, right, disparity_near);

    // Propagate confidence-guided per layer successivo
    updateDepthMap(disparity_near, depthMap, confidenceMap, LAYER_NEAR);

    // Layer 2: MID objects (disparity 80-180)
    // Usa geometry constraint da layer 1
    cv::Mat disparity_mid;
    sgbm_params.minDisparity = 80;
    sgbm_params.numDisparities = 100;
    // Apply geometric constraints from NEAR layer
    sgbmMatcher->computeDisparityWithPrior(left, right, depthMap, disparity_mid);
    updateDepthMap(disparity_mid, depthMap, confidenceMap, LAYER_MID);

    // Layer 3: FAR objects (disparity 50-100)
    // Solo se necessario (confidence bassa in zone)
    if (hasLowConfidenceRegions(confidenceMap)) {
        cv::Mat disparity_far;
        sgbm_params.minDisparity = 50;
        sgbm_params.numDisparities = 50;
        sgbmMatcher->computeDisparity(left, right, disparity_far);
        updateDepthMap(disparity_far, depthMap, confidenceMap, LAYER_FAR);
    }

    return depthMap;
}
```

**Agent da usare**: `stereo-vision-optimizer` per progressive matching

**Validazione**:
- Processing time ridotto 30-40%
- Background ignorato se oltre range utile
- Focus computazionale su ROI oggetti vicini

---

## 🔧 FASE 1: REFACTORING MINIMO & CLEANUP GUI

**Obiettivo**: Allineare codebase, rimuovere complessità inutile

### Task 1.1: Rimozione Slider Parametri
```cpp
// GUI depth_test_widget.cpp:
// - Rimuovi TUTTI gli slider parametri stereo
// - Mantieni SOLO: algorithm selector → FISSO su SGBM
// - Mantieni: Capture button, Debug checkbox, Export
// - Parametri SGBM: hardcoded ottimali (Problema 2)
```

### Task 1.2: Fixare Modalità Scansione
```cpp
// Rimuovi selezione algoritmo, usa SOLO SGBM ottimizzato:

// In DepthProcessor initialization:
depthProcessor->setStereoMatcher(StereoAlgorithm::SGBM);
// NO AdaptiveHybrid, NO Census, NO selezione runtime

// Hardcode parametri ottimali VCSEL (vedi Problema 2)
StereoMatchingParams params;
params.blockSize = 5;
params.preFilterCap = 63;
params.P1 = 200;
params.P2 = 800;
params.uniquenessRatio = 15;
// ...
depthProcessor->setStereoParameters(params);
```

### Task 1.3: Cleanup Codice Morto
- Rimuovi AdaptiveHybridStereoMatcher (se presente post-revert)
- Rimuovi ADCensusMatcher
- Rimuovi TextureAnalyzer
- Rimuovi Census Transform
- Solo SGBMStereoMatcher rimane

**Agent da usare**: `ux-ui-design-architect` per GUI cleanup, `code-integrity-architect` per review cleanup

---

## 🚀 FASE 2: IMPLEMENTAZIONE SOLUZIONI

### Ordine Implementazione (CRITICO - Seguire esattamente):

1. **PROBLEMA 1: RAW Images Pipeline** ⭐⭐⭐⭐⭐
   - Agent: `camera-sync-manager`
   - Modifica camera capture per RAW/B-channel
   - Test: verifica variance aumentata
   - Build & Test con VCSEL singolo

2. **PROBLEMA 2: Parametri SGBM Ottimizzati** ⭐⭐⭐⭐
   - Agent: `stereo-vision-optimizer`
   - Applica parametri corretti per VCSEL
   - Test: Z topologicamente corrette
   - Build & Test con VCSEL singolo

3. **PROBLEMA 4: Background Filtering** ⭐⭐⭐
   - Agent: `stereo-vision-optimizer`
   - Implementa pre-filtering + progressive layers
   - Test: performance +30%, accuracy migliore
   - Build & Test con VCSEL singolo

4. **PROBLEMA 3: Temporal Matching** ⭐⭐⭐⭐⭐ (FINALE)
   - Agent: `hardware-interface-controller` + `stereo-vision-optimizer`
   - Implementa triple-frame sequence
   - Implementa pattern isolation
   - Test: Prima con VCSEL singolo (baseline)
   - Test: Poi con dual VCSEL (setup finale)

---

## 📋 CHECKLIST QUALITÀ (Prima di ogni step)

Segui @PROJECT_GUIDELINES.md rigorosamente:

- [ ] **Ricerca PRIMA di implementare** (Google Scholar, OpenCV docs, structured light papers)
- [ ] **Analisi algoritmica PRIMA di codificare** (pseudocode, flow diagrams)
- [ ] **Design review**: Valida approccio con letteratura
- [ ] **Code incrementale**: Un problema alla volta, test dopo ogni fix
- [ ] **Logging aggressivo**: qDebug() per ogni stage critico
- [ ] **Validazione numerica**: Verifica variance, coverage, Z ranges
- [ ] **No patch**: Solo soluzioni robuste basate su teoria
- [ ] **Thread safety**: Atomic ops, mutexes dove necessario
- [ ] **Error handling**: Exception catching, validation

---

## 🧪 TESTING PROTOCOL

### Test Fase 1 (Singolo VCSEL - Baseline)
```bash
# Dopo ogni implementazione:
./build.sh -j4

# Test 1: RAW images
# - Verifica variance > 150 (era 86)
# - Pattern VCSEL definito in debug images

# Test 2: SGBM ottimizzato
# - Z scatola ~60cm ± 2cm
# - Z porta ~3m ± 10cm (NON invertite!)
# - Coverage 40-60%

# Test 3: Background filtering
# - Processing < 16sec (era 23sec)
# - Background oltre 1.5m ignorato

# Test 4: Temporal matching (singolo VCSEL)
# - Pattern isolato variance > 200
# - Coverage 60-80%
# - Z precision ± 1cm
```

### Test Fase 2 (Dual VCSEL - Finale)
```bash
# Dopo hardware upgrade dual VCSEL:

# Test 1: Triple-frame capture
# - Sequence timing < 200ms totale
# - 3 frame pairs salvati correttamente

# Test 2: Pattern isolation
# - Pattern_A e Pattern_B distintivi
# - Combined pattern smooth, alta variance

# Test 3: Stereo matching temporale
# - Coverage 70-90% (target)
# - Z precision ± 0.5cm
# - Topologia CORRETTA (porta > scatola > tavolo)

# Test 4: Superficie nera opaca
# - Coverage > 60% anche su nero
# - Z precision mantenuta
```

---

## 🎯 AGENT WORKFLOW

### Agent da Utilizzare (in ordine):

1. **`camera-sync-manager`**: RAW image pipeline, triple-frame capture
2. **`stereo-vision-optimizer`**: SGBM params, temporal matching, progressive layers
3. **`hardware-interface-controller`**: AS1170 dual VCSEL control
4. **`ux-ui-design-architect`**: GUI cleanup, slider removal
5. **`code-integrity-architect`**: Code review, cleanup validation
6. **`cmake-build-system-architect`**: Build finale validato

### Nuovi Agent Necessari?
**NO** - Gli agent esistenti coprono tutte le necessità:
- Camera pipeline: `camera-sync-manager`
- Stereo algorithms: `stereo-vision-optimizer`
- Hardware LED: `hardware-interface-controller`
- GUI: `ux-ui-design-architect`
- Build: `cmake-build-system-architect`

---

## ⚠️ CRITICAL REMINDERS

### APPROCCIO MINDSET:
- **SMART**: Usa teoria consolidata (structured light papers, Intel RealSense)
- **PRECISO**: Ogni parametro giustificato da letteratura o misure
- **DECISO**: No tentennamenti, implementa soluzione completa
- **ANALITICO**: Debug metodico, logging completo, metriche quantitative

### NO PUTTANATE:
- ❌ NO patch temporanee o workaround
- ❌ NO "proviamo così" senza giustificazione teorica
- ❌ NO modifiche parallele non correlate
- ❌ NO build parziali, solo build completi validati
- ✅ SOLO soluzioni robuste basate su analisi profonda
- ✅ SEMPRE logging/metrics per validare ipotesi
- ✅ SEMPRE test incrementale dopo ogni fix

### SEQUENZA RIGOROSA:
1. Git revert a commit pulito
2. Refactoring minimo (Fase 1)
3. Fix incrementali validati (Fase 2)
4. Test singolo VCSEL
5. Hardware upgrade dual VCSEL
6. Test finale dual VCSEL
7. Build production `cmake-build-system-architect`

---

## 📚 RIFERIMENTI TECNICI

### Papers da Consultare:
- "Temporal Phase Unwrapping for Structured Light" (Zhang et al.)
- "Real-time Structured Light Depth Sensing" (Intel RealSense)
- "Optimal Parameters for Semi-Global Matching" (Hirschmuller)
- "VCSEL-based 3D Sensing for Mobile Applications" (ams OSRAM)

### Documentazione:
- OpenCV SGBM documentation
- AS1170 LED Driver datasheet (dual LED control)
- IMX296 sensor Bayer pattern specs
- BELAGO 1.2 VCSEL specifications

---

## 🏁 SUCCESS CRITERIA

**Depth Processing COMPLETO quando**:
- ✅ Pattern VCSEL isolato correttamente (variance > 200)
- ✅ Coverage 70-90% su qualsiasi superficie
- ✅ Z topologicamente corrette (porta > scatola > tavolo)
- ✅ Precision ± 0.5cm a 60cm
- ✅ Processing time < 500ms per frame
- ✅ Funziona su nero opaco come su bianco
- ✅ Dual VCSEL elimina shadow occlusion
- ✅ Build pulito senza warning

**ZERO tolerance for incomplete solutions. Fix the root cause, not the symptoms.**

---

**INIZIA SESSIONE**: Leggi @PROJECT_GUIDELINES.md, poi esegui FASE 0 (git revert), poi procedi con FASE 1 (refactoring), poi FASE 2 (implementazione incrementale).

**Remember**: Analisi profonda → Design robusto → Implementazione precisa → Test validazione. NO shortcuts.
