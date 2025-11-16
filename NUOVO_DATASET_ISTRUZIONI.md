# ISTRUZIONI CATTURA NUOVO DATASET (100 frames)

**Data:** 2025-11-15
**Obiettivo:** Calibrazione con coverage completa + simplified distortion model
**Target:** Epipolar error < 5px su ORB features (vs 58px attuale)

---

## ✅ MODIFICHE APPLICATE

### 1. CALIB_RATIONAL_MODEL rimosso ✓
```cpp
// PRIMA (14 parametri - OVERFITTING):
CALIB_RATIONAL_MODEL + k1-k6, p1-p2

// ADESSO (8 parametri - OPTIMAL):
k1, k2, k3 (radial polynomial)
p1, p2 (tangential)
NO k4-k6 (rational fraction)
```

### 2. Limite frames aumentato: 100 pairs ✓

### 3. Bug computeEpipolarErrors fixato ✓

---

## 📸 STRATEGIA CATTURA DATASET

### Setup Iniziale
- **Distanza base:** 100cm (1 metro)
- **Illuminazione:** LED flood attivo, uniforme, no ombre
- **Focus:** FISSO (no autofocus!)
- **Checkerboard:** 9x6 (54 corner interni), quadrati 24mm

### Distribuzione 100 Frames

**GRUPPO 1: CENTRO (30 frames) - Distanza 90-110cm**
```
Posizione: Checkerboard al CENTRO dell'immagine
Angolazioni:
- 10 frames: 0-10° (quasi flat)
- 10 frames: 10-20° (moderate tilt)
- 10 frames: 20-30° (marked rotation)

Variazioni: ruota in tutte le direzioni (roll, pitch, yaw)
```

**GRUPPO 2: BORDI ESTREMI (30 frames) - Distanza 90-110cm**
```
CRITICAL: Posiziona checkerboard MOLTO VICINO ai bordi!

- 8 frames: TOP edge (checkerboard in alto, quasi fuori view)
- 8 frames: BOTTOM edge (checkerboard in basso)
- 7 frames: LEFT edge (checkerboard a sinistra)
- 7 frames: RIGHT edge (checkerboard a destra)

Per ogni bordo:
- 3 frames: angolo 0-15°
- 3 frames: angolo 15-25°
- 2 frames: angolo 25-35°
```

**GRUPPO 3: ANGOLI (20 frames) - Distanza 90-110cm**
```
- 5 frames: TOP-LEFT corner (checkerboard nell'angolo alto-sinistra)
- 5 frames: TOP-RIGHT corner
- 5 frames: BOTTOM-LEFT corner
- 5 frames: BOTTOM-RIGHT corner

Per ogni angolo, varia:
- Distanza: 90cm, 100cm, 110cm
- Angolazione: 0-30°
```

**GRUPPO 4: DISTANZE VARIE (20 frames)**
```
Coverage completa a distanze diverse!

- 7 frames: 70-80cm (VICINO)
  - Centro + 2 bordi + 1 angolo

- 7 frames: 120-130cm (LONTANO)
  - Centro + 2 bordi + 1 angolo

- 6 frames: 60cm (MOLTO VICINO - edge coverage)
  - Solo bordi e angoli!
```

---

## ⚠️ REGOLE CRITICHE

### Durante Cattura:
1. ✅ **FERMA** checkerboard per 1-2 secondi prima del click
2. ✅ Verifica che **TUTTI i 54 corner siano VERDI** nel preview
3. ✅ Se anche UN SOLO corner è rosso → **NON catturare**
4. ✅ Muovi **LENTAMENTE** (no motion blur)
5. ✅ Tieni checkerboard **PIATTA** (no pieghe, no curvature)

### Setup Hardware:
- ✅ Camere fisse, mount rigido
- ✅ LED flood ON
- ✅ LED VCSEL OFF (per calibrazione)
- ✅ No autofocus, no auto white balance
- ✅ Exposure fisso o solo auto-exposure time

---

## 🎯 OBIETTIVO COVERAGE

**Cosa copriamo:**
```
+-----------------------------------+
|  TL    TOP edge         TR       |
|   ●------●------●------●         |
|   |                     |         |
| LEFT                  RIGHT      |
|   |                     |         |
|   ●------●------●------●         |
|  BL   BOTTOM edge       BR       |
+-----------------------------------+

● = Angoli (20 frames)
━ = Bordi (30 frames)
Centro = 30 frames
Distanze varie = 20 frames
TOTAL = 100 frames
```

**Perché è CRITICO:**
- OpenCV richiede checkerboard INTERA visibile
- Quindi NON può calibrare i bordi se checkerboard sempre al centro
- Con coverage ai bordi → calibriamo TUTTO il field of view
- Distortion model non deve extrapolate → ha dati osservati ovunque!

---

## 📊 ASPETTATIVE POST-CALIBRAZIONE

**Con simplified model (8 params) + 100 frames coverage:**

```
METRICHE ATTESE:

RMS reprojection error:  < 0.4px  (attuale: 0.46px)
Epipolar (checkerboard): < 0.3px  (attuale: 0.26px - già OK!)
Epipolar (ORB features): < 5px    (attuale: 58px - CRITICAL!)

Baseline:                69.5-70.5mm (attuale: 69.77mm - OK!)
```

**Improvement Factor Atteso:**
- Da 58px a 2-5px = **10-30x miglioramento!**

**Se ancora problemi:**
- Tentativo B: ChArUco board (pattern detection più robusta)
- Tentativo C: Fisheye model (se FOV è sottostimato)

---

## 🔬 VALIDATION POST-CALIBRAZIONE

Dopo la calibrazione, testeremo con:

```bash
cd /home/alessandro/unlook-standalone/test_stereo_matching

# Update script per nuova calibrazione
python3 test_calib_dataset_frames.py

# Expected output:
# Average error: < 5px (vs 58px attuale)
# If > 10px → dataset quality issue
# If 5-10px → good but can improve
# If < 5px → EXCELLENT!
```

**CRITICAL VALIDATION:**
- Se epipolar error su checkerboard < 1px MA su ORB > 10px
  → Ancora overfitting (coverage insufficiente o angoli troppo estremi)
- Se entrambi < 5px → **SUCCESS!** ✓

---

## 📋 CHECKLIST PRE-CATTURA

- [ ] LED flood ON, VCSEL OFF
- [ ] Camere fisse, no movimento mount
- [ ] Checkerboard piatta su superficie rigida
- [ ] Distanza iniziale 100cm misurata
- [ ] Illuminazione uniforme, no ombre
- [ ] Focus fisso verificato
- [ ] GUI mostra 100 frame target (non 50)
- [ ] Test capture 1 frame → verifica tutti 54 corner verdi

---

## 🚀 PROCEDURA CATTURA

1. **Avvia GUI:** `unlook`
2. **Tab "Dataset Capture"**
3. **Verifica setup:** preview mostra entrambe camere OK
4. **Click "Start Dataset Capture (100 pairs)"**
5. **Countdown 5 secondi** → posiziona checkerboard
6. **Segui distribuzione sopra**:
   - Prima 30 frames centro
   - Poi 30 frames bordi
   - Poi 20 frames angoli
   - Infine 20 frames distanze varie
7. **Ogni capture:** verifica LED verde, wait 1-2 sec, steady hand
8. **Completati 100 frames** → auto-switch a "Process Dataset" tab
9. **Click "Process Calibration"**
10. **Wait ~2 minuti** (stereoCalibrate con 100 frames)
11. **Verifica YAML:**
    - RMS < 0.5px
    - Baseline 69.5-70.5mm
    - k4, k5, k6 = 0 (rational model disabled ✓)

---

## 💡 TIPS

**Se corner detection fallisce:**
- Avvicina/allontana leggermente checkerboard
- Cambia angolo di pochi gradi
- Verifica illuminazione uniforme (no riflessi!)

**Se troppi frame scartati (blur):**
- Muovi PIÙ LENTAMENTE
- Aspetta 2-3 secondi con checkerboard ferma prima di click

**Se calibrazione fallisce:**
1. Verifica dataset: almeno 60 frames validi
2. Controlla distribuzione: coperto centro + bordi + angoli?
3. Verifica angoli: nessuno >40°?
4. Distanze: variazione 60-130cm coperta?

---

## 📚 RIFERIMENTI

- **Research completo:** `/home/alessandro/unlook-standalone/CALIBRATION_RESEARCH_FINDINGS.md`
- **GitHub Issue OpenCV:** #15992 - "calibrateCamera does not enforce lens model constraints"
- **Commit fix:** bc73461 - "Remove CALIB_RATIONAL_MODEL"

---

**Pronto? Let's capture! 🎯**

**Expected time:** 15-20 minuti cattura + 2 minuti calibrazione = **~20 minuti total**

Good luck! 🚀
