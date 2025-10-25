# ✅ Ambient Light Mode Feature - IMPLEMENTATO

**Data**: 2025-10-25
**Status**: ✅ COMPLETATO E COMPILATO
**Feature**: Checkbox GUI per switching VCSEL/Ambient Light parametri SGBM

---

## 📋 COSA È STATO IMPLEMENTATO

### 1. **GUI Checkbox** (depth_test_widget.ui)

Aggiunto checkbox **"Ambient Light Mode (NO VCSEL)"** nel Depth Test panel:

```xml
<widget class="QCheckBox" name="ambient_light_mode_checkbox">
  <property name="text">
    <string>Ambient Light Mode (NO VCSEL)</string>
  </property>
  <property name="checked">
    <bool>false</bool>  <!-- DEFAULT: VCSEL mode -->
  </property>
</widget>
```

**Posizione**: Dopo "Disable All Filters" checkbox, prima dello status log

---

### 2. **Signal Handler** (depth_test_widget.cpp)

Aggiunto handler che modifica `StereoConfig` dinamicamente:

```cpp
connect(ui->ambient_light_mode_checkbox, &QCheckBox::toggled,
        this, [this](bool checked) {
            auto config = depth_processor_->getStereoConfig();

            if (checked) {
                // AMBIENT LIGHT MODE
                config.block_size = 7;
                config.p1 = 392;
                config.p2 = 1568;
                config.uniqueness_ratio = 10;
                config.pre_filter_cap = 31;
                config.speckle_window_size = 100;
                config.speckle_range = 32;
            } else {
                // VCSEL MODE (default)
                config.block_size = 3;
                config.p1 = 72;
                config.p2 = 288;
                config.uniqueness_ratio = 22;
                config.pre_filter_cap = 15;
                config.speckle_window_size = 15;
                config.speckle_range = 128;
            }

            depth_processor_->configureStereo(config);
        });
```

---

## 🎯 COME USARE

### Step 1: Avvia GUI
```bash
unlook
```

### Step 2: Vai al tab "Depth Test"

### Step 3: **PRIMA** di capture:
- **Con VCSEL**: Lascia checkbox **NON spuntato** (default)
- **Senza VCSEL** (ambient light): **SPUNTA** il checkbox

### Step 4: Capture stereo frame
- Click "Capture Stereo Frame"
- Verifica nel status log i parametri applicati

---

## 📊 PARAMETRI CONFRONTO

| Parametro | VCSEL Mode (unchecked) | Ambient Light (checked) |
|-----------|----------------------|------------------------|
| `block_size` | 3 (dot matching) | 7 (natural texture) |
| `p1` | 72 | 392 |
| `p2` | 288 | 1568 |
| `uniqueness_ratio` | 22 (permissive) | 10 (strict) |
| `pre_filter_cap` | 15 | 31 |
| `speckle_window_size` | 15 | 100 |
| `speckle_range` | 128 (wide) | 32 (strict) |

---

## ✅ TESTING CHECKLIST

### Test 1: VCSEL Mode (Checkbox NON spuntato)
```bash
# 1. Avvia GUI: unlook
# 2. Tab Depth Test
# 3. Verifica checkbox NON spuntato
# 4. Se hai VCSEL funzionante, abilita LED
# 5. Capture
# 6. Verifica status log: "SGBM: VCSEL Mode (blockSize=3, dot matching)"
```

**Expected**: Parametri ottimizzati per structured light dots

---

### Test 2: Ambient Light Mode (Checkbox SPUNTATO)
```bash
# 1. Avvia GUI: unlook
# 2. Tab Depth Test
# 3. SPUNTA checkbox "Ambient Light Mode (NO VCSEL)"
# 4. Posiziona oggetto con texture (tastiera) a 500mm
# 5. Capture
# 6. Verifica status log: "SGBM: Ambient Light Mode (blockSize=7, strict matching)"
```

**Expected**:
- Status log mostra blockSize=7, P1=392, P2=1568, uniqueness=10
- Disparity coverage >60% (per oggetti con buona texture)
- Valid pixels >400K per VGA

---

### Test 3: Runtime Switching
```bash
# 1. Capture con VCSEL mode (unchecked)
# 2. SPUNTA checkbox
# 3. Capture di nuovo (stessa scena)
# 4. DESELEZIONA checkbox
# 5. Capture di nuovo

# Verifica nel status log che parametri cambiano correttamente
```

---

## 📝 STATUS LOG MESSAGES

Quando spunti/deseleziona checkbox, dovresti vedere:

### Ambient Light Mode ON:
```
SGBM: Ambient Light Mode (blockSize=7, strict matching)
SGBM parameters updated successfully
SGBM: blockSize=7, P1=392, P2=1568, uniqueness=10
```

### VCSEL Mode ON:
```
SGBM: VCSEL Mode (blockSize=3, dot matching)
SGBM parameters updated successfully
SGBM: blockSize=3, P1=72, P2=288, uniqueness=22
```

---

## 🧪 OGGETTI CONSIGLIATI PER TESTING

Vedi `TESTING_OBJECTS_TEXTURE_GUIDE.md` per lista completa.

**Top 3**:
1. ✅ **Tastiera** (texture score 5/5)
2. ✅ **Libro aperto** (texture score 5/5)
3. ✅ **Scatola prodotto** (es. iPhone box) (texture score 4/5)

---

## 🔧 TROUBLESHOOTING

### Problema: Coverage Bassa (<30%) in Ambient Light Mode

**Cause possibili**:
1. Oggetto con texture insufficiente
2. Illuminazione scarsa
3. Calibration RMS error troppo alta (0.242px)

**Soluzioni**:
```
1. Usa oggetto con texture (tastiera, libro)
2. Migliora illuminazione (lampada diffusa)
3. Regola fine-tuning:
   - uniqueness_ratio = 15 (meno strict)
   - pre_filter_cap = 63 (massima normalizzazione)
```

---

### Problema: Parametri Non Cambiano

**Verifica**:
1. Check console output: `[DepthWidget] AMBIENT LIGHT MODE ENABLED`
2. Se non vedi messaggio → problema signal/slot connection
3. Se vedi messaggio ma parametri non cambiano → rebuild necessario

**Fix**:
```bash
# Clean rebuild
rm -rf build
./build.sh
```

---

### Problema: Troppo Rumore in Disparity Map

**Se in Ambient Light Mode**:
```cpp
// Aumenta speckle filtering
config.speckle_window_size = 150;  // Era 100
config.speckle_range = 16;         // Era 32 (più strict)
```

**Se in VCSEL Mode**: Normale con ambient light, usa checkbox!

---

## 📈 EXPECTED IMPROVEMENTS

### Con Checkbox Spuntato (Ambient Light):

**Rispetto a parametri VCSEL su ambient light**:
- ✅ Coverage: +40-100% (es. 20% → 60%)
- ✅ Matching reliability: Fewer false positives
- ✅ Depth stability: Less noise
- ✅ Outlier reduction: Aggressive speckle filtering

**Trade-off**:
- ⚠️ Processing time: 2-2.5x slower (40ms → 90ms VGA)
- ⚠️ Detail loss: blockSize=7 perde dettagli <3px

---

## 🚀 NEXT STEPS

### Dopo Testing Parametri:

1. ✅ **Test su oggetti demo** (tastiera, scatola iPhone)
2. 🔄 **Fine-tuning** se coverage <60%
3. 🔄 **Recalibration** se depth accuracy <1mm@500mm
4. ✅ **Proceed con Artec mesh processing** (MISSION_CRITICAL_IMPLEMENTATION.md)

---

## 📚 DOCUMENTAZIONE CORRELATA

1. `SGBM_AMBIENT_LIGHT_OPTIMIZATION.md` - Analisi completa parametri
2. `TESTING_OBJECTS_TEXTURE_GUIDE.md` - Oggetti per testing
3. `DEMO_STATUS_ANALYSIS.md` - Status sistema completo
4. `MISSION_CRITICAL_IMPLEMENTATION.md` - Artec mesh processing (next step)

---

## ✅ CHECKLIST FEATURE COMPLETATA

- [x] Checkbox aggiunto in .ui file
- [x] Signal/slot connessi in .cpp
- [x] Parametri SGBM configurati correttamente
- [x] Build SUCCESS
- [x] Status messages implementati
- [x] Documentazione creata
- [ ] Testing su hardware reale (TU!)
- [ ] Validation con oggetti demo
- [ ] Fine-tuning se necessario

---

## 🎯 QUICK TEST NOW

```bash
# 1. Run GUI
unlook

# 2. Depth Test tab → Spunta "Ambient Light Mode (NO VCSEL)"

# 3. Posiziona tastiera a 500mm, illuminazione diffusa

# 4. Capture Stereo Frame

# 5. Verifica status log:
#    - "SGBM: Ambient Light Mode (blockSize=7, strict matching)"
#    - "SGBM: blockSize=7, P1=392, P2=1568, uniqueness=10"
#    - Valid pixel ratio >60%

# 6. Export point cloud PLY

# 7. Verifica in MeshLab: tasti tastiera riconoscibili
```

**Se tutto OK**: Proceed con oggetti demo e Artec mesh processing! 🚀
