# 🎯 Guida Oggetti per Testing Stereo (Ambient Light)

**Data**: 2025-10-25
**Context**: Testing stereo vision SENZA VCSEL (solo luce ambiente naturale)
**Requirement**: Oggetti con buona texture per matching robusto

---

## ✅ OGGETTI ECCELLENTI per Stereo Matching

### 1. **Tastiera** 🎹 (PERFETTA!)
**Texture Score**: ⭐⭐⭐⭐⭐ (5/5)

**Perché eccellente**:
- Tasti con bordi netti e ben definiti
- Pattern ripetitivo ma distinguibile (lettere diverse)
- Contrasto elevato (tasti scuri/chiari su sfondo)
- Alta frequenza di dettagli (lettere, simboli)
- Superfici planari (facili da calibrare/misurare)

**Best Practice**:
- Usa tastiera con **tasti stampati** (non blank keyboard)
- Illuminazione laterale per enfatizzare i tasti
- Angolazione 30-45° (non frontale) per vedere i lati dei tasti

**Esempio**:
```
[Q] [W] [E] [R] [T] [Y]
[A] [S] [D] [F] [G] [H]
     [SPACE]
```
Ogni tasto è un feature distintivo!

---

### 2. **Libri con Testo** 📚
**Texture Score**: ⭐⭐⭐⭐⭐ (5/5)

**Perché eccellente**:
- Testo stampato = texture ad altissima frequenza
- Ogni lettera è un feature unico
- Bordi di pagine ben definiti
- Copertina spesso texturizzata

**Best Practice**:
- Libro aperto (2 pagine visibili)
- Font grande (>12pt) per migliore matching
- Buona illuminazione diffusa (no ombre dure)

---

### 3. **Tessuti Stampati** 👕
**Texture Score**: ⭐⭐⭐⭐ (4/5)

**Perché buono**:
- T-shirt con grafica/logo
- Coperte con pattern
- Tappeti con disegni
- Texture non ripetitiva = feature matching facile

**Best Practice**:
- Evita tessuti monocromatici
- Preferisci pattern geometrici o grafiche complesse
- Superficie tesa (non piegata troppo)

---

### 4. **Oggetti con Scritte/Etichette** 🏷️
**Texture Score**: ⭐⭐⭐⭐⭐ (5/5)

**Esempi**:
- Scatole di cereali (piene di testo/immagini)
- Bottiglie con etichette
- Pacchi Amazon con codici a barre
- Riviste/quotidiani

**Perché eccellente**:
- Testo = feature distintive infinite
- Codici a barre = pattern ad alta frequenza
- Loghi colorati = contrasto elevato

---

### 5. **Piante/Foglie** 🌿
**Texture Score**: ⭐⭐⭐⭐ (4/5)

**Perché buono**:
- Nervature foglie = texture naturale
- Bordi irregolari ma ben definiti
- Variazioni di colore (verde chiaro/scuro)

**Best Practice**:
- Illuminazione da dietro per enfatizzare nervature
- Foglie grandi (>10cm)
- Evita piante troppo fitte (difficili da separare)

---

### 6. **Circuiti Stampati/PCB** 🔌
**Texture Score**: ⭐⭐⭐⭐⭐ (5/5)

**Perché eccellente**:
- Densità altissima di feature
- Componenti elettronici con forme diverse
- Tracce dorate/argentate su sfondo verde
- Pattern non ripetitivo

**Best Practice**:
- PCB assemblato (con componenti)
- Illuminazione diffusa (no riflessi metallici)

---

### 7. **Puzzle Assemblati** 🧩
**Texture Score**: ⭐⭐⭐⭐⭐ (5/5)

**Perché eccellente**:
- Ogni pezzo ha bordi unici
- Immagine stampata = texture variabile
- Giunzioni tra pezzi = feature geometriche

---

### 8. **Superfici Ruvide/Texturizzate** 🪨
**Texture Score**: ⭐⭐⭐⭐ (4/5)

**Esempi**:
- Mattoni/pietra
- Legno con venature
- Carta vetrata
- Superfici sabbiate

**Perché buono**:
- Texture naturale non uniforme
- Piccole variazioni di profondità
- Nessuna ripetizione perfetta

---

## ❌ OGGETTI DA EVITARE (Povera Texture)

### 1. **Superfici Uniformi** ❌
- Pareti bianche/colorate
- Plastica liscia monocromatica
- Metallo lucido
- Vetro/specchi

**Problema**: Nessuna texture → SGBM non trova corrispondenze

---

### 2. **Pattern Ripetitivi** ⚠️
- Piastrelle identiche
- Griglia metallica regolare
- Tessuti a righe uniformi

**Problema**: Ambiguità nel matching (tutte le righe sembrano uguali)

---

### 3. **Oggetti Troppo Riflettenti** ❌
- Metallo lucido/cromato
- Plastica lucida
- Superfici vetrate

**Problema**: Riflessi cambiano con viewpoint → matching fallisce

---

### 4. **Oggetti Troppo Piccoli** ⚠️
- Oggetti <5cm
- Dettagli <2mm

**Problema**: Risoluzione insufficiente per matching affidabile

---

## 🎯 OGGETTI CONSIGLIATI PER DEMO INVESTITORI

### Top 3 per Impressionare:

#### 1. **Tastiera Meccanica RGB** (se disponibile)
- Tasti retroilluminati + texture lettere
- Mostra capacità stereo in condizioni difficili (illuminazione variabile)
- Oggetto riconoscibile e "tech"

#### 2. **Scatola di Prodotto Commerciale** (es. iPhone/SmartPhone Box)
- Logo Apple/brand noto
- Testo multilingua
- Superfici lucide + opache (sfida)
- Riconoscibile = impatto visivo

#### 3. **Libro Aperto con Immagini**
- Testo + illustrazioni
- Mostra versatilità (pattern misti)
- Profondità variabile (pagine curve)

---

## 📋 CHECKLIST PRE-SCANSIONE

Prima di scansionare, verifica:

- [ ] **Illuminazione**: Diffusa, no ombre dure
- [ ] **Distanza**: 300-800mm dal scanner
- [ ] **Angolazione**: 30-45° (non perfettamente frontale)
- [ ] **Stabilità**: Oggetto fermo (no movimento durante capture)
- [ ] **Background**: Neutro, diverso dall'oggetto
- [ ] **Texture visibile**: Controlla che dettagli siano visibili ad occhio

---

## 🧪 PROTOCOLLO DI TESTING

### Test 1: Baseline Validation (Tastiera)
```bash
# 1. Posiziona tastiera a 500mm
# 2. Illuminazione diffusa (lampada da tavolo)
# 3. Angolazione 45°
# 4. Checkbox "Ambient Light Mode" SPUNTATO
# 5. Capture
# 6. Verifica disparity coverage >60% in status log
```

**Success Criteria**:
- Valid pixel ratio >60%
- Median disparity >0 (non near-zero)
- Point cloud >400K points

---

### Test 2: Challenge Test (Scatola Lucida)
```bash
# 1. Scatola prodotto commerciale (iPhone box, ecc.)
# 2. Illuminazione laterale (evita riflessi)
# 3. Angolazione variabile (testa 30°, 45°, 60°)
# 4. Ambient Light Mode SPUNTATO
# 5. Multiple capture da angolazioni diverse
```

**Success Criteria**:
- Almeno 2 angolazioni con coverage >50%
- Dettagli loghi/testi riconoscibili in point cloud

---

### Test 3: Precision Validation (Oggetto Noto)
```bash
# 1. Oggetto con dimensioni note (es. cubo 100x100x100mm)
# 2. Distanza fissa 500mm
# 3. Capture + Export PLY
# 4. Misura in MeshLab/CloudCompare
```

**Success Criteria**:
- Dimensioni misurate: ±1-2mm (acceptable per calibration RMS=0.242px)
- Se error >3mm → recalibration necessaria

---

## 💡 TIPS & TRICKS

### Migliorare Coverage:

1. **Illuminazione ottimale**:
   - LED diffuso (no spotlight)
   - Angolazione 30-45° dall'alto
   - Evita ombre dure

2. **Oggetto positioning**:
   - Centratura nel field of view
   - Distanza 400-600mm (sweet spot per 70mm baseline)
   - Angolazione 30-45° (non frontale)

3. **Fine-tuning parametri**:
   - Se coverage bassa → `uniquenessRatio = 15` (meno strict)
   - Se troppo rumore → `speckleWindowSize = 150`
   - Se bordi troppo smooth → `P2 = 1176` (P2/P1 = 3)

---

## 🚀 QUICK START

**Per testing rapido OGGI**:

1. ✅ Prendi **tastiera** (oggetto #1)
2. ✅ Posiziona a **500mm** dal scanner
3. ✅ Illumina con **lampada da tavolo** (luce diffusa)
4. ✅ Angolazione **45°** (non frontale)
5. ✅ Build e run GUI:
   ```bash
   ./build.sh
   unlook
   ```
6. ✅ Nella GUI Depth tab:
   - Spunta **"Ambient Light Mode (NO VCSEL)"**
   - Click **"Capture Stereo Frame"**
   - Controlla **status log** per coverage %
   - Export **point cloud PLY**

7. ✅ Verifica in **MeshLab**:
   ```bash
   meshlab /tmp/unlook_pointcloud_*.ply
   ```

**Expected result**: Point cloud con tasti chiaramente riconoscibili, >400K punti

---

## 📊 TABELLA COMPARATIVA OGGETTI

| Oggetto | Texture Score | Difficulty | Demo Impact | Recommended |
|---------|---------------|------------|-------------|-------------|
| Tastiera | ⭐⭐⭐⭐⭐ | Easy | High | ✅ YES |
| Libro aperto | ⭐⭐⭐⭐⭐ | Easy | Medium | ✅ YES |
| Scatola iPhone | ⭐⭐⭐⭐ | Medium | Very High | ✅ YES |
| T-shirt grafica | ⭐⭐⭐⭐ | Easy | Medium | ⚠️ Maybe |
| PCB | ⭐⭐⭐⭐⭐ | Easy | High (tech) | ✅ YES |
| Pianta | ⭐⭐⭐⭐ | Medium | Low | ⚠️ Maybe |
| Parete bianca | ⭐ | Impossible | N/A | ❌ NO |
| Metallo lucido | ⭐ | Very Hard | N/A | ❌ NO |

---

**🎯 TL;DR**: Usa **tastiera** per primo test. Se funziona bene, testa **scatola iPhone** per demo investitori. Evita superfici uniformi/lucide.
