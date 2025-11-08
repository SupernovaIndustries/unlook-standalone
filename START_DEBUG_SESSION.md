# 🚨 AD-CENSUS STEREO MATCHING - CRITICAL DEBUG SESSION

## 📍 CONTEXT

Progetto: **Unlook 3D Scanner** (professional handheld 3D scanner, target 0.005mm precision)
Repository: `/home/alessandro/unlook-standalone`
Sistema: Raspberry Pi CM5, 2x IMX296 cameras (1280x720), baseline 70mm

## ⚠️ PROBLEMA CRITICO

**DEMO INVESTITORE TRA POCHE ORE** - Scansione 3D non funziona correttamente!

**Sintomi:**
1. ❌ **PLY completamente PIATTA** - nessuna geometria 3D
2. ❌ **Disparity maps con FORME GEOMETRICHE CASUALI** (cerchi/triangoli) invece di oggetti reali
3. ❌ **Depth map UNIFORME** (valore 169.9 costante su tutta la mano)
4. ✅ Log dice "Valid: 96-97%" MA risultati visivi SBAGLIATI
5. ✅ Rettificazione epipolare CORRETTA (verificata)

## 📄 DOCUMENTAZIONE COMPLETA

**LEGGI ATTENTAMENTE:**
`/home/alessandro/unlook-standalone/CRITICAL_AD_CENSUS_DEBUG.md`

Questo file contiene:
- Descrizione dettagliata di TUTTI i problemi
- Contesto implementazione (basata su MEGA_PROMPT_AD_CENSUS_HANDHELD.md)
- 6 STEP di diagnosi richiesta
- Bug comuni da cercare nel codice
- Reference implementations (libSGM, opencv_contrib)
- Deliverables per demo
- Fallback plan (cv::StereoSGBM standard)

## 🎯 TASK

**ANALIZZA E RISOLVI** i problemi AD-Census seguendo le istruzioni in `CRITICAL_AD_CENSUS_DEBUG.md`.

**FOCUS PRIORITARIO:**
1. Analizza `/home/alessandro/unlook-standalone/src/stereo/VCSELStereoMatcher.cpp` linea per linea
2. Cerca bug in: Census transform, Hamming distance, SGM aggregation, Disparity selection
3. Verifica calibrazione `/unlook_calib/default.yaml` (aggiornata 8 Nov 01:48, validata MATLAB)
4. Testa fix con scansione reale

**DEADLINE:** Poche ore per demo investitore

**SUCCESS CRITERIA:**
- Disparity map mostra VERA geometria oggetto (non forme casuali)
- Depth map ha variazione 3D (non piatta)
- PLY riconoscibile (anche se non precision perfetta)

## 🚀 START NOW

Leggi `CRITICAL_AD_CENSUS_DEBUG.md` e inizia la diagnosi.
