# ML Depth Refinement Research - Unlook 3D Scanner

**Data ricerca**: 8 Ottobre 2025
**Obiettivo**: Migliorare depth estimation per thin objects (dita, edges) usando ML

---

## Problema Identificato

### Sintomi
- Palmo della mano: depth corretta ✅
- Dita: Z values errati che "vanno verso il fondo" ❌
- Cause: WLS filter aggressivo, uniqueness ratio alto, edge fattening, mancanza texture VCSEL sulle dita

### Depth Map Issues
- **Disparity**: Palmo rosso (vicino) → Dita verde/cyan (interpretate come lontane)
- **Root cause**: SGBM stereo matching fallisce su thin structures con low texture

---

## Ricerca ML - Stato dell'Arte 2024-2025

### 1. Neural Disparity Refinement (NDR)
**Paper**: "Neural Disparity Refinement" - IEEE Transactions on Pattern Analysis and Machine Intelligence (TPAMI) 2024

**Link**:
- https://dl.acm.org/doi/10.1109/TPAMI.2024.3411292
- https://www.computer.org/csdl/journal/tp/2024/12/10552115/1XApeqgJWjC

**Concetto**:
- Framework ibrido che combina algoritmi tradizionali (SGBM) con deep learning
- Post-processing neurale su disparity maps per migliorare qualità e risoluzione
- Specificamente progettato per refinement di mappe generate da metodi classici

**Performance**:
- Inference speed: ~3 FPS su NVIDIA RTX 3060 GPU
- Model size: Non specificato nel paper
- Migliora significativamente edge e thin objects

**Limitazioni per Unlook**:
- ❌ Troppo pesante per real-time su CM5 (3 FPS su GPU desktop)
- ❌ Richiede GPU/acceleratore per inference
- ⚠️ Non ottimizzato per embedded deployment

### 2. Hybrid SGBM + Lightweight Neural Networks
**Paper**: "A two-stage fast stereo matching algorithm for real-time 3D coordinate computation" - ScienceDirect 2025

**Concetto**:
- Two-stage approach: SGBM initial estimate + neural refinement
- Lightweight neural models for edge devices
- Balance tra accuracy e computational efficiency

**Performance**:
- Speed-up: 13-56.9x rispetto a SGBM tradizionale
- Mantiene accuracy comparabile
- Design specifico per real-time embedded systems

**Vantaggi per Unlook**:
- ✅ Progettato per sistemi embedded
- ✅ Mantiene SGBM come base (già implementato)
- ✅ Lightweight = possibile deployment su CM5

**Limitazioni**:
- ⚠️ Non ancora validato su Raspberry Pi CM5
- ⚠️ Richiede implementazione custom
- ⚠️ Training dataset necessario

### 3. TensorFlow Lite on Raspberry Pi 5/CM5
**Status**: Production-ready (2024-2025)

**Performance**:
- TensorFlow Lite 2.15.0 funziona nativamente su Pi 5/CM5
- Performance simile a Coral TPU accelerator
- Support per modelli di object detection e classification

**Benchmarks**:
- Object detection: 10-20 FPS su Pi 5
- Lightweight models (3-5M params): 15-30 FPS
- Model size limit raccomandato: <10MB

**Applicabilità Unlook**:
- ✅ Platform ready per deployment
- ✅ Performance sufficienti per post-processing (~10-20 FPS target)
- ❌ Nessun modello pre-trained per depth refinement disponibile

### 4. Foundation Models for Depth Estimation
**Research**: "Towards Depth Foundation Model" - arXiv 2025

**Trend emergente**:
- Foundation models: 3.5M - 11M parameters
- End-to-end depth estimation (senza SGBM)
- Transfer learning da grandi dataset

**Limitazioni**:
- ❌ Troppo grandi per CM5 real-time
- ❌ Richiedono GPU potente
- ⚠️ Non adatti per handheld scanning

---

## Opzioni di Implementazione per Unlook

### OPZIONE A: Non-ML Parameter Tuning (IMMEDIATE)
**Timeline**: 1-3 giorni
**Effort**: Basso

**Implementazione**:
1. Ridurre WLS Lambda: 9000 → 6000-7000
2. Ridurre uniqueness ratio: 30 → 20-25
3. Disabilitare/ridurre edge filtering per thin objects
4. Adaptive parameters per regioni (dita vs palmo)

**Pro**:
- ✅ Zero overhead computazionale
- ✅ Implementazione immediata
- ✅ No training/dataset richiesto
- ✅ Completamente deterministico

**Contro**:
- ⚠️ Miglioramento limitato (~20-30% stimato)
- ⚠️ Non risolve problema fondamentale texture

**Costo**: €0, 1-3 giorni sviluppo

---

### OPZIONE B: TensorFlow Lite Neural Disparity Refinement (FUTURO)
**Nome progetto**: **"Unlook Neural Depth Refiner (UNDR)"**

**Timeline**: 2-3 mesi
**Effort**: Alto

**Architecture Proposta**:
```
Input: SGBM disparity map (1456x1088)
       Left/Right images (grayscale)

Network: Lightweight U-Net style
         - Encoder: 3-4 layers (feature extraction)
         - Bottleneck: 512-1024 features
         - Decoder: 3-4 layers (refinement)
         - Skip connections per preservare details

Output: Refined disparity map (1456x1088)

Parameters: 3-5M (target per TFLite CM5)
Inference: 10-20 FPS target su CM5
Model size: <8MB quantized INT8
```

**Fasi Implementazione**:

1. **Dataset Collection** (2-3 settimane)
   - Raccogliere 500-1000 scans diversi oggetti
   - Ground truth da scanner professionale o manual labeling
   - Augmentation: rotazioni, lighting, noise
   - Split: 80% training, 10% validation, 10% test

2. **Model Training** (2-3 settimane)
   - Framework: TensorFlow/Keras
   - Loss function: L1 + edge-aware loss
   - Training su GPU workstation
   - Hyperparameter tuning
   - Validation su test set

3. **Model Optimization** (1 settimana)
   - Post-training quantization (INT8)
   - TensorFlow Lite conversion
   - Model pruning se necessario
   - Performance benchmarking

4. **CM5 Integration** (1-2 settimane)
   - TFLite runtime integration
   - Pipeline: SGBM → Neural Refiner → Point Cloud
   - Real-time performance testing
   - Memory optimization

**Dataset Requirements**:
- Minimum: 500 scan pairs (SGBM + ground truth)
- Ideal: 1000-2000 scan pairs
- Variety: mani, oggetti industriali, superfici smooth/textured
- Annotations: disparity ground truth

**Hardware Requirements**:
- Training: GPU workstation (RTX 3060+, 16GB+ VRAM)
- Inference: Raspberry Pi CM5 (già disponibile)

**Pro**:
- ✅ Significativo miglioramento edge/thin objects (50-80% stimato)
- ✅ Generalizza a nuovi oggetti (se training diversificato)
- ✅ Real-time feasible su CM5 (10-20 FPS)
- ✅ Scalabile e migliorabile con più dati

**Contro**:
- ❌ Richiede 2-3 mesi sviluppo full-time
- ❌ Dataset collection effort significativo
- ❌ Richiede GPU workstation per training
- ⚠️ Rischio overfitting se dataset limitato
- ⚠️ +10-15ms latency per frame

**Costo Stimato**:
- GPU Workstation: €1000-2000 (una tantum)
- Sviluppo: 2-3 mesi ingegnere ML
- Dataset labeling: 50-100 ore
- Cloud GPU (alternativa): €200-500 per training

**Performance Target**:
- Inference time: 50-100ms per frame (10-20 FPS)
- Memory: <200MB
- Accuracy improvement: 50-80% su thin objects
- Real-time scanning: 10 FPS with refinement

**Alternative Approcci**:

1. **Transfer Learning**
   - Base model: Pre-trained depth estimation (MiDaS, DPT)
   - Fine-tune su Unlook dataset
   - Pro: Meno dati richiesti
   - Contro: Model size potenzialmente troppo grande

2. **Knowledge Distillation**
   - Teacher: Large model (NDR, foundation model)
   - Student: Lightweight per CM5
   - Pro: Migliore accuracy
   - Contro: Richiede teacher model + training più complesso

3. **Ensemble Methods**
   - Combinare SGBM con 2-3 lightweight models
   - Voting/averaging per robustezza
   - Pro: Più robusto
   - Contro: Latency aumentata

**Recommended Approach**: **Single lightweight U-Net con skip connections**
- Balance ottimale accuracy/speed/size
- Provata efficacia in letteratura
- Feasible per CM5 real-time

---

### OPZIONE C: Dual VCSEL Hardware (IMMEDIATE)
**Timeline**: Dipende da fix hardware
**Effort**: Hardware debugging

**Implementazione**:
1. Debug LED2 hardware issue (attualmente non illumina)
2. Attivare entrambi VCSEL simultaneamente
3. Pattern density raddoppiata

**Pro**:
- ✅ Boost immediato se hardware risolto
- ✅ Zero overhead computazionale
- ✅ Migliora matching su tutta la superficie

**Contro**:
- ⚠️ Dipende da fix hardware
- ⚠️ LED2 potrebbe avere problema fisico

**Status**: LED2 connesso a flood pins ma non illumina (problema HW)

---

## Raccomandazioni per Unlook

### Short-term (Ora - 1 settimana)
1. **Implementare Opzione A** (parameter tuning)
2. **Debuggare LED2** per Opzione C
3. **Testare e validare** miglioramenti

### Medium-term (1-2 mesi)
1. **Raccogliere dataset** per Opzione B
2. **Prototipare UNDR** su GPU workstation
3. **Benchmark TFLite** su CM5 con model dummy

### Long-term (3-6 mesi)
1. **Deploy UNDR production-ready**
2. **Real-time handheld scanning** (10 FPS target)
3. **User-friendly** per utenti non-tecnici

---

## References

### Papers
1. **Neural Disparity Refinement** - IEEE TPAMI 2024
   - https://dl.acm.org/doi/10.1109/TPAMI.2024.3411292

2. **Two-stage fast stereo matching** - ScienceDirect 2025
   - https://www.sciencedirect.com/science/article/abs/pii/S0263224125000314

3. **Awesome Deep Stereo Matching** - GitHub Curated List
   - https://github.com/fabiotosi92/Awesome-Deep-Stereo-Matching

### Implementations
1. **TensorFlow Lite on Raspberry Pi 5**
   - https://learn.adafruit.com/running-tensorflow-lite-on-the-raspberry-pi-4/overview

2. **StereoPi Depth Mapping**
   - https://stereopi.com/blog/opencv-and-depth-map-stereopi-tutorial

3. **OpenCV Stereo Vision Tutorial**
   - https://learnopencv.com/depth-perception-using-stereo-camera-python-c/

---

## Next Steps

Quando pronti per **OPZIONE B (UNDR)**:

1. **Setup Development Environment**
   - GPU workstation o cloud GPU
   - TensorFlow 2.15+, TFLite tools
   - Dataset storage (100-200GB)

2. **Dataset Strategy**
   - Decidere: manual labeling vs ground truth scanner
   - Pipeline automatica di collection
   - Quality control

3. **Model Architecture**
   - Prototipare su PyTorch/TensorFlow
   - Iterare su architecture
   - Validare su test set

4. **Production Deployment**
   - TFLite conversion e optimization
   - CM5 integration testing
   - Performance profiling

**Point of Contact**: Riferimento a questo documento per iniziare fase ML

---

**Status**: ✅ Research completata, pronto per decisione strategica
**Next Action**: Implementare Opzione A + C (immediate fixes)
**Future**: Opzione B (UNDR) quando sistema stabile e dataset pronto
