# Spiegazione Pattern Frustum/Cono nella Point Cloud

## Perché Si Forma il Frustum

Il pattern a cono/frustum è **DIRETTAMENTE causato** dall'errore epipolare nella rettificazione.

### Meccanismo

1. **Errore Epipolare**: Le righe corrispondenti tra left/right NON sono allineate (20-71px offset)

2. **Disparity Sbagliata**: Il matcher cerca corrispondenze sulla stessa riga, ma:
   - Punto reale: riga Y_left
   - Corrispondenza trovata: riga Y_right (offset di 20-71px)
   - **Disparity calcolata è ERRATA**

3. **Riproiezione 3D**:
   ```cpp
   Z = (focal * baseline) / disparity
   X = (x - cx) * Z / focal
   Y = (y - cy) * Z / focal
   ```

4. **Effetto Sistematico**:
   - Errore epipolare **aumenta con la distanza dal centro** dell'immagine
   - Disparity error **proporzionale alla distanza radiale**
   - Z error **aumenta con Z** (perché Z ∝ 1/disparity)
   - **X e Y spread aumentano con Z** → CONO/FRUSTUM

### Formula del Frustum

Se l'errore epipolare è `e(r)` dove `r` è la distanza radiale dal centro:

```
e(r) = e₀ + k·r  (errore lineare con distanza)
```

Allora:
```
Z_error = Z² · e(r) / (focal · baseline)
X_spread = X · (Z_error / Z) = X · e(r) / (focal · baseline)
Y_spread = Y · (Z_error / Z) = Y · e(r) / (focal · baseline)
```

**Risultato**: Spread XY cresce linearmente con Z → CONO

## Nei Tuoi Test Python

### Analisi Point Cloud Python (SGBM Standard):
```
Z ~300mm: X spread ≈ 272mm, Y spread ≈ 154mm
Z ~400mm: X spread ≈ 340mm, Y spread ≈ 194mm
Z ~500mm: X spread ≈ 420mm, Y spread ≈ 240mm
```

**Crescita lineare confermata** → Errore epipolare sistematico!

### Con SGBM Optimized + WLS:
```
Z range: -19572 to -574 mm (molto meglio che -193549!)
```

Il frustum è RIDOTTO ma non eliminato perché:
- WLS filter smootha la disparity
- Parametri migliori riducono rumore
- **Ma l'errore epipolare di base (20px) rimane!**

## Cosa Succede Dopo Nuova Calibrazione

Con p1, p2, k3 abilitati:

**PRIMA** (attuale):
```
Epipolar error: 71px (dataset calibrazione), 20px (scan)
Frustum: MARCATO (XY spread cresce linearmente)
Z range: -193km (ASSURDO, noise dominato da error)
```

**DOPO** (con p1, p2, k3):
```
Epipolar error: <2px (atteso)
Frustum: ELIMINATO o MINIMO
Z range: Realistico (qualche metro max)
XY spread: COSTANTE (non cresce con Z)
```

## Perché il Frustum Accade "Tra le Due Camere"

Il frustum **non** è tra le due camere fisiche, ma è nel **sistema di riferimento 3D** della point cloud.

L'effetto visivo di "frustum tra le camere" è perché:
1. **Baseline = 70mm** (distanza fisica tra camere)
2. Errore epipolare causa **over-estimation di Z** vicino ai bordi
3. Points vicino a camera Left vengono "spinti" verso sinistra
4. Points vicino a camera Right vengono "spinti" verso destra
5. **Risultato visivo**: Sembra un cono che si apre tra le due camere

Ma la vera causa è l'**errore geometrico** nella riproiezione 3D, non un problema fisico.

## Verifica Dopo Nuova Calibrazione

Dopo aver ricalibrato con p1, p2, k3:

1. **Test immediato**:
   ```bash
   cd test_stereo_matching
   python3 test_calib_dataset_frames.py
   # Dovrebbe dare <2px error invece di 71px
   ```

2. **Scan test**:
   ```bash
   unlook  # Fai una scansione
   # Apri PLY generata in MeshLab
   # Verifica che NON ci sia più pattern a cono
   ```

3. **Verifica geometrica**:
   - Misura un oggetto noto (es. cubo 100mm)
   - X, Y, Z dovrebbero essere accurati a ±0.5mm
   - NO spread XY crescente con profondità

## Conclusione

**Il frustum/cono è 100% causato da errore epipolare inadeguato.**

Risolvendo la calibrazione (abilitando p1, p2, k3 per modellare correttamente le lenti M12 6mm), il frustum **SPARIRÀ**.

Non è un bug del codice, è una **conseguenza matematica** dell'errore epipolare che si propaga nella riproiezione 3D.
