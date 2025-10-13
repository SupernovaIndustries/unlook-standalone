# Dual AS1170 Configuration - VCSEL + Flood LEDs

## Proposed Hardware Configuration

### Current Setup (AS1170 #1)
- **I2C Bus**: 1
- **I2C Address**: 0x30
- **LED1**: VCSEL1 (upper, vertical mount)
- **LED2**: VCSEL2 (lower, vertical mount) - needs repair
- **Position**: Center between cameras, 2cm vertical separation

### Planned Addition (AS1170 #2)
- **I2C Bus**: 2
- **I2C Address**: 0x30
- **LED1**: Flood LED Left
- **LED2**: Flood LED Right
- **Position**: Center between cameras, 2cm horizontal separation
- **Purpose**: Texture illumination for low-light conditions

## Physical Layout

```
    TOP VIEW:

    Camera LEFT ←70mm→ Camera RIGHT
         |               |
         |    [VCSEL1]   |  ← Upper (vertical)
         |       ↕2cm    |
         |    [VCSEL2]   |  ← Lower (vertical)
         |               |
    [Flood L]←2cm→[Flood R]  ← Horizontal floods
         |               |
```

## Advantages of VCSEL + Flood Combination

### 1. **Complementary Illumination**
- **VCSEL**: 15K dots structured pattern for geometric stereo matching
- **Flood**: Uniform IR illumination for texture-based matching
- **Combined**: Robust depth extraction in all conditions

### 2. **Optimal for Different Surfaces**
| Surface Type | Best Illumination | Reason |
|-------------|------------------|---------|
| Textured | Flood | Natural features visible |
| Uniform/Smooth | VCSEL | Dots provide artificial texture |
| Dark/Black | Flood | Better IR penetration |
| Reflective | VCSEL + Flood | Multiple angles reduce specular reflections |
| Far Distance | Flood | Uniform illumination reaches further |

### 3. **Advanced Temporal Matching Sequence**

#### 4-Frame Capture Mode (Full Temporal)
```cpp
// Total time: ~400ms for complete sequence
Frame 1: VCSEL1 ON, VCSEL2 OFF, Floods OFF  // Upper pattern
Frame 2: VCSEL1 OFF, VCSEL2 ON, Floods OFF  // Lower pattern
Frame 3: VCSELs OFF, Floods ON              // Texture only
Frame 4: All OFF                            // Ambient baseline

// Pattern extraction
Pattern1 = Frame1 - Frame4  // Pure upper VCSEL pattern
Pattern2 = Frame2 - Frame4  // Pure lower VCSEL pattern
Texture = Frame3 - Frame4   // Pure flood texture
```

#### Adaptive 2-Frame Mode (Fast)
```cpp
if (low_light_detected) {
    Frame 1: Floods ON   // Texture
    Frame 2: Ambient     // Baseline
} else {
    Frame 1: VCSEL1 ON   // Pattern
    Frame 2: Ambient     // Baseline
}
```

## Implementation Details

### Hardware Connections

#### AS1170 #1 (Existing)
```
I2C Bus 1, Address 0x30
├── LED1+ → VCSEL1 Anode
├── LED1- → VCSEL1 Cathode
├── LED2+ → VCSEL2 Anode
├── LED2- → VCSEL2 Cathode
├── STROBE → GPIO 19
└── I2C → Bus 1 (SDA/SCL)
```

#### AS1170 #2 (New)
```
I2C Bus 2, Address 0x30
├── LED1+ → Flood Left Anode
├── LED1- → Flood Left Cathode
├── LED2+ → Flood Right Anode
├── LED2- → Flood Right Cathode
├── STROBE → GPIO ?? (to be assigned)
└── I2C → Bus 2 (SDA/SCL)
```

### Software Architecture

```cpp
class DualAS1170System {
    AS1170Controller* vcsel_controller;  // Bus 1, 0x30
    AS1170Controller* flood_controller;  // Bus 2, 0x30

    enum IlluminationMode {
        VCSEL_ONLY,      // Structured light only
        FLOOD_ONLY,      // Texture illumination only
        TEMPORAL_FULL,   // 4-frame sequence
        TEMPORAL_ADAPTIVE // 2-frame based on conditions
    };

    void captureWithIllumination(IlluminationMode mode) {
        switch(mode) {
            case VCSEL_ONLY:
                vcsel_controller->setLEDState(LED1, true, 200);
                capture();
                vcsel_controller->setLEDState(LED1, false, 0);
                break;

            case FLOOD_ONLY:
                flood_controller->setLEDState(BOTH, true, 200);
                capture();
                flood_controller->setLEDState(BOTH, false, 0);
                break;

            case TEMPORAL_FULL:
                // 4-frame sequence as described above
                break;
        }
    }
};
```

### Current Control Settings

| LED Type | Recommended Current | Max Safe Current | Purpose |
|----------|-------------------|------------------|---------|
| VCSEL | 200mA | 250mA | Pattern projection |
| Flood | 200-300mA | 400mA | Area illumination |

**Total Power Budget**:
- All 4 LEDs: 800-1000mA (within USB 3.0 limits)
- Typical usage: 400mA (2 LEDs at a time)

## Adaptive Illumination Algorithm

```cpp
struct SceneAnalysis {
    float ambient_brightness;  // Lux measurement
    float texture_score;       // Texture richness (0-1)
    float distance_estimate;   // Meters
    bool has_specular;        // Reflective surfaces detected
};

IlluminationMode selectIllumination(const SceneAnalysis& scene) {
    if (scene.ambient_brightness < 10.0f) {
        // Very dark - need flood
        return FLOOD_ONLY;
    } else if (scene.texture_score < 0.3f) {
        // Low texture - need VCSEL pattern
        return VCSEL_ONLY;
    } else if (scene.has_specular) {
        // Reflective - use temporal to separate
        return TEMPORAL_FULL;
    } else {
        // Good conditions - adaptive based on speed needs
        return TEMPORAL_ADAPTIVE;
    }
}
```

## Benefits for Industrial Use

1. **Robustness**: Works in warehouse darkness or bright sunlight
2. **Surface Agnostic**: Handles black plastic, metal, wood equally well
3. **Speed Options**: Fast 2-frame or thorough 4-frame modes
4. **Failure Redundancy**: If one illumination fails, others compensate
5. **Quality Metrics**: Can compare VCSEL vs Flood depth maps for confidence

## Implementation Priority

### Phase 1: Current System Optimization
- [x] Temporal matching with single AS1170
- [x] VCSEL1 working
- [ ] Fix VCSEL2 connection issue
- [x] Optimize SGBM parameters for dots

### Phase 2: Dual AS1170 Integration
- [ ] Add second AS1170 on Bus 2
- [ ] Connect flood LEDs (horizontal mount)
- [ ] Implement dual controller software
- [ ] Add illumination mode selection to GUI

### Phase 3: Adaptive Intelligence
- [ ] Scene analysis module
- [ ] Automatic illumination selection
- [ ] Quality-based frame fusion
- [ ] Performance optimization

## Testing Considerations

1. **Gradual Testing**:
   - Test flood LEDs independently first
   - Verify no I2C conflicts between buses
   - Measure thermal performance with 4 LEDs

2. **Safety**:
   - Implement thermal monitoring for both AS1170s
   - Add current limiting per LED type
   - Emergency shutdown for over-temperature

3. **Performance Metrics**:
   - Compare depth quality: VCSEL vs Flood vs Combined
   - Measure capture time for each mode
   - Validate in various lighting conditions

## Cost Analysis

| Component | Quantity | Unit Cost | Total |
|-----------|----------|-----------|--------|
| AS1170 | 1 | $5-10 | $5-10 |
| Flood LED | 2 | $3-5 | $6-10 |
| Mounting | 1 | $2 | $2 |
| **Total** | | | **$13-22** |

## Conclusion

Adding a second AS1170 with flood LEDs would significantly enhance the scanner's robustness in non-optimal lighting conditions. The combination of structured light (VCSEL) and uniform illumination (flood) provides the best of both worlds for industrial 3D scanning applications.

The modular approach allows for gradual implementation and testing, with immediate benefits even before full adaptive intelligence is implemented.