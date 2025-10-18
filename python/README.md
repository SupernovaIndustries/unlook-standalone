# MediaPipe Python Backend

This directory contains the Python implementation of the MediaPipe gesture recognition backend for the Unlook 3D Scanner.

## Architecture

The Python backend provides a clean interface to Google MediaPipe's gesture recognition, which is then wrapped by C++ using pybind11 for integration with the main Unlook system.

```
┌─────────────────────────────────────────────────────────┐
│                   Unlook Qt GUI (C++)                   │
└─────────────────────┬───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│         GestureRecognitionSystem (C++)                  │
└─────────────────────┬───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│            MediaPipeWrapper (C++ pybind11)              │
└─────────────────────┬───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│      mediapipe_gesture_detector.py (Python)             │
└─────────────────────┬───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│            MediaPipe Library (Google)                   │
└─────────────────────────────────────────────────────────┘
```

## Files

- **mediapipe_gesture_detector.py**: Main gesture detection module
- **README.md**: This file

## Installation

### 1. Create Virtual Environment

```bash
cd /home/alessandro/unlook-gesture
python3 -m venv mediapipe_env
source mediapipe_env/bin/activate
```

### 2. Install Dependencies

```bash
pip3 install mediapipe opencv-python numpy
```

### 3. Download Model

```bash
mkdir -p ../models
cd ../models
wget https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task
```

### 4. Verify Installation

```bash
python3 -c "import mediapipe as mp; print('MediaPipe version:', mp.__version__)"
```

Expected output: `MediaPipe version: 0.10.18` (or later)

## Testing

### Test with Webcam

```bash
cd /home/alessandro/unlook-gesture/python
source ../mediapipe_env/bin/activate
python3 mediapipe_gesture_detector.py
```

**Controls:**
- Press `q` to quit
- Press `r` to reset performance statistics
- Press `d` to toggle debug visualization

### Expected Performance (Raspberry Pi 5)

- **FPS**: 14-15 FPS (baseline, CPU-only)
- **Inference Time**: ~65-70ms per frame
- **With Hailo-8L**: 26-28 FPS (future upgrade)

## Supported Gestures

The pre-trained MediaPipe model recognizes these gestures:

1. **Closed_Fist**: Hand closed in a fist
2. **Open_Palm**: Open palm facing camera
3. **Pointing_Up**: Index finger pointing up
4. **Thumb_Down**: Thumbs down gesture
5. **Thumb_Up**: Thumbs up gesture
6. **Victory**: Peace sign (V with index and middle fingers)
7. **ILoveYou**: I Love You sign (ASL)

## API Reference

### MediaPipeGestureDetector

```python
detector = MediaPipeGestureDetector(
    model_path="../models/gesture_recognizer.task",
    num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)
```

#### detect(frame) -> Tuple[Optional[str], List[List[float]], float]

Detect gesture from BGR frame.

**Parameters:**
- `frame`: OpenCV BGR frame (numpy array, shape: [H, W, 3])

**Returns:**
- `gesture_name`: Recognized gesture string or None
- `landmarks`: List of 21 landmarks [[x, y, z], ...] in normalized coords [0,1]
- `confidence`: Detection confidence [0.0, 1.0]

**Example:**
```python
gesture, landmarks, confidence = detector.detect(frame)
if gesture:
    print(f"Detected: {gesture} (confidence: {confidence:.2f})")
```

#### get_fps() -> float

Get current processing FPS.

#### get_avg_inference_time_ms() -> float

Get average inference time in milliseconds.

#### reset_stats()

Reset performance statistics.

#### close()

Clean up resources.

## Hand Landmarks

MediaPipe detects 21 hand landmarks in this order:

```
0:  WRIST
1:  THUMB_CMC
2:  THUMB_MCP
3:  THUMB_IP
4:  THUMB_TIP
5:  INDEX_FINGER_MCP
6:  INDEX_FINGER_PIP
7:  INDEX_FINGER_DIP
8:  INDEX_FINGER_TIP
9:  MIDDLE_FINGER_MCP
10: MIDDLE_FINGER_PIP
11: MIDDLE_FINGER_DIP
12: MIDDLE_FINGER_TIP
13: RING_FINGER_MCP
14: RING_FINGER_PIP
15: RING_FINGER_DIP
16: RING_FINGER_TIP
17: PINKY_MCP
18: PINKY_PIP
19: PINKY_DIP
20: PINKY_TIP
```

Each landmark has:
- `x`: Normalized x-coordinate [0, 1]
- `y`: Normalized y-coordinate [0, 1]
- `z`: Depth value (negative = closer to camera)

## Custom Gesture Training

To train custom gestures, use MediaPipe Model Maker:

```python
from mediapipe_model_maker import gesture_recognizer

# Train on custom dataset
model = gesture_recognizer.GestureRecognizer.create(
    train_data=my_custom_gestures,
    validation_data=validation_data,
    hparams=gesture_recognizer.HParams(
        export_dir='custom_model',
        epochs=50
    )
)

# Export to TFLite
model.export_model()
```

See: https://ai.google.dev/edge/mediapipe/solutions/customization/gesture_recognizer

## Troubleshooting

### Model Not Found

```
ERROR: MediaPipe initialization failed: Model file not found
```

**Solution:** Ensure model is downloaded to `../models/gesture_recognizer.task`

### Low FPS (< 10 FPS)

**Possible causes:**
1. High camera resolution (reduce to 640x480)
2. Heavy system load (close other applications)
3. CPU throttling (check system temperature)

**Solution:** Monitor with `htop` and `vcgencmd measure_temp`

### No Gesture Detected

**Possible causes:**
1. Low lighting conditions
2. Hand too far from camera
3. Confidence threshold too high

**Solution:**
- Improve lighting
- Move hand closer to camera
- Lower `min_detection_confidence` parameter

## References

- **MediaPipe Hand Landmarker**: https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker
- **MediaPipe Gesture Recognizer**: https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer
- **Raspberry Pi Guide**: https://github.com/google-ai-edge/mediapipe-samples/tree/main/examples/hand_landmarker/raspberry_pi

## Performance Optimization

### Current Performance (Raspberry Pi 5)
- **Baseline**: 14-15 FPS (CPU-only)
- **Inference**: ~65-70ms per frame
- **Memory**: ~150MB

### Future Optimization with Hailo-8L
- **Expected FPS**: 26-28 FPS
- **Hardware Acceleration**: 13 TOPS AI accelerator
- **Cost**: $70 (Raspberry Pi AI Kit)
- **Integration**: Use MediaPipe with Hailo delegate

### Code Optimizations
1. **Frame Skipping**: Process every 2nd frame for 2x throughput
2. **Resolution Reduction**: Use 640x480 instead of 1456x1088
3. **Model Quantization**: Convert to INT8 for Edge TPU/Hailo
4. **Multi-threading**: Async frame capture + processing

## License

This module is part of the Unlook 3D Scanner project (MIT License).

MediaPipe is developed by Google and licensed under Apache License 2.0.
