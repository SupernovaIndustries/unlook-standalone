#!/bin/bash
echo "Testing MediaPipe timestamp fix..."
cd /home/alessandro/unlook-gesture

python3 << 'PYTHON_EOF'
import sys
sys.path.insert(0, '/home/alessandro/unlook-gesture/python')
from mediapipe_gesture_detector import MediaPipeGestureDetector
import numpy as np

print("Creating detector...")
detector = MediaPipeGestureDetector(
    model_path="/home/alessandro/unlook-gesture/models/gesture_recognizer.task",
    num_hands=1
)

print("Testing 50 frames to verify monotonic timestamps...")
frame = np.zeros((480, 640, 3), dtype=np.uint8)
errors = 0

for i in range(50):
    try:
        gesture, landmarks, confidence = detector.detect(frame)
        if (i+1) % 10 == 0:
            print(f"  Frame {i+1}: OK (avg inference: {detector.get_avg_inference_time_ms():.1f}ms)")
    except Exception as e:
        print(f"  Frame {i+1}: ERROR - {e}")
        errors += 1

if errors == 0:
    print("✅ SUCCESS: All 50 frames processed without timestamp errors!")
else:
    print(f"❌ FAILED: {errors} errors detected")

detector.close()
PYTHON_EOF
