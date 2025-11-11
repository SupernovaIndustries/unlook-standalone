#!/bin/bash
# Quick test of MediaPipe Python backend

echo "Testing MediaPipe Python backend..."
cd /home/alessandro/unlook-gesture

python3 << 'PYTHON_EOF'
import sys
sys.path.insert(0, '/home/alessandro/unlook-gesture/python')

print("Importing MediaPipe gesture detector...")
from mediapipe_gesture_detector import MediaPipeGestureDetector

print("Creating detector instance...")
detector = MediaPipeGestureDetector(
    model_path="/home/alessandro/unlook-gesture/models/gesture_recognizer.task",
    num_hands=1
)

print("Creating test frame...")
import cv2
import numpy as np
frame = np.zeros((480, 640, 3), dtype=np.uint8)

print("Running 10 detection cycles to test time.time() fix...")
for i in range(10):
    gesture, landmarks, confidence = detector.detect(frame)
    if i % 3 == 0:
        print(f"  Frame {i+1}: gesture={gesture}, landmarks={len(landmarks)}, confidence={confidence:.3f}")

print("SUCCESS: MediaPipe backend working without time.time() errors!")
print(f"Final FPS: {detector.get_fps():.1f}")
print(f"Avg inference: {detector.get_avg_inference_time_ms():.1f}ms")

detector.close()
PYTHON_EOF

echo "Test completed!"
