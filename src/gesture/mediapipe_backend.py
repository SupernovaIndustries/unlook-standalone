#!/usr/bin/env python3
"""
MediaPipe Gesture Recognition Backend

Wraps Google MediaPipe SDK for hand tracking and gesture recognition.
This backend provides the Python interface that will be called from C++ via pybind11.

Performance: 14-15 FPS on Raspberry Pi 5 (baseline)
             26-28 FPS with Hailo-8L accelerator

Copyright 2025 Unlook Project
License: MIT License
"""

import cv2
import mediapipe as mp
import numpy as np
from typing import Optional, List, Tuple
from dataclasses import dataclass
import sys


@dataclass
class GestureResult:
    """Result from gesture detection"""
    gesture_name: str
    confidence: float
    hand_landmarks: Optional[List[Tuple[float, float, float]]]  # 21 points (x, y, z)
    handedness: str  # "Left" or "Right"


class MediaPipeGestureBackend:
    """MediaPipe gesture recognition backend

    This class wraps the MediaPipe Gesture Recognizer solution for real-time
    hand tracking and gesture recognition on Raspberry Pi.

    Architecture:
        Input Image (640x480)
            ↓
        Palm Detection (TFLite)
            ↓
        Hand ROI Extraction
            ↓
        Hand Landmark Detection (21 points, TFLite)
            ↓
        Gesture Classification (TFLite)
            ↓
        Output: Gesture + Landmarks + Confidence
    """

    def __init__(self, model_path: str = ""):
        """Initialize MediaPipe Gesture Recognizer

        Args:
            model_path: Path to gesture_recognizer.task model file
                       If empty, uses default path relative to project root
        """
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        # Determine model path
        if not model_path:
            # Default path relative to project root
            model_path = "/home/alessandro/unlook-gesture/models/gesture_recognizer.task"

        self.model_path = model_path

        # Initialize Gesture Recognizer with MediaPipe Tasks API
        try:
            BaseOptions = mp.tasks.BaseOptions
            GestureRecognizer = mp.tasks.vision.GestureRecognizer
            GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
            VisionRunningMode = mp.tasks.vision.RunningMode

            options = GestureRecognizerOptions(
                base_options=BaseOptions(model_asset_path=self.model_path),
                running_mode=VisionRunningMode.VIDEO,
                num_hands=1,  # Single hand for Unlook Scanner
                min_hand_detection_confidence=0.3,  # Lower for easier detection
                min_hand_presence_confidence=0.3,
                min_tracking_confidence=0.3
            )

            self.recognizer = GestureRecognizer.create_from_options(options)
            self.frame_timestamp_ms = 0
            self.initialized = True

            print(f"[MediaPipeGestureBackend] Initialized successfully with model: {self.model_path}")

        except Exception as e:
            print(f"[MediaPipeGestureBackend] ERROR: Failed to initialize: {e}", file=sys.stderr)
            self.initialized = False
            raise

    def is_initialized(self) -> bool:
        """Check if backend is initialized

        Returns:
            True if initialized successfully
        """
        return self.initialized

    def process_frame(self, frame: np.ndarray) -> Optional[GestureResult]:
        """Process frame and detect gestures

        Args:
            frame: OpenCV BGR image (numpy array)

        Returns:
            GestureResult if hand detected, None otherwise
        """
        if not self.initialized:
            print("[MediaPipeGestureBackend] ERROR: Not initialized", file=sys.stderr)
            return None

        if frame is None or frame.size == 0:
            print("[MediaPipeGestureBackend] ERROR: Empty frame", file=sys.stderr)
            return None

        try:
            # Convert BGR to RGB (MediaPipe uses RGB)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Create MediaPipe Image
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

            # Process with timestamp
            recognition_result = self.recognizer.recognize_for_video(
                mp_image,
                self.frame_timestamp_ms
            )
            self.frame_timestamp_ms += 33  # ~30 FPS increment

            # Extract results
            if not recognition_result.gestures:
                return None

            # Get top gesture
            top_gesture = recognition_result.gestures[0][0]

            # Get hand landmarks
            landmarks = None
            if recognition_result.hand_landmarks:
                hand_landmarks_proto = recognition_result.hand_landmarks[0]
                landmarks = [
                    (lm.x, lm.y, lm.z)
                    for lm in hand_landmarks_proto
                ]

            # Get handedness
            handedness = "Unknown"
            if recognition_result.handedness:
                handedness = recognition_result.handedness[0][0].category_name

            return GestureResult(
                gesture_name=top_gesture.category_name,
                confidence=top_gesture.score,
                hand_landmarks=landmarks,
                handedness=handedness
            )

        except Exception as e:
            print(f"[MediaPipeGestureBackend] ERROR: Exception in process_frame: {e}", file=sys.stderr)
            return None

    def get_gesture_categories(self) -> List[str]:
        """Get list of supported gestures

        Returns:
            List of gesture category names
        """
        # MediaPipe default gestures
        return [
            "None",
            "Closed_Fist",
            "Open_Palm",
            "Pointing_Up",
            "Thumb_Down",
            "Thumb_Up",
            "Victory",
            "ILoveYou"
        ]

    def __del__(self):
        """Cleanup resources"""
        if hasattr(self, 'recognizer') and self.recognizer:
            self.recognizer.close()


# Test function for standalone testing
def test_backend():
    """Test MediaPipe backend with webcam"""
    print("[MediaPipeGestureBackend] Starting test...")

    try:
        backend = MediaPipeGestureBackend()

        if not backend.is_initialized():
            print("[MediaPipeGestureBackend] ERROR: Failed to initialize")
            return

        print("[MediaPipeGestureBackend] Supported gestures:", backend.get_gesture_categories())

        # Open webcam
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("[MediaPipeGestureBackend] ERROR: Cannot open webcam")
            return

        print("[MediaPipeGestureBackend] Press 'q' to quit")

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Process frame
            result = backend.process_frame(frame)

            # Draw results
            if result:
                # Draw gesture name
                text = f"{result.gesture_name} ({result.confidence:.2f})"
                cv2.putText(frame, text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

                # Draw landmarks
                if result.hand_landmarks:
                    h, w = frame.shape[:2]
                    for lm in result.hand_landmarks:
                        x = int(lm[0] * w)
                        y = int(lm[1] * h)
                        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

            cv2.imshow('MediaPipe Gesture Test', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

        print("[MediaPipeGestureBackend] Test complete")

    except Exception as e:
        print(f"[MediaPipeGestureBackend] ERROR: Test failed: {e}", file=sys.stderr)


if __name__ == "__main__":
    test_backend()
