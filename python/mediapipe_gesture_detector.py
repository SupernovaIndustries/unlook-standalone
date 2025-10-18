#!/usr/bin/env python3
"""
MediaPipe Gesture Recognition Backend
Production-ready gesture detector for Unlook 3D Scanner

Target Performance: 14-15 FPS on Raspberry Pi 5 (baseline)
                    26-28 FPS with Hailo-8L accelerator (future upgrade)

This module provides a Python interface to MediaPipe's gesture recognition,
designed to be wrapped by C++ via pybind11 for integration with the Unlook
gesture recognition system.
"""

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from typing import Optional, Tuple, List, Dict
import time
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] %(message)s'
)
logger = logging.getLogger(__name__)


class MediaPipeGestureDetector:
    """
    MediaPipe-based gesture detection with 21-point hand landmarks.

    Uses Google MediaPipe's official gesture recognizer task, which includes:
    - Palm detection model (SSD-based)
    - Hand landmark detection (21 keypoints)
    - Gesture classification (pre-trained gestures + custom)

    Performance on Raspberry Pi 5:
    - Baseline: 14-15 FPS (CPU-only)
    - With Hailo-8L: 26-28 FPS (hardware acceleration)

    Supported Gestures (pre-trained):
    - Closed_Fist
    - Open_Palm
    - Pointing_Up
    - Thumb_Down
    - Thumb_Up
    - Victory
    - ILoveYou
    """

    def __init__(self,
                 model_path: str = "../models/gesture_recognizer.task",
                 num_hands: int = 1,
                 min_detection_confidence: float = 0.5,
                 min_tracking_confidence: float = 0.5):
        """
        Initialize MediaPipe gesture recognizer.

        Args:
            model_path: Path to gesture_recognizer.task model file
            num_hands: Maximum number of hands to detect (1 for Unlook - single hand)
            min_detection_confidence: Minimum palm detection confidence [0.0, 1.0]
            min_tracking_confidence: Minimum hand tracking confidence [0.0, 1.0]

        Raises:
            RuntimeError: If model file not found or initialization fails
        """
        logger.info(f"Initializing MediaPipe gesture detector...")
        logger.info(f"  Model path: {model_path}")
        logger.info(f"  Max hands: {num_hands}")
        logger.info(f"  Detection confidence: {min_detection_confidence}")
        logger.info(f"  Tracking confidence: {min_tracking_confidence}")

        try:
            # Configure base options
            base_options = python.BaseOptions(model_asset_path=model_path)

            # Configure gesture recognizer options
            options = vision.GestureRecognizerOptions(
                base_options=base_options,
                running_mode=vision.RunningMode.VIDEO,
                num_hands=num_hands,
                min_hand_detection_confidence=min_detection_confidence,
                min_hand_presence_confidence=min_tracking_confidence,
                min_tracking_confidence=min_tracking_confidence
            )

            # Create recognizer
            self.recognizer = vision.GestureRecognizer.create_from_options(options)

            # Performance tracking (simplified - no FPS counter)
            self.total_frame_count = 0  # For monotonic timestamps (never reset)
            self.total_inference_time = 0.0
            self.avg_inference_time = 0.0

            logger.info("MediaPipe gesture detector initialized successfully")

        except Exception as e:
            logger.error(f"Failed to initialize MediaPipe: {e}")
            raise RuntimeError(f"MediaPipe initialization failed: {e}")

    def detect(self, frame: np.ndarray) -> Tuple[Optional[str], List[List[float]], float]:
        """
        Detect gesture and hand landmarks from frame.

        Args:
            frame: BGR image from OpenCV (numpy array, uint8, shape: [H, W, 3])

        Returns:
            Tuple of (gesture_name, landmarks, confidence):
            - gesture_name: Recognized gesture string (e.g., "Thumb_Up", "Victory")
                           or None if no gesture detected
            - landmarks: List of 21 landmarks [[x, y, z], ...] in normalized coordinates [0,1]
                        Empty list if no hand detected
            - confidence: Detection confidence score [0.0, 1.0]
                         0.0 if no gesture detected

        Example:
            gesture, landmarks, confidence = detector.detect(frame)
            if gesture:
                print(f"Detected: {gesture} (confidence: {confidence:.2f})")
                for i, (x, y, z) in enumerate(landmarks):
                    print(f"  Landmark {i}: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        """
        try:
            start_inference = time.time()
        except Exception:
            # Workaround for pybind11 SystemError with pending exceptions
            import sys
            sys.exc_clear() if hasattr(sys, 'exc_clear') else None
            start_inference = time.time()

        # Validate input
        if frame is None or frame.size == 0:
            logger.warning("Empty frame received")
            return None, [], 0.0

        if len(frame.shape) != 3 or frame.shape[2] != 3:
            logger.warning(f"Invalid frame shape: {frame.shape}, expected [H, W, 3]")
            return None, [], 0.0

        # Convert BGR to RGB (MediaPipe expects RGB)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Create MediaPipe image
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        # Calculate timestamp in milliseconds (MediaPipe video mode requires timestamp)
        # Use total_frame_count to ensure monotonically increasing timestamps
        # MediaPipe requires timestamps to ALWAYS increase, never reset!
        timestamp_ms = int(self.total_frame_count * (1000 / 30))

        # Run gesture recognition
        result = self.recognizer.recognize_for_video(mp_image, timestamp_ms)

        # Update performance stats (simplified)
        try:
            inference_time = time.time() - start_inference
        except Exception:
            inference_time = 0.0

        self.total_inference_time += inference_time
        self.total_frame_count += 1
        self.avg_inference_time = self.total_inference_time / self.total_frame_count

        # Extract results
        gesture_name = None
        landmarks = []
        confidence = 0.0

        # Extract gesture classification
        if result.gestures and len(result.gestures) > 0:
            # Get first hand's gesture (we only detect one hand)
            top_gesture = result.gestures[0][0]
            gesture_name = top_gesture.category_name
            confidence = top_gesture.score

        # Extract hand landmarks
        if result.hand_landmarks and len(result.hand_landmarks) > 0:
            # Get first hand landmarks (21 points)
            hand = result.hand_landmarks[0]
            landmarks = [[lm.x, lm.y, lm.z] for lm in hand]

        return gesture_name, landmarks, confidence

    def get_avg_inference_time_ms(self) -> float:
        """
        Get average inference time in milliseconds.

        Returns:
            Average inference time in ms
        """
        return self.avg_inference_time * 1000.0

    def close(self):
        """Clean up resources."""
        if hasattr(self, 'recognizer') and self.recognizer:
            self.recognizer.close()
            logger.info("MediaPipe gesture detector closed")


def main():
    """
    Test MediaPipe gesture detector with webcam.

    Usage:
        python3 mediapipe_gesture_detector.py [model_path]

    Controls:
        - 'q': Quit application
        - 'r': Reset performance statistics
        - 'd': Toggle debug info display
    """
    import sys

    # Parse command line arguments
    model_path = "../models/gesture_recognizer.task"
    if len(sys.argv) > 1:
        model_path = sys.argv[1]

    print("=" * 80)
    print("MediaPipe Gesture Recognition Test")
    print("=" * 80)
    print(f"Model: {model_path}")
    print("Controls:")
    print("  'q': Quit")
    print("  'r': Reset statistics")
    print("  'd': Toggle debug info")
    print("=" * 80)

    # Initialize detector
    try:
        detector = MediaPipeGestureDetector(model_path, num_hands=1)
    except RuntimeError as e:
        print(f"ERROR: {e}")
        return 1

    # Open webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("ERROR: Could not open webcam")
        detector.close()
        return 1

    print("Webcam opened successfully. Starting gesture detection...")

    show_debug = True
    frame_counter = 0

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("WARNING: Failed to read frame from webcam")
                break

            frame_counter += 1

            # Detect gesture
            gesture, landmarks, confidence = detector.detect(frame)

            # Draw results on frame
            display_frame = frame.copy()

            # Draw FPS and inference time
            if show_debug:
                fps = detector.get_fps()
                inference_ms = detector.get_avg_inference_time_ms()

                cv2.putText(display_frame, f"FPS: {fps:.1f}",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(display_frame, f"Inference: {inference_ms:.1f}ms",
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(display_frame, f"Frame: {frame_counter}",
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # Draw gesture name LARGE in center
            if gesture and confidence > 0.5:
                # Draw gesture name
                text = f"{gesture}"
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_BOLD, 2.0, 3)[0]
                text_x = (display_frame.shape[1] - text_size[0]) // 2
                text_y = (display_frame.shape[0] + text_size[1]) // 2

                # Draw background rectangle
                cv2.rectangle(display_frame,
                             (text_x - 10, text_y - text_size[1] - 10),
                             (text_x + text_size[0] + 10, text_y + 10),
                             (0, 0, 0), -1)

                # Draw text
                cv2.putText(display_frame, text,
                           (text_x, text_y), cv2.FONT_HERSHEY_BOLD, 2.0,
                           (0, 255, 255), 3)

                # Draw confidence bar at bottom
                bar_height = 20
                bar_width = int(confidence * display_frame.shape[1])
                cv2.rectangle(display_frame,
                             (0, display_frame.shape[0] - bar_height),
                             (bar_width, display_frame.shape[0]),
                             (0, 255, 0), -1)

                # Confidence text
                conf_text = f"Confidence: {confidence:.2f}"
                cv2.putText(display_frame, conf_text,
                           (10, display_frame.shape[0] - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Draw 21 hand landmarks
            if landmarks and show_debug:
                h, w = display_frame.shape[:2]

                # Draw landmarks as circles
                for i, lm in enumerate(landmarks):
                    x, y = int(lm[0] * w), int(lm[1] * h)
                    cv2.circle(display_frame, (x, y), 5, (0, 0, 255), -1)

                    # Draw landmark index for key points
                    if i in [0, 4, 8, 12, 16, 20]:  # Wrist and fingertips
                        cv2.putText(display_frame, str(i),
                                   (x + 7, y), cv2.FONT_HERSHEY_SIMPLEX,
                                   0.4, (255, 255, 255), 1)

                # Draw hand skeleton connections
                # MediaPipe hand connections
                connections = [
                    (0, 1), (1, 2), (2, 3), (3, 4),      # Thumb
                    (0, 5), (5, 6), (6, 7), (7, 8),      # Index
                    (0, 9), (9, 10), (10, 11), (11, 12), # Middle
                    (0, 13), (13, 14), (14, 15), (15, 16), # Ring
                    (0, 17), (17, 18), (18, 19), (19, 20), # Pinky
                    (5, 9), (9, 13), (13, 17)            # Palm
                ]

                for conn in connections:
                    pt1 = (int(landmarks[conn[0]][0] * w), int(landmarks[conn[0]][1] * h))
                    pt2 = (int(landmarks[conn[1]][0] * w), int(landmarks[conn[1]][1] * h))
                    cv2.line(display_frame, pt1, pt2, (0, 255, 0), 2)

            # Display frame
            cv2.imshow("MediaPipe Gesture Recognition - Press 'q' to quit", display_frame)

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Quit requested by user")
                break
            elif key == ord('r'):
                detector.reset_stats()
                frame_counter = 0
                print("Statistics reset")
            elif key == ord('d'):
                show_debug = not show_debug
                print(f"Debug info: {'ON' if show_debug else 'OFF'}")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        detector.close()
        print("MediaPipe detector closed successfully")
        print(f"Total frames processed: {frame_counter}")

    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
