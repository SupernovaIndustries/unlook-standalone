# Test Plan: Swap Camera Matrices

## Problem
- Images are distorted with AND without R1↔R2 swap
- This means the problem is not just in rectification transforms

## Hypothesis
The calibration file has:
- `camera_matrix_left` → actually for RIGHT camera (Camera 0)
- `camera_matrix_right` → actually for LEFT camera (Camera 1)

Same for distortion coefficients.

## Test
Swap camera matrices AND distortion coeffs during loading, in addition to R1↔R2 swap.
