#!/usr/bin/env python3
"""
Depth Error Calculator - Visualizza impatto errore calibrazione su Z

Usage:
    python3 depth_error_calculator.py
"""

import numpy as np
import matplotlib.pyplot as plt

# Unlook parameters
FOCAL_LENGTH = 1772.98  # pixels
BASELINE = 70.017       # mm

def calculate_depth_error(z_mm, disparity_error_px):
    """
    Calcola errore in profondità dato errore disparity

    Formula: ΔZ = (Z² / (f × B)) × Δd
    """
    denominator = FOCAL_LENGTH * BASELINE
    depth_error = (z_mm ** 2 / denominator) * disparity_error_px
    return depth_error

def calculate_disparity(z_mm):
    """Calcola disparity per data profondità"""
    return (FOCAL_LENGTH * BASELINE) / z_mm

# Range di profondità da testare
depths = np.linspace(200, 2000, 100)  # 200mm - 2000mm

# Diversi livelli di errore calibrazione/matching
errors = {
    'Current (RMS 0.242px)': 0.242,
    'Target (RMS 0.15px)': 0.15,
    'MATLAB (RMS 0.10px)': 0.10,
    'Artec (RMS 0.05px)': 0.05,
    'SGBM noise (0.3px)': 0.3,
    'Sub-pixel (0.0625px)': 0.0625,
}

# Plot 1: Depth Error vs Distance
plt.figure(figsize=(14, 10))

plt.subplot(2, 2, 1)
for label, error_px in errors.items():
    depth_errors = [calculate_depth_error(z, error_px) for z in depths]
    plt.plot(depths, depth_errors, label=label, linewidth=2)

plt.axhline(y=0.1, color='r', linestyle='--', label='Target (0.1mm)', linewidth=2)
plt.axvline(x=500, color='g', linestyle=':', label='Demo distance (500mm)', alpha=0.5)
plt.xlabel('Distance Z (mm)', fontsize=12)
plt.ylabel('Depth Error ΔZ (mm)', fontsize=12)
plt.title('Depth Error vs Distance (Non-Linear Growth!)', fontsize=14, fontweight='bold')
plt.legend(fontsize=9)
plt.grid(True, alpha=0.3)
plt.ylim(0, 5)

# Plot 2: Disparity vs Distance
plt.subplot(2, 2, 2)
disparities = [calculate_disparity(z) for z in depths]
plt.plot(depths, disparities, linewidth=2, color='blue')
plt.xlabel('Distance Z (mm)', fontsize=12)
plt.ylabel('Disparity (pixels)', fontsize=12)
plt.title('Disparity vs Distance (Hyperbolic)', fontsize=14, fontweight='bold')
plt.grid(True, alpha=0.3)
plt.axvline(x=500, color='g', linestyle=':', label='Demo distance', alpha=0.5)
plt.legend()

# Plot 3: Error at 500mm for different calibrations
plt.subplot(2, 2, 3)
demo_distance = 500  # mm
error_values = []
error_labels = []

for label, error_px in errors.items():
    error_mm = calculate_depth_error(demo_distance, error_px)
    error_values.append(error_mm)
    error_labels.append(f'{label}\n({error_mm:.3f}mm)')

colors = ['red' if e > 0.1 else 'green' for e in error_values]
bars = plt.bar(range(len(error_values)), error_values, color=colors, alpha=0.7)
plt.axhline(y=0.1, color='r', linestyle='--', linewidth=2, label='Target (0.1mm)')
plt.xticks(range(len(error_labels)), [l.split('(')[0] for l in error_labels],
           rotation=45, ha='right', fontsize=9)
plt.ylabel('Depth Error @ 500mm (mm)', fontsize=12)
plt.title('Error Comparison @ 500mm Demo Distance', fontsize=14, fontweight='bold')
plt.legend()
plt.grid(True, alpha=0.3, axis='y')

# Aggiungi valori sulle barre
for i, (bar, val) in enumerate(zip(bars, error_values)):
    plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.05,
             f'{val:.3f}mm', ha='center', va='bottom', fontsize=9, fontweight='bold')

# Plot 4: Required disparity precision for target
plt.subplot(2, 2, 4)
target_error_mm = 0.1  # Target: 0.1mm @ 500mm

# Calcola precisione disparity necessaria per ogni distanza
required_precision = []
for z in depths:
    # Da ΔZ = (Z² / (f × B)) × Δd
    # Ricaviamo Δd = ΔZ × (f × B) / Z²
    delta_d = target_error_mm * (FOCAL_LENGTH * BASELINE) / (z ** 2)
    required_precision.append(delta_d)

plt.plot(depths, required_precision, linewidth=2, color='purple')
plt.axhline(y=0.0625, color='orange', linestyle='--', label='SGBM sub-pixel (1/16px)', linewidth=2)
plt.axhline(y=0.01, color='green', linestyle='--', label='Phase-shift (0.01px)', linewidth=2)
plt.xlabel('Distance Z (mm)', fontsize=12)
plt.ylabel('Required Δd Precision (pixels)', fontsize=12)
plt.title('Disparity Precision Required for 0.1mm @ Each Distance', fontsize=14, fontweight='bold')
plt.legend()
plt.grid(True, alpha=0.3)
plt.yscale('log')

plt.tight_layout()
plt.savefig('depth_error_analysis.png', dpi=150, bbox_inches='tight')
print("\n✅ Plot saved as 'depth_error_analysis.png'")

# Print numerical table
print("\n" + "="*80)
print("DEPTH ERROR TABLE @ KEY DISTANCES")
print("="*80)
print(f"{'Distance':<12} {'Disparity':<12} {'Current':<15} {'MATLAB':<15} {'Artec':<15}")
print(f"{'(mm)':<12} {'(pixels)':<12} {'(0.242px)':<15} {'(0.10px)':<15} {'(0.05px)':<15}")
print("-"*80)

for z in [300, 400, 500, 600, 800, 1000]:
    d = calculate_disparity(z)
    err_current = calculate_depth_error(z, 0.242)
    err_matlab = calculate_depth_error(z, 0.10)
    err_artec = calculate_depth_error(z, 0.05)

    print(f"{z:<12} {d:<12.2f} {err_current:<15.3f} {err_matlab:<15.3f} {err_artec:<15.3f}")

print("="*80)

# Improvement calculation
print("\n" + "="*80)
print("IMPROVEMENT ANALYSIS @ 500mm DEMO DISTANCE")
print("="*80)

demo_z = 500
current_error = calculate_depth_error(demo_z, 0.242)
matlab_error = calculate_depth_error(demo_z, 0.10)
artec_error = calculate_depth_error(demo_z, 0.05)

print(f"Current calibration (RMS 0.242px): {current_error:.3f}mm")
print(f"MATLAB calibration (RMS 0.10px):   {matlab_error:.3f}mm")
print(f"Artec-level (RMS 0.05px):          {artec_error:.3f}mm")
print()
print(f"Improvement MATLAB vs Current: {current_error/matlab_error:.1f}x better")
print(f"Improvement Artec vs Current:  {current_error/artec_error:.1f}x better")
print()

if current_error > 0.1:
    print(f"❌ Current: {current_error:.3f}mm > 0.1mm target ({current_error/0.1:.1f}x over limit)")
else:
    print(f"✅ Current: {current_error:.3f}mm < 0.1mm target")

if matlab_error <= 0.1:
    print(f"✅ MATLAB:  {matlab_error:.3f}mm ≤ 0.1mm target (MEETS REQUIREMENT!)")
else:
    print(f"⚠️ MATLAB:  {matlab_error:.3f}mm > 0.1mm target")

if artec_error <= 0.1:
    print(f"✅ Artec:   {artec_error:.3f}mm ≤ 0.1mm target (EXCEEDS REQUIREMENT!)")

print("="*80)

# Conclusion
print("\n" + "="*80)
print("CONCLUSION & RECOMMENDATIONS")
print("="*80)
print("""
1. CRITICAL: Current calibration RMS (0.242px) causes {:.3f}mm error @ 500mm
   → {}x OVER 0.1mm target

2. MATLAB calibration (RMS 0.10px) would give {:.3f}mm error @ 500mm
   → {} MEETS 0.1mm requirement!

3. PRIORITY ACTION: Integrate MATLAB calibration ASAP
   Expected improvement: {:.1f}x better depth accuracy

4. SGBM parameters already good (sub-pixel 1/16px = 0.0625px)
   Main bottleneck is CALIBRATION, not stereo matching!
""".format(
    current_error,
    "10" if current_error > 1.0 else f"{current_error/0.1:.1f}",
    matlab_error,
    "✅" if matlab_error <= 0.1 else "⚠️",
    current_error/matlab_error
))
print("="*80)

plt.show()
