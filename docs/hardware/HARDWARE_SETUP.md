# Unlook 3D Scanner - Hardware Setup Guide

## Hardware Components

### Camera System
- **Sensors**: 2x Raspberry Pi IMX296 Global Shutter Cameras
  - Resolution: 1456x1088 SBGGR10
  - Baseline: 70.017mm
- **Lens**: 6mm focal length
- **Synchronization**: Hardware XVS/XHS sync

### LED System
- **LED Controller**: AS1170 LED Driver
  - I2C Bus: 1
  - Device Address: 0x30
  - Strobe Control: GPIO 19
- **LED Components**:
  - LED1: OSRAM BELAGO1.1 VCSEL Dot Projector
  - LED2: Flood Illuminator

## Wiring and Configuration

### Camera Connections
1. Camera 1 (LEFT/MASTER):
   - Device Path: `/base/soc/i2c0mux/i2c@1/imx296@1a`
   - Role: Master camera
   - Connections:
     - XVS: Connected
     - XHS: Connected
     - MAS Pin: Soldered as MASTER

2. Camera 0 (RIGHT/SLAVE):
   - Device Path: `/base/soc/i2c0mux/i2c@0/imx296@1a`
   - Role: Slave camera
   - Connections:
     - XVS: Connected
     - XHS: Connected
     - MAS Pin: Soldered as SLAVE

### LED Wiring
1. AS1170 LED Driver Configuration:
   - I2C Connection
   - GPIO 19 for Strobe Control
   - Thermal Monitoring Enabled

## Calibration Requirements

### Calibration Targets
- Checkerboard: 7x10 pattern, 24mm squares
- ChArUco Board:
  - 7x10 pattern
  - 24mm squares
  - ArUco markers: 17mm
  - Dictionary: DICT_4X4_250

## Setup Procedure

1. **Camera Mounting**
   - Ensure rigid, thermally stable mount
   - Baseline distance: Precisely 70.017mm
   - Check mechanical alignment

2. **LED System Setup**
   - Mount VCSEL projector (LED1)
   - Position flood illuminator (LED2)
   - Verify I2C connections
   - Configure GPIO strobe control

3. **Initial Calibration**
   - Use provided calibration targets
   - Follow onscreen calibration wizard
   - Verify RMS error <0.2px

## Performance Verification

- **Precision Target**: 0.1mm industrial standard
- **Synchronization**: <1ms camera sync
- **LED Timing**: ±20μs activation accuracy

## Troubleshooting

1. Check physical connections
2. Verify I2C bus and GPIO configuration
3. Ensure thermal stability
4. Run hardware validation tests