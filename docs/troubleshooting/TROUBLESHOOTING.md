# Unlook 3D Scanner - Troubleshooting Guide

## Camera Synchronization Issues

### Symptom: No Camera Detected
- **Possible Causes**:
  1. Loose I2C connections
  2. Incorrect I2C bus configuration
  3. Damaged camera module
- **Troubleshooting Steps**:
  1. Check physical connections
  2. Verify I2C bus settings
  3. Run `libcamera-hello --list-cameras`
  4. Inspect system logs

### Symptom: Desynchronized Camera Frames
- **Possible Causes**:
  1. Incorrect XVS/XHS connection
  2. Thermal instability
  3. Misaligned MAS pin configuration
- **Troubleshooting Steps**:
  1. Verify hardware sync connections
  2. Check thermal environment
  3. Re-run hardware sync validation
  4. Use `./build/test_hardware_sync_new`

## LED System Problems

### Symptom: LED Not Responding
- **Possible Causes**:
  1. I2C communication failure
  2. Incorrect device address
  3. Power supply issues
- **Troubleshooting Steps**:
  1. Confirm I2C bus (Bus 1, Address 0x30)
  2. Check GPIO 19 strobe control
  3. Verify power connections

### Symptom: Thermal Shutdown
- **Possible Causes**:
  1. Overheating
  2. Inadequate ventilation
  3. Extended high-intensity operation
- **Troubleshooting Steps**:
  1. Check ambient temperature
  2. Ensure proper ventilation
  3. Reduce LED intensity
  4. Allow cooling period

## Calibration and Precision Issues

### Symptom: High Calibration Error
- **Possible Causes**:
  1. Misaligned calibration target
  2. Unstable mounting
  3. Lens distortion
- **Troubleshooting Steps**:
  1. Re-check calibration target alignment
  2. Verify mechanical stability
  3. Clean camera lenses
  4. Recalibrate using provided wizard

### Symptom: Inconsistent Depth Measurements
- **Possible Causes**:
  1. Thermal variations
  2. Mechanical vibrations
  3. Lighting inconsistencies
- **Troubleshooting Steps**:
  1. Maintain stable environment
  2. Use anti-vibration mount
  3. Consistent lighting conditions
  4. Perform multiple measurements

## Software and Runtime Issues

### Symptom: GUI Fails to Launch
- **Possible Causes**:
  1. Library path misconfiguration
  2. Missing dependencies
  3. Incompatible system configuration
- **Troubleshooting Steps**:
  1. Verify LD_LIBRARY_PATH
  2. Reinstall dependencies
  3. Check system compatibility
  ```bash
  export LD_LIBRARY_PATH=build/src:third-party/libcamera-sync-fix/build/src/libcamera:third-party/libcamera-sync-fix/build/src/libcamera/base:$LD_LIBRARY_PATH
  ```

### Symptom: Performance Degradation
- **Possible Causes**:
  1. Memory leaks
  2. Background processes
  3. Insufficient system resources
- **Troubleshooting Steps**:
  1. Use AddressSanitizer
  2. Monitor system resources
  3. Close unnecessary applications
  4. Consider CM5 with 16GB RAM

## Advanced Diagnostics

### Logging and Debugging
- Enable verbose logging
- Capture system logs
- Use provided diagnostic tools

### Performance Validation
```bash
# Run performance tests
./build_and_test_performance.sh

# Memory analysis
valgrind ./build/src/gui/unlook_scanner
```

## Contact and Support

- **Email**: support@unlook3dscanner.com
- **Community Forum**: https://forum.unlook3dscanner.com
- **Emergency Support**: Refer to README for contact details

## Version and Compatibility

- **Current Version**: Check `unlook.h` for exact version
- **Recommended Hardware**: Raspberry Pi CM4/CM5
- **Supported OS**: Custom Raspbian-based distribution