# Motor Control Diagnostic Guide

## Current Issues Diagnosed:

1. **Velocity readings showing 0.00** - This suggests encoder problems
2. **PWM values going above 1023** - This should be limited to 1023 for 10-bit resolution
3. **Inconsistent velocity readings** - Timing and calculation issues

## Changes Made:

### 1. Fixed PID Sample Time
- Changed from 500ms to 100ms for better motor response

### 2. Fixed Velocity Calculation  
- Corrected the odometry velocity calculation
- Fixed RPM to linear velocity conversion

### 3. Improved RPM Calculation
- Added minimum time threshold for reliable measurements
- Added gradual decay when no movement is detected

### 4. Added Debug Functions
- `debugEncoders()` - Monitor encoder pulse counts
- Better serial output formatting

## Testing Steps:

### Step 1: Check Encoder Connections
1. Verify encoder pins EN_A (27) and EN_B (14) are properly connected
2. Check if encoders have pull-up resistors (code uses INPUT_PULLUP)
3. Test encoder signals with multimeter or oscilloscope

### Step 2: Verify Encoder Pulse Count
With the new debug output, you should see:
```
=== ENCODER DEBUG ===
Left Encoder Position: [increasing numbers]
Right Encoder Position: [increasing numbers]
```

### Step 3: Calibrate Encoder Pulses Per Revolution
- Current setting: 20 pulses per revolution
- This might be too low for your encoders
- Common values: 20, 100, 360, 400, 1024 pulses per revolution

### Step 4: Test Motor Movement
Send Bluetooth commands:
- 'A' - Forward at 20 RPM
- 'S' - Forward at 45 RPM  
- 'P' - Stop

## Potential Issues to Check:

1. **Encoder Type**: Make sure ENCODER_PULSES_PER_REVOLUTION matches your actual encoder
2. **Encoder Wiring**: Check if both encoders are wired correctly
3. **Power Supply**: Ensure motors have sufficient power
4. **Motor Direction**: Check if motor directions are correct

## Next Steps:
1. Upload the modified code
2. Monitor the debug output
3. Check if encoder positions are incrementing when motors run
4. Adjust ENCODER_PULSES_PER_REVOLUTION if needed
