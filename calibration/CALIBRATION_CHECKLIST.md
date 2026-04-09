# Calibration Checklist

1. **Prep**
   - Stabilize board temperature (leave powered on for 15+ minutes).
   - Mount the board so each axis can point straight up without rotating anything else.
2. **Gyro bias / temp**
   - Run `g` with the board perfectly still.
   - Repeat at 2–3 temperatures if you want `kGyroTempBiasSlopeRadPerSecPerC[3]`.
3. **Accel faces**
   - Capture `x`/`X`/`y`/`Y`/`z`/`Z` sequentially.
   - Verify each capture passes the `PrintAccelFaceTable` quality check (`"captured"`).
4. **Mag sweep**
   - Start magnetometer capture with `m`.
   - Rotate slowly through all orientations (pan, tilt, spin) for at least 20 seconds.
   - Stop with `m`; check the sample count is ≥32 and span covers full axes.
5. **Data dump**
   - Run `p` to print the paste-ready block; copy into `src/settings.h`.
   - Confirm printed `kAccelAinv`, `kMountRotation`, and mount quaternion make sense (not identity unless intentional).
6. **Optional gyro matrix**
   - Use a rate table or controlled rotation rig to drive each axis at known rates.
   - Fit `kGyroAinv[3][3]` offline; update `src/settings.h` accordingly only when the evidence supports it.

Keep this checklist near the bench so you can tick each step before the next flight.
