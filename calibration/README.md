# Sensor Calibration

This folder contains standalone calibration targets that are built separately from the flight firmware.

Targets:

- `icm20948_advanced.cpp`
- `lsm9ds1_advanced.cpp`
- `pulse20_advanced.cpp`

Build and upload:

```bash
platformio run -e icm_calibration -t upload
platformio run -e lsm9ds1_calibration -t upload
platformio run -e pulse20_calibration -t upload
platformio device monitor -b 115200
```

The serial programs print calibration data in `settings::sensors::*` format so the results can be pasted into [`src/settings.h`](../src/settings.h).

For the ICM target, repeat the `g` gyro-bias capture at two or more different temperatures if you want it to fit
`kGyroTempBiasSlopeRadPerSecPerC[3]` instead of printing zeros.

Useful ICM commands:

- `w` prints the recommended capture workflow
- `p` prints the paste-ready settings block and tells you exactly where it goes
- `d` prints a more detailed dump with capture quality, raw face captures, mag sweep extents, and gyro temp points

Useful LSM commands:

- `w` prints the recommended capture workflow
- `p` prints the paste-ready settings block and tells you exactly where it goes
- `d` prints a more detailed dump with capture quality, raw face captures, mag sweep extents, and gyro temp points

Useful Pulse20 commands:

- `w` prints the recommended capture workflow
- `p` prints the paste-ready settings block for `settings::sensors::ellipse20`
- `d` prints a more detailed dump with capture quality, face captures, mag sweep extents, and gyro temp points
