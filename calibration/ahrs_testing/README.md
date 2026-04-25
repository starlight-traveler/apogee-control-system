# AHRS Testing

Standalone Teensy 4.1 target for streaming both the LSM9DS1 and ICM-20948 through separate `xioTechnologies/Fusion` AHRS instances.

## Build and upload

```bash
pio run -e ahrs_testing -t upload
```

## Monitor serial output

```bash
pio device monitor -e ahrs_testing
```

The sketch prints CSV at 115200 baud about every 50 ms after both sensors initialize.

## Output fields

`lsm_ax_g..lsm_az_g`
Calibrated LSM9DS1 accelerometer values in g, remapped into the body frame using the existing `src/settings.h` calibration and mount rotation.

`lsm_raw_mx,lsm_raw_my,lsm_raw_mz`
Raw LSM9DS1 magnetometer register values. These should change when you rotate the board.

`lsm_mx,lsm_my,lsm_mz`
Calibrated LSM9DS1 magnetometer vector in the same body frame used by Fusion.

`lsm_roll_deg,lsm_pitch_deg,lsm_yaw_deg`
LSM9DS1 Fusion Euler output.

`icm_ax_g..icm_az_g`
Calibrated ICM-20948 accelerometer values in g, remapped into the same body frame.

`icm_raw_mx,icm_raw_my,icm_raw_mz`
Raw ICM-20948 magnetometer register values.

`icm_mx,icm_my,icm_mz`
Calibrated ICM-20948 magnetometer vector in the same body frame used by Fusion.

`icm_roll_deg,icm_pitch_deg,icm_yaw_deg`
ICM-20948 Fusion Euler output.

`delta_roll_deg,delta_pitch_deg,delta_yaw_deg`
LSM minus ICM angle deltas, wrapped to `[-180, 180]` for easier comparison.
