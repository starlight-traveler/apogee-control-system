# Firmware Pad Notes

This folder is the flight firmware. The main thing to keep straight on launch day
is which values are meant to be updated at the pad and which ones are baked into
the build.

`ACSCFG.TXT` on the SD card holds the runtime settings. Those can also be pushed
over telemetry, but only while the flight computer is still in `Ground`.

## What To Check At The Pad

### Environment

These feed the atmosphere and wind model used by the predictor:

- `environment.ground_temperature_f`
- `environment.wind_speed_mph`
- `environment.wind_direction_deg`
- `environment.launch_direction_deg`
- `environment.measurement_height_m`

Use actual field values here.

These usually do not need to change unless the site assumptions are wrong:

- `environment.roughness_length_m`
- `environment.gradient_height_m`

### Vehicle

These should match the loaded rocket:

- `vehicle.dry_mass_kg`
- `vehicle.center_of_pressure_offset_m`
- `vehicle.moment_of_inertia_kg_m2`

In practice:

- update `dry_mass_kg` if the loaded vehicle mass changed
- update `center_of_pressure_offset_m` only if the aero setup changed
- update `moment_of_inertia_kg_m2` only if the mass distribution changed enough to matter

## Things The Firmware Handles On Its Own

- Pad altitude reference
  The firmware takes the first good barometric altitude and uses that as the AGL zero.
- Sensor rail health
  Rail agreement and freshness are evaluated online.

## Things That Are Not Pad Settings

These live in `settings.h` and are compile-time values, not things you should be
editing in the field:

- apogee target
- liftoff / burnout / descent thresholds
- servo timing and flap limits
- sensor mounting and rail selection

## Quick Pad Flow

1. Make sure the computer is still in `Ground`.
2. Make sure pad altitude has been captured.
3. Update:
   `ground_temperature_f`, `wind_speed_mph`, `wind_direction_deg`,
   `launch_direction_deg`, `measurement_height_m`, and `dry_mass_kg`.
4. Leave `roughness_length_m`, `gradient_height_m`, `center_of_pressure_offset_m`,
   and `moment_of_inertia_kg_m2` alone unless there is a real reason to change them.
5. Check that the live log still shows healthy primary rails before arming.

## Current Default Runtime Values

From `settings.h`:

- `ground_temperature_f = 42.0`
- `wind_speed_mph = 13.0`
- `wind_direction_deg = 317.0`
- `launch_direction_deg = 317.0`
- `roughness_length_m = 0.075`
- `gradient_height_m = 300.0`
- `measurement_height_m = 10.0`
- `center_of_pressure_offset_m = 0.4389`
- `moment_of_inertia_kg_m2 = 8.28`
- `dry_mass_kg = 18.09975`

## Sea-Level Pressure

Sea-level pressure still is not a runtime setting. It is compile-time data in
`settings.h`.

There are three separate pressure references in the code:

- `settings::sensors::bmp585::kSeaLevelPressureHpa = 1022.689`
- `settings::sensors::ms5611::kSeaLevelPressureHpa = 1018.8`
- `settings::predictor::kSeaLevelPressurePa = 101325.0`

What that means:

- if you care about absolute pad altitude, the barometer pressure reference should match the day
- flight logic mostly works off captured AGL, so it is less sensitive to this than the absolute displayed altitude
- the predictor uses its own fixed sea-level pressure, separate from the barometer conversion

If you change pressure in code before a flight, update every place that matters
for the active sensors and predictor, not just one constant.

## Misc

- runtime settings are validated before they are applied
- runtime settings are rejected once the vehicle leaves `Ground`
- `BNO055` and `WT901` are comparison rails right now, not critical flight rails
- this build uses `LSM` as the primary raw accel/gyro rail, with `ICM` as fallback
