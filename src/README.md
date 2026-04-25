# Firmware Pad Notes

This folder is the flight firmware. The main thing to keep straight on launch day
is which values are meant to be updated at the pad and which ones are baked into
the build.

The flashed defaults in `settings.h` are authoritative at boot. `ACSCFG.TXT` on
the SD card is overwritten as a mirror of those flashed values when storage is
available, so an old SD file cannot silently override a new firmware flash.
Runtime settings can still be pushed over telemetry while the flight computer is
in `Ground`; those take effect for the running boot and are saved, but the next
boot returns to the flashed defaults.

## What To Check At The Pad

### Environment

These feed the atmosphere and wind model used by the predictor:

- `environment.ground_temperature_f`
- `environment.sea_level_pressure_hpa`
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
   `ground_temperature_f`, `sea_level_pressure_hpa`, `wind_speed_mph`, `wind_direction_deg`,
   `launch_direction_deg`, `measurement_height_m`, and `dry_mass_kg`.
4. Leave `roughness_length_m`, `gradient_height_m`, `center_of_pressure_offset_m`,
   and `moment_of_inertia_kg_m2` alone unless there is a real reason to change them.
5. Check that the live log still shows healthy primary rails before arming.

## Current Default Runtime Values

From `settings.h`:

- `ground_temperature_f = 53.0`
- `sea_level_pressure_hpa = 1030.9`
- `wind_speed_mph = 8.0`
- `wind_direction_deg = 111.0`
- `launch_direction_deg = 111.0`
- `roughness_length_m = 0.075`
- `gradient_height_m = 300.0`
- `measurement_height_m = 10.0`
- `center_of_pressure_offset_m = 0.22`
- `moment_of_inertia_kg_m2 = 0.529`
- `dry_mass_kg = 3.33`

## Sea-Level Pressure

Sea-level pressure is now a runtime setting:

- `environment.sea_level_pressure_hpa`

That single value drives:

- `BMP585` altitude conversion
- `MS5611` altitude conversion
- predictor atmosphere pressure reference

If you change it on the pad while still in `Ground`, the firmware also clears
the captured pad altitude and re-baselines the ground-reference altitude on the
next valid barometer sample.

## Misc

- runtime settings are validated before they are applied
- runtime settings are rejected once the vehicle leaves `Ground`
- `BNO055` and `WT901` are comparison rails right now, not critical flight rails
- this build uses `LSM` as the primary raw accel/gyro rail, with `ICM` as fallback
