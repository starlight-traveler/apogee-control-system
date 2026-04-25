# Libraries And Tables

This directory contains vendored libraries, locally patched library code, and data tables that are compiled or loaded by the firmware/tooling.

Important entries:

- `cfd.csv`: aerodynamic lookup table used by the apogee prediction/control path.
- `Fusion/`: AHRS support code used by calibration and comparison workflows.
- `SparkFunLSM9DS1_SPI1/`: local LSM9DS1 driver copy.
- `WiFiNINA/`: vendored networking dependency.
- `sbgECom/`: SBG/Ellipse support library.

Do not put new application logic here unless it is intentionally being packaged as a reusable library. Flight behavior normally belongs in `src/`, calibration programs in `calibration/`, and host-side replay code in `tools/`.
