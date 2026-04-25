# Tests

This directory is reserved for automated checks that can run outside the vehicle.

Current practical checks are:

- firmware builds through PlatformIO environments in `platformio.ini`
- hosted replay builds through `tools/CMakeLists.txt`
- replay/plot scripts under `tools/replay/scripts/`

Add focused tests here when a behavior can be checked without hardware. Flight-critical changes should still be validated with replay, bench tests, and hardware-in-loop checks where applicable.
