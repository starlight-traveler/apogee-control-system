## Summary

- 

## Affected Area

- [ ] Flight firmware
- [ ] Calibration / bench tooling
- [ ] Decode / replay tooling
- [ ] Python analysis
- [ ] Documentation only

## Validation

- [ ] `pio run -e flight`
- [ ] `pio run -e debug`
- [ ] `cmake -S tools -B tools/build -DACS_TOOLS_ENABLE_NATIVE_CPP_GUI=OFF`
- [ ] `cmake --build tools/build --target acs_replay -j`
- [ ] Hardware or bench check
- [ ] Replay against recorded data

Notes:

## Flight-Safety Notes

State whether this changes estimator behavior, apogee prediction, actuation, sensor frames, units, calibration, or runtime settings.
