# Native Decode Tool

Native C++ decoder for ACS binary sensor logs.

Build from the repository root:

```bash
cmake -S tools -B tools/build -DACS_TOOLS_BUILD_DECODER=ON -DACS_TOOLS_ENABLE_NATIVE_CPP_GUI=OFF
cmake --build tools/build --target acs_ndrt_rocketry_fast_decode -j
```

Run:

```bash
tools/build/bin/acs_ndrt_rocketry_fast_decode path/to/SENS010.BIN -o tools/replay/data/SENS010.csv
```

For long logs, use smart trimming:

```bash
tools/build/bin/acs_ndrt_rocketry_fast_decode input.BIN -o output.csv --smart-parser \
  --smart-pre-seconds 3 --smart-post-seconds 10 \
  --smart-min-alt-ft 25 --smart-min-vel-ftps 20 --smart-min-cmd-deg 0.5
```

The Python decoder wrapper is `tools/replay/scripts/decode_log.py`. Use the native decoder when log size or decode speed matters.
