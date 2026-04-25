# Decode Tools

Binary telemetry decode tools.

There are two decoder paths:

| Decoder | Path | Use |
| ------- | ---- | --- |
| Python decoder | `tools/replay/scripts/decode_log.py` | Portable, easy to inspect, good default for normal logs. |
| Native decoder | `tools/decode/native/` | Faster C++ decoder for large logs or batch work. |

Both decoders read binary logs produced by `src/data_logger.*` and write telemetry CSV. Event JSON is produced by the Python decoder and by native paths where supported.

## Native Decoder

Build from the repository root through the shared tools workspace:

```bash
cmake -S tools -B tools/build -DACS_TOOLS_BUILD_DECODER=ON -DACS_TOOLS_ENABLE_NATIVE_CPP_GUI=OFF
cmake --build tools/build --target acs_ndrt_rocketry_fast_decode -j
```

Run it against a flight log:

```bash
tools/build/bin/acs_ndrt_rocketry_fast_decode path/to/SENS010.BIN -o output.csv
```

The Python decoder entry point lives at [`../replay/scripts/decode_log.py`](../replay/scripts/decode_log.py):

```bash
python3 tools/replay/scripts/decode_log.py path/to/SENS010.BIN -o output.csv
```
