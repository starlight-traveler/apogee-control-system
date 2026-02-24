# Local ImGui Telemetry Receiver

This app listens for binary UDP telemetry packets from the Teensy firmware and renders live values with Dear ImGui.

## Packet source

The firmware sends `telemetry::PacketV1` (see `include/telemetry_packet.h`) at a fixed rate.

Default network settings in firmware (`src/settings.h`):
- AP mode enabled
- AP credentials from `lib/arduino_secrets.h`
- Teensy UDP local port: `5006`
- Ground-station UDP listen port: `5005`
- Ground-station target IP: `192.168.4.2`

## Build requirements

Install these development packages (via your package manager or vcpkg):
- `glfw3`
- `imgui` with OpenGL3+GLFW backends
- OpenGL headers/libs
- CMake 3.20+

## Build and run

```bash
cd local-imgui-telemetry
cmake -S . -B build
cmake --build build -j
./build/teensy_imgui_telemetry 5005 192.168.4.1 5006
```

Arguments:
- `arg1`: UDP listen port for telemetry (default `5005`)
- `arg2`: Teensy IP for heartbeat/subscription (default `192.168.4.1`)
- `arg3`: Teensy UDP port for heartbeat (default `5006`)

The app sends a heartbeat every 200 ms. Firmware streams telemetry only while these heartbeats are received.

Current stream includes raw sensor vectors (BNO/ICM accel, gyro, quaternion), full filtered XYZ state vectors, servo command/effective angle, and altitude AGL.

## Voice control (optional)

You can enable voice control from the ImGui panel with:
- Wake word: `ACS` or `Apogee` (optional now for command phrases)
- Example commands:
  - `move 30`
  - `set angle 45`
  - `actuate thirty`
  - `three zero`
  - `full` (maps to 60)
  - `half` (maps to 30)
- Safety cap: command is clamped to `0..60` degrees
- Auto commands:
  - `return to auto`
  - `auto off`
  - `disable automatic`

The UI toggle launches `voice_listener.py`, which requires:
- Python package `vosk`
- Python package `sounddevice`
- A local Vosk English model directory

Arch/PEP668-safe setup (recommended):
```bash
cd local-imgui-telemetry
python3 -m venv .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install vosk sounddevice

# Download a Vosk model (example):
mkdir -p models
cd models
wget https://alphacephei.com/vosk/models/vosk-model-en-us-0.22.zip
unzip vosk-model-en-us-0.22.zip
cd ..

# Point to model:
export VOSK_MODEL_PATH=/path/to/vosk-model-en-us-0.22
```

Notes:
- The app now auto-prefers `.venv/bin/python3` for voice mode if present.
- You can override interpreter explicitly with `ACS_VOICE_PYTHON=/path/to/python3`.
- The build copies `voice_listener.py` next to the executable; runtime can also be overridden with `ACS_VOICE_SCRIPT=/path/to/voice_listener.py`.
- If dependencies/model are missing, the UI shows an `ERROR:` message in the voice section.

CMake bootstrap option:
- Create/update the local voice venv manually:
```bash
cmake -S . -B build
cmake --build build --target voice_env
```
- Download/unpack the Vosk model into the runtime `models/` folder:
```bash
cmake -S . -B build
cmake --build build --target voice_model
```
- Or enable automatic venv bootstrap during normal builds:
```bash
cmake -S . -B build -DACS_SETUP_VOICE_ENV=ON
cmake --build build -j
```
- Or enable both auto venv + auto model download:
```bash
cmake -S . -B build -DACS_SETUP_VOICE_ENV=ON -DACS_SETUP_VOICE_MODEL=ON
cmake --build build -j
```

By default, bootstrap is off, so builds remain fast and offline-friendly unless you opt in.

## Latency behavior

- UDP receive is done in a dedicated thread.
- UI thread consumes only the latest packet snapshot.
- Packet loss is tracked by sequence number.
- No file I/O is performed in the hot receive path.
