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

Required:
- OpenGL headers/libs
- CMake 3.20+

Default CMake behavior now fetches/builds:
- `glfw`
- `imgui`
- `whisper.cpp` when native voice is enabled

On macOS, the build prefers a Homebrew `portaudio` install before falling back
to `FetchContent`.

## Build and run

```bash
cd telemetry/imgui
cmake -S . -B build
cmake --build build -j
./build/bin/teensy_imgui_telemetry 5005 192.168.4.1 5006
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

Voice control now runs natively inside the C++ app.

Native dependencies:
- `whisper.cpp`
- a local Whisper model file

CMake behavior:
- `portaudio` is fetched automatically by default
- `whisper.cpp` is fetched automatically by default
- models are runtime files, not compile-time SDK dependencies

Example:
```bash
cd telemetry/imgui
cmake -S . -B build
cmake --build build -j
```

Inside the GUI you can:
- choose a Whisper model preset
- download the selected model
- switch the active voice model path

Default downloaded model location:
- `build/bin/models` at runtime, or `models/` next to the executable if present

If you want to use system-installed packages instead of `FetchContent`:

```bash
brew install portaudio
cmake -S . -B build \
  -DACS_FETCH_GLFW=OFF \
  -DACS_FETCH_IMGUI=OFF \
  -DACS_FETCH_PORTAUDIO=OFF
cmake --build build -j
```

Model setup:
```bash
mkdir -p models
cd models
wget https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-base.en.bin
cd ..
export WHISPER_MODEL_PATH=/path/to/ggml-base.en.bin
```

Notes:
- If `whisper.cpp` or `portaudio` are not available, the app still builds but voice mode is disabled.
- The UI shows the current model path and any native voice initialization error.
- You can still use `voice_model` to download the default model into the runtime `models/` folder:
```bash
cmake -S . -B build -DACS_SETUP_VOICE_MODEL=ON
cmake --build build -j
```

## Latency behavior

- UDP receive is done in a dedicated thread.
- UI thread consumes only the latest packet snapshot.
- Packet loss is tracked by sequence number.
- No file I/O is performed in the hot receive path.
