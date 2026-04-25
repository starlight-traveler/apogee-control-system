# Hosted Replay Include Shims

Headers in this directory let the hosted replay build compile firmware code outside the Arduino/Teensy runtime.

Current shim:

- `Arduino.h`: minimal `Serial`, GPIO, delay, and constant definitions used by replayed firmware modules.

Keep these shims narrow. They should provide just enough compatibility for `tools/replay/main.cpp` and reused `src/` code to build on a desktop machine. Do not put flight behavior here.

