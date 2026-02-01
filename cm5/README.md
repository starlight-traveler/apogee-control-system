# CM5 Flight Computer Host

Build and run the CM5-side flight computer that consumes Teensy telemetry and sends commands.

## Build
```sh
cmake -S cm5 -B cm5/build
cmake --build cm5/build
```

## Run
```sh
cm5/build/cm5_flight_computer /dev/ttyACM0 921600 cm5_telemetry.bin cfd.csv
```

The second argument is optional and sets the baud rate (defaults to 921600). The third argument sets the binary log file path. The fourth argument is the CFD table path.

## Optional PWM Command
```sh
cm5/build/cm5_flight_computer /dev/ttyACM0 921600 cm5_telemetry.bin cfd.csv 0 2048 400
```

Arguments 5-7 send a single PWM command at startup: `channel`, `duty` (0-4095), and optional `frequencyHz`.
