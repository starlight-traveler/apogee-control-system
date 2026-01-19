# Tools

## tui_dashboard.py

Unified TUI dashboard for choosing CSV, JSON, or binary log inputs.

```bash
python tools/tui_dashboard.py
```

On first launch the dashboard asks for a data root directory and stores it
in `tools/.tui_settings.json`. You can change it later from the Settings
menu.

The dashboard also includes a Rocketry Analysis panel that summarizes
flight metrics (apogee, max velocity, burn/coast/descent timing) and can
estimate thrust-to-weight and drag if you provide mass and reference area
in Settings.

## decode_sensor_log.py

Decode binary telemetry logs written by the flight computer into CSV + JSON.

```bash
python tools/decode_sensor_log.py data/subscale_3/SENS010.BIN \
  -o data/subscale_3/sens010.csv
```

## tui_telemetry_viewer.py

Scrollable TUI for telemetry CSV files (arrow keys, PgUp/PgDn, q to quit).

```bash
python tools/tui_telemetry_viewer.py data/subscale_3/sens010.csv
```

Pick specific columns:

```bash
python tools/tui_telemetry_viewer.py data/subscale_3/sens010.csv \
  --columns state_time,state_apogee_estimate,flight_status
```

## tui_tail_csv.py

Live tail view of the latest CSV rows.

```bash
python tools/tui_tail_csv.py data/subscale_3/sens010.csv --rows 15 --interval 0.5
```

## tui_event_viewer.py

Scrollable TUI for event JSON files produced by `decode_sensor_log.py`.

```bash
python tools/tui_event_viewer.py data/events/sample_events.json
```
