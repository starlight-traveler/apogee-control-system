# Include Directory

Shared headers that are not owned by a single firmware module live here.

- `telemetry_packet.h`: packet layout shared by firmware, decode tools, and telemetry consumers.
- `icm20948_sensor_old.*`: retained legacy/reference implementation for the ICM20948 path.

Most flight code should keep module-private declarations in `src/`. Put headers here only when multiple build targets or host tools need the same interface.
