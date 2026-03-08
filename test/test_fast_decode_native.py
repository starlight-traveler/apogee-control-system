import csv
import struct
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DECODER_SRC = ROOT / "tools" / "decode" / "native" / "fast_decode_sensor_log.cpp"
DECODER_BIN = Path(tempfile.gettempdir()) / "acs_fast_decode_test_bin"

K_SENSOR_FLOAT_COUNT = 33
K_STATE_FLOAT_COUNT = 15


def _build_decoder() -> None:
    subprocess.run(
        [
            "clang++",
            "-std=c++17",
            "-I.",
            "-I./src",
            "-I./tools/decode/native",
            str(DECODER_SRC),
            "-o",
            str(DECODER_BIN),
        ],
        cwd=ROOT,
        check=True,
    )


def _make_preamble(schema_version: int) -> bytes:
    firmware_hash = b"unknown".ljust(40, b"\0")
    return struct.pack("<8sHH40s12s", b"ACSNDRT1", 1, schema_version, firmware_hash, b"\0" * 12)


def _make_telemetry_record(status: int = 2, flags: int = 1) -> bytes:
    sensor_floats = [100.0 + float(i) for i in range(K_SENSOR_FLOAT_COUNT)]
    state_floats = [200.0 + float(i) for i in range(K_STATE_FLOAT_COUNT)]
    bool_bytes = bytes([1, 0, 1, 0])
    payload = struct.pack("<" + "f" * K_SENSOR_FLOAT_COUNT, *sensor_floats)
    payload += bool_bytes
    payload += struct.pack("<" + "f" * K_STATE_FLOAT_COUNT, *state_floats)
    return struct.pack("<BBBB", 0, status, flags, 0) + payload


def _read_csv_rows(path: Path):
    with path.open(newline="") as handle:
        data_lines = [line for line in handle if not line.startswith("#")]
    return list(csv.DictReader(data_lines))


class FastDecodeNativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        _build_decoder()

    def test_schema4_decode_includes_predictor_seed_columns(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            input_path = tmp / "synthetic.BIN"
            output_path = tmp / "synthetic.csv"
            input_path.write_bytes(_make_preamble(4) + _make_telemetry_record())

            result = subprocess.run(
                [str(DECODER_BIN), str(input_path), "-o", str(output_path), "--no-events-output"],
                cwd=ROOT,
                check=True,
                capture_output=True,
                text=True,
            )

            self.assertIn("Decoded telemetry records: 1", result.stdout)
            rows = _read_csv_rows(output_path)
            self.assertEqual(len(rows), 1)
            row = rows[0]
            self.assertEqual(row["flight_status"], "coast")
            self.assertEqual(row["flight_status_raw"], "2")
            self.assertEqual(row["has_filtered_state"], "True")
            self.assertEqual(row["sensor_predictor_seed_horizontal_speed_mps"], "129")
            self.assertEqual(row["sensor_predictor_seed_clamped_zenith_rad"], "130")
            self.assertEqual(row["sensor_predictor_seed_clamped_angular_rate_rad_per_sec"], "131")
            self.assertEqual(row["sensor_predictor_seed_confidence_flags"], "132")
            self.assertEqual(row["sensor_has_quaternion"], "True")
            self.assertEqual(row["sensor_has_icm_quaternion"], "False")
            self.assertEqual(row["sensor_has_icm_ypr"], "True")
            self.assertEqual(row["state_time"], "200")
            self.assertEqual(row["state_apogee_estimate"], "214")

    def test_schema_mismatch_warns_but_decodes(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            input_path = tmp / "schema3.BIN"
            output_path = tmp / "schema3.csv"
            input_path.write_bytes(_make_preamble(3) + _make_telemetry_record())

            result = subprocess.run(
                [str(DECODER_BIN), str(input_path), "-o", str(output_path), "--no-events-output"],
                cwd=ROOT,
                check=True,
                capture_output=True,
                text=True,
            )

            self.assertIn("Warning: Schema version mismatch: file=3 decoder=4", result.stdout)
            rows = _read_csv_rows(output_path)
            self.assertEqual(len(rows), 1)

    def test_truncated_telemetry_record_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            input_path = tmp / "truncated.BIN"
            output_path = tmp / "truncated.csv"
            good_record = _make_telemetry_record()
            input_path.write_bytes(_make_preamble(4) + good_record[:-11])

            result = subprocess.run(
                [str(DECODER_BIN), str(input_path), "-o", str(output_path), "--no-events-output"],
                cwd=ROOT,
                capture_output=True,
                text=True,
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("Truncated telemetry record.", result.stderr)


if __name__ == "__main__":
    unittest.main()
