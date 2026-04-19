import math
import unittest
from pathlib import Path

from python.apogee_predictor_sim import (
    ReplayRow,
    SaferSeedConfig,
    build_seed_legacy,
    build_seed_safer,
    compute_actual_apogee,
    horizontal_speed_cap,
    interp_axis,
    legacy_horizontal_speed,
    load_force_table,
    parse_replay_rows,
    simulate_predictions,
)


ROOT = Path(__file__).resolve().parents[1]


class ApogeePredictorSimTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.table = load_force_table(ROOT / "lib" / "cfd.csv")

    def test_legacy_horizontal_speed_blows_up_at_large_tilt(self) -> None:
        vertical_velocity = 120.0
        zenith_rad = math.radians(80.0)
        legacy = legacy_horizontal_speed(vertical_velocity, zenith_rad)
        self.assertGreater(legacy, 600.0)

    def test_safer_seed_caps_horizontal_speed(self) -> None:
        cfg = SaferSeedConfig(max_horizontal_speed_mps=65.0, max_seed_zenith_deg=20.0)
        row = ReplayRow(
            time_s=10.0,
            status="coast",
            altitude_m=1000.0,
            vertical_velocity_mps=120.0,
            inertial_ax_mps2=25.0,
            inertial_ay_mps2=25.0,
            zenith_rad=math.radians(80.0),
            logged_apogee_m=1500.0,
        )
        cap = horizontal_speed_cap(row, cfg)
        seed = build_seed_safer(row, angular_rate_rad_s=4.0, horizontal_v_mps=200.0, config=cfg)
        self.assertLessEqual(seed.horizontal_v, cfg.max_horizontal_speed_mps)
        self.assertLessEqual(seed.horizontal_v, cap)
        self.assertLessEqual(abs(seed.zenith), math.radians(cfg.max_seed_zenith_deg))
        self.assertLessEqual(abs(seed.angular_v), cfg.max_seed_rate_rad_s)

    def test_small_tilt_preserves_reasonable_seed(self) -> None:
        row = ReplayRow(
            time_s=10.0,
            status="coast",
            altitude_m=1000.0,
            vertical_velocity_mps=100.0,
            inertial_ax_mps2=0.1,
            inertial_ay_mps2=0.2,
            zenith_rad=math.radians(3.0),
            logged_apogee_m=1500.0,
        )
        legacy_seed, legacy_h = build_seed_legacy(row, angular_rate_rad_s=0.05)
        safer_seed = build_seed_safer(row, angular_rate_rad_s=0.05, horizontal_v_mps=5.0, config=SaferSeedConfig())
        self.assertAlmostEqual(legacy_seed.vertical_v, safer_seed.vertical_v)
        self.assertAlmostEqual(legacy_h, 100.0 * math.tan(math.radians(3.0)), delta=0.25)
        self.assertAlmostEqual(safer_seed.horizontal_v, 5.0, delta=1.0e-6)

    def test_interp_axis_clamps_at_table_edges(self) -> None:
        grid = [0.675, 0.7]

        lower, upper, t_low = interp_axis(grid, 0.6)
        self.assertEqual((lower, upper), (0, 1))
        self.assertEqual(t_low, 0.0)

        lower, upper, t_high = interp_axis(grid, 1.2)
        self.assertEqual((lower, upper), (0, 1))
        self.assertEqual(t_high, 1.0)

    def test_replay_smoke_test(self) -> None:
        rows = parse_replay_rows(ROOT / "tools" / "replay" / "output.csv")
        self.assertGreater(len(rows), 1000)
        actual_apogee = compute_actual_apogee(rows)
        trace = simulate_predictions(rows[:50000], self.table, sample_stride=200)
        self.assertGreater(len(trace), 5)
        for sample in trace[:5]:
            self.assertTrue(math.isfinite(sample.legacy_apogee_m))
            self.assertTrue(math.isfinite(sample.safer_apogee_m))
        self.assertGreater(actual_apogee, 1400.0)


if __name__ == "__main__":
    unittest.main()
