import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


class USVMavlinkContractTests(unittest.TestCase):
    def test_payload_struct_caches_baseline_fields(self):
        rover_h = (REPO_ROOT / "Rover" / "Rover.h").read_text(encoding="utf-8")

        self.assertIn("float baseline_set;", rover_h)
        self.assertIn("float reference_voltage;", rover_h)
        self.assertIn("float baseline_voltage;", rover_h)

    def test_named_value_handler_accepts_baseline_fields_and_completion_only_updates_nav(self):
        gcs = (REPO_ROOT / "Rover" / "GCS_MAVLink_Rover.cpp").read_text(encoding="utf-8")

        self.assertIn('"USV_BSET"', gcs)
        self.assertIn('"USV_REF"', gcs)
        self.assertIn('"USV_BASE"', gcs)
        self.assertIn('"USV_DONE"', gcs)

    def test_scheduler_forwards_sixteen_payload_fields(self):
        sensors = (REPO_ROOT / "Rover" / "sensors.cpp").read_text(encoding="utf-8")

        for name in ("USV_BSET", "USV_REF", "USV_BASE"):
            self.assertIn(f'gcs().send_named_float("{name}"', sensors)

    def test_baseline_telemetry_names_are_not_used_as_auto_commands(self):
        mode_auto = (REPO_ROOT / "Rover" / "mode_auto.cpp").read_text(encoding="utf-8")

        self.assertNotIn('"USV_BASE"', mode_auto)


if __name__ == "__main__":
    unittest.main()
