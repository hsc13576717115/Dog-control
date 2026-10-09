import importlib.util
import re
import sys
import tempfile
import unittest
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from acceptance import DEFAULT_CONFIG, load_thresholds


class Contracts(unittest.TestCase):
    def test_wire_error_codes_match_native(self):
        native = (
            ROOT
            / "src/custom_dog_control/include/custom_dog_control/precision/PrecisionError.def"
        ).read_text()
        wire = (ROOT / "src/qr_interfaces/msg/ExecutionError.msg").read_text()
        cpp = dict(re.findall(r"QR_ERROR\((\w+), (\d+),", native))
        ros = dict(re.findall(r"uint16 (\w+)=(\d+)", wire))
        self.assertEqual(cpp, ros)
        self.assertEqual(len(cpp), len(set(cpp.values())))
        phase_cpp = (
            ROOT
            / "src/custom_dog_control/include/custom_dog_control/precision/PrecisionTypes.hpp"
        ).read_text()
        phase_ros = (ROOT / "src/qr_interfaces/msg/ExecutionStatus.msg").read_text()
        self.assertEqual(
            dict(
                re.findall(
                    r"(STANCE|SHIFT|SWING|CONFIRM|RESTORE|HOLD|DONE)\s*=\s*(\d+)",
                    phase_cpp,
                )
            ),
            dict(re.findall(r"PHASE_(\w+)=(\d+)", phase_ros)),
        )

    def test_acceptance_rejects_nonfinite_and_malformed(self):
        baseline = load_thresholds()
        for key, value in [
            ("max_error_m", float("nan")),
            ("max_tilt_deg", -1),
            ("minimum_success_rate", 1.1),
            ("minimum_trials_per_foot", 2.5),
            ("p95_error_m", baseline["max_error_m"] * 2),
        ]:
            with tempfile.TemporaryDirectory() as d:
                p = Path(d) / "bad.yaml"
                p.write_text(yaml.safe_dump(dict(baseline, **{key: value})))
                with self.assertRaises(ValueError):
                    load_thresholds(p)

    def test_runtime_layers_do_not_import_ros_or_nmpc_backend(self):
        folder = ROOT / "src/custom_dog_control/src/precision"
        for name in ["PrecisionExecutionCore.cpp", "PrecisionPlanner.cpp"]:
            s = (folder / name).read_text()
            self.assertNotIn("rclcpp", s)
            self.assertNotIn("qr_interfaces", s)
            self.assertNotIn("NmpcBackend", s)
        estimator = (
            ROOT
            / "src/custom_dog_control/include/custom_dog_control/nmpc/KinematicStateEstimator.hpp"
        )
        self.assertNotIn("precision/", estimator.read_text())

    def test_snapshot_payload_has_no_dynamic_text(self):
        s = (
            ROOT
            / "src/custom_dog_control/include/custom_dog_control/precision/PrecisionState.hpp"
        ).read_text()
        self.assertNotIn("std::string", s)
        self.assertNotIn("std::vector", s)


if __name__ == "__main__":
    unittest.main()
