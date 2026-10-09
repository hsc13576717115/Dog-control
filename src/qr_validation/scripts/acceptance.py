"""Independent acceptance contract; no ROS or controller imports."""

import math
from pathlib import Path
import yaml

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config/acceptance.yaml"
KEYS = {
    "max_error_m",
    "p95_error_m",
    "max_support_slip_m",
    "max_contact_slip_m",
    "max_tilt_deg",
    "minimum_success_rate",
    "minimum_trials_per_foot",
}


def load_thresholds(path=DEFAULT_CONFIG):
    config = yaml.safe_load(Path(path).read_text())
    if not isinstance(config, dict) or set(config) != KEYS:
        raise ValueError("acceptance threshold keys differ from the contract")
    for key, value in config.items():
        if (
            isinstance(value, bool)
            or not isinstance(value, (float, int))
            or not math.isfinite(value)
            or value <= 0
        ):
            raise ValueError(f"invalid acceptance threshold: {key}")
    if (
        config["minimum_success_rate"] > 1
        or config["p95_error_m"] > config["max_error_m"]
    ):
        raise ValueError("inconsistent acceptance thresholds")
    if not isinstance(config["minimum_trials_per_foot"], int):
        raise ValueError("minimum_trials_per_foot must be an integer")
    return config
