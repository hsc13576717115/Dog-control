#!/usr/bin/env python3
"""Verify explicit known-plane initialization, using simulation and a 50 mm slab."""

import argparse
from pathlib import Path
import subprocess
import sys
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--domain", type=int, default=142)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    control = Path("src/custom_dog_control/config")
    model = yaml.safe_load((control / "precision_model.yaml").read_text())
    model["model"]["initial_height_m"] += 0.05
    model["model"]["initial_support_height_m"] = 0.05
    controllers = yaml.safe_load((control / "controllers.yaml").read_text())
    nominal = controllers["nmpc_wbc_controller"]["ros__parameters"][
        "nominal_joint_positions"
    ]
    startup = {
        "joint_names": [
            f"{leg}_{joint}_joint"
            for leg in ("FR", "FL", "RR", "RL")
            for joint in ("hip", "thigh", "calf")
        ],
        "joint_positions": nominal,
        "euler_zyx": [0.0, 0.0, 0.0],
    }
    fixture = {
        "version": "qr-coordinated-local-v1",
        "pads": [{"id": 1, "center": [0.0, 0.0], "size": [2.0, 2.0], "height": 0.05}],
    }
    template = yaml.safe_load(
        Path("src/qr_planning/config/flat_shift.yaml").read_text()
    )
    for step in template["steps"]:
        step["surface_id"] = 1
    for name, data in (
        ("model", model),
        ("startup", startup),
        ("fixture", fixture),
        ("template", template),
    ):
        (args.output / f"{name}.yaml").write_text(yaml.safe_dump(data))
    return subprocess.run(
        [
            sys.executable,
            str(Path(__file__).with_name("run_sequence.py")),
            "--domain",
            str(args.domain),
            "--coordinated",
            "--solver-iterations",
            "400",
            "--control-config",
            str(control / "precision_coordinated.yaml"),
            "--model-config",
            str(args.output / "model.yaml"),
            "--startup-file",
            str(args.output / "startup.yaml"),
            "--fixture-file",
            str(args.output / "fixture.yaml"),
            "--template",
            str(args.output / "template.yaml"),
            "--output",
            str(args.output / "sequence"),
        ],
        check=False,
    ).returncode


if __name__ == "__main__":
    raise SystemExit(main())
