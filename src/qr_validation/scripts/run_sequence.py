#!/usr/bin/env python3
"""Independent M2 entry validation: finite four-foot repositioning, Gazebo only."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import time
import yaml
import rclpy
from run_precision import Probe, stop
from acceptance import load_thresholds


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--domain", type=int, default=141)
    parser.add_argument("--fault", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(
        ROS_DOMAIN_ID=str(args.domain),
        ROS_LOCALHOST_ONLY="1",
        GAZEBO_MODEL_DATABASE_URI="",
    )
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    os.environ["GAZEBO_MASTER_URI"] = f"http://127.0.0.1:{port}"
    report = dict(
        status="failed",
        hardware_tested=False,
        fault=args.fault,
        scope="four-foot flat repositioning; no continuous walking or obstacle completion",
    )
    report["git_commit"] = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], text=True
    ).strip()
    paths = (
        subprocess.check_output(
            ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"]
        )
        .decode()
        .split("\0")
    )
    report["source_hashes"] = {
        name: hashlib.sha256(Path(name).read_bytes()).hexdigest()
        for name in paths
        if name.startswith("src/")
        and "third_party/" not in name
        and Path(name).is_file()
        and Path(name).suffix
        in (".cpp", ".hpp", ".def", ".py", ".yaml", ".xml", ".msg", ".srv", ".action")
    }
    report["model_sha256"] = hashlib.sha256(
        Path(
            "external/model_ws/src/custom_dog_description/urdf/custom_dog.urdf"
        ).read_bytes()
    ).hexdigest()
    report["binary_hashes"] = {
        name: hashlib.sha256(Path(name).read_bytes()).hexdigest()
        for name in (
            "install/custom_dog_control/lib/libcustom_dog_control_nmpc_wbc_controller.so",
            "install/qr_planning/lib/qr_planning/finite_sequence",
        )
    }
    limits = load_thresholds()
    report["thresholds"] = limits
    launch = client = node = None
    try:
        log = (args.output / "launch.log").open("w")
        launch = subprocess.Popen(
            [
                "ros2",
                "launch",
                "qr_bringup",
                "precision_step.launch.py",
                "surface_mode:=ground",
                f"artifact_dir:={args.output.resolve()/'inputs'}",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        rclpy.init()
        node = Probe(0)
        if not node.wait(lambda: node.status is not None and node.status.ready, 100):
            raise RuntimeError("initial_stance_not_ready")
        template = Path("src/qr_planning/config/flat_shift.yaml").resolve()
        report["template_sha256"] = hashlib.sha256(template.read_bytes()).hexdigest()
        client = subprocess.Popen(
            [
                "ros2",
                "run",
                "qr_planning",
                "finite_sequence",
                "--ros-args",
                "-p",
                "use_sim_time:=true",
                "-p",
                f"template_file:={template}",
                "-p",
                f"result_file:={args.output.resolve()/'sequence.yaml'}",
            ],
            stdout=(args.output / "sequence.log").open("w"),
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        if args.fault:
            from physical_faults import inject

            report["injection"] = inject(node, "support_loss", 0)
        if not node.wait(lambda: client.poll() is not None, 160):
            raise RuntimeError("sequence_client_timeout")
        node.wait(lambda: False, 0.5)
        summary = yaml.safe_load((args.output / "sequence.yaml").read_text())
        report["client"] = summary
        ids = {
            s["plan_id"]
            for s in node.trace
            if s["phase"] in ("SHIFT", "SWING", "RESTORE", "DONE")
        }
        if args.fault:
            from physical_faults import witness

            if not witness(
                node.trace, "support_loss", report["injection"]["sim_stamp"]
            ):
                raise RuntimeError("physical_support_loss_not_witnessed")
            if (
                client.returncode == 0
                or summary["completed_steps"] != 0
                or len(ids) != 1
            ):
                raise RuntimeError("sequence_advanced_after_abort")
        else:
            if (
                client.returncode != 0
                or not summary["success"]
                or summary["completed_steps"] != 4
            ):
                raise RuntimeError("sequence_incomplete: " + str(summary.get("error")))
            results = []
            for entry in summary["steps"]:
                leg = entry["foot"]
                plan = entry["plan_id"]
                samples = [
                    s
                    for s in node.trace
                    if s["plan_id"] == plan
                    and s["phase"]
                    in ("SHIFT", "SWING", "CONTACT_CONFIRM", "RESTORE", "DONE")
                ]
                done = [s for s in samples if s["phase"] == "DONE"]
                if not done:
                    raise RuntimeError("missing_done_witness")
                s = done[0]
                name = "custom_dog::" + ["FR", "FL", "RR", "RL"][leg] + "_foot"
                error = math.dist(s["truth_feet"][name], s["targets"][leg])
                slip = max(
                    s["contact_slip_integral_m"][i]
                    - samples[0]["contact_slip_integral_m"][i]
                    for i in range(4)
                    if i != leg
                )
                if error > limits["max_error_m"] or slip > limits["max_contact_slip_m"]:
                    raise RuntimeError("sequence_step_error_or_slip")
                if not all(
                    s["truth_contacts"].get(l) for l in ("FR", "FL", "RR", "RL")
                ):
                    raise RuntimeError("step_finished_without_four_true_contacts")
                if any(any(x["nonfoot_contacts"].values()) for x in samples):
                    raise RuntimeError("sequence_body_or_leg_collision")
                displacement = 0.0
                tilt = 0.0
                for support in range(4):
                    if support == leg:
                        continue
                    name = "custom_dog::" + ["FR", "FL", "RR", "RL"][support] + "_foot"
                    origin = samples[0]["truth_feet"][name]
                    displacement = max(
                        displacement,
                        max(math.dist(origin, x["truth_feet"][name]) for x in samples),
                    )
                for sample in samples:
                    x, y, z, w = sample["truth_body_quaternion"]
                    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
                    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
                    tilt = max(tilt, math.degrees(abs(roll)), math.degrees(abs(pitch)))
                if (
                    displacement > limits["max_support_slip_m"]
                    or tilt > limits["max_tilt_deg"]
                ):
                    raise RuntimeError("sequence_support_displacement_or_tilt")
                results.append(
                    dict(
                        foot=leg,
                        plan_id=plan,
                        error_m=error,
                        support_contact_slip_m=slip,
                        support_foot_center_displacement_m=displacement,
                        max_tilt_deg=tilt,
                    )
                )
            report["steps"] = results
        report["status"] = "passed"
    except Exception as e:
        report["error"] = str(e)
    finally:
        if client and client.poll() is None:
            stop(client)
        if node:
            (args.output / "trace.json").write_text(json.dumps(node.trace))
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if launch:
            stop(launch)
        report["input_hashes"] = {
            p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (args.output / "inputs").glob("*")
            if p.is_file()
        }
        (args.output / "result.json").write_text(json.dumps(report, indent=2))
        print(
            json.dumps(
                {k: v for k, v in report.items() if k != "source_hashes"}, indent=2
            ),
            flush=True,
        )
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
