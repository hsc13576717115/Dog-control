#!/usr/bin/env python3
"""Reproducible QR test entry. Simulation only; source the built workspace first."""

import argparse
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ("custom_dog_control", "qr_planning", "qr_course", "qr_validation")


def dependencies(simulation):
    missing = []
    for tool in ["ctest", "git"] + (["ros2", "gzserver", "gz"] if simulation else []):
        if shutil.which(tool) is None:
            missing.append(tool)
    for package in PACKAGES:
        if not (ROOT / "build" / package / "CTestTestfile.cmake").exists():
            missing.append(
                f"build/{package}/CTestTestfile.cmake (build with BUILD_TESTING=ON)"
            )
    for module in ["yaml"] + (
        ["rclpy", "qr_interfaces.action", "gazebo_msgs.msg"] if simulation else []
    ):
        try:
            if importlib.util.find_spec(module) is None:
                missing.append(module)
        except ModuleNotFoundError:
            missing.append(module)
    return missing


def run_case(name, command, output, timeout):
    started = time.monotonic()
    timed_out = False
    interrupted = False
    with (output / f"{name}.log").open("w") as log:
        proc = subprocess.Popen(
            command,
            cwd=ROOT,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            code = proc.wait(timeout=timeout)
        except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
            interrupted = isinstance(exc, KeyboardInterrupt)
            timed_out = not interrupted
            os.killpg(proc.pid, signal.SIGINT)
            try:
                code = proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                code = proc.wait()
    if interrupted:
        raise KeyboardInterrupt
    return dict(
        name=name,
        status="passed" if code == 0 and not timed_out else "failed",
        returncode=code,
        timed_out=timed_out,
        duration_s=time.monotonic() - started,
        command=command,
        log=f"{name}.log",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--suite",
        choices=["unit", "smoke", "faults", "regression", "matrix", "robustness"],
        default="unit",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--domain", type=int, default=121)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error("ROS domain must be 0..232")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    report = {
        "schema_version": 1,
        "suite": args.suite,
        "hardware_tested": False,
        "status": "running",
        "results": [],
        "not_covered": [
            "hardware acquisition timestamps and real solver WCET",
            "sensor noise, delays and full self-collision coverage",
            "NX and real motors",
            "full competition",
        ],
    }
    summary = args.output / "summary.json"

    def save():
        summary.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")

    try:
        report["git_commit"] = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip()
        report["worktree_dirty"] = bool(
            subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT)
        )
        report["missing_dependencies"] = dependencies(args.suite != "unit")
        if report["missing_dependencies"]:
            report["status"] = "blocked"
            save()
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 2
        cases = [
            (
                f"unit-{p}",
                ["ctest", "--test-dir", str(ROOT / "build" / p), "--output-on-failure"],
                120,
            )
            for p in PACKAGES
        ]
        script = str(ROOT / "src/qr_validation/scripts/run_precision.py")

        def trial(name, foot, height, scenario="normal", profile="nominal"):
            cases.append(
                (
                    name,
                    [
                        sys.executable,
                        script,
                        "--foot",
                        str(foot),
                        "--height",
                        str(height),
                        "--scenario",
                        scenario,
                        "--profile",
                        profile,
                        "--domain",
                        str(args.domain),
                        "--output",
                        str(args.output / name),
                    ],
                    180,
                )
            )

        if args.suite in ("smoke", "regression"):
            for foot in range(4):
                for height in (
                    [0, 0.03, 0.05] if args.suite == "regression" else [0.05]
                ):
                    trial(f"step-{foot}-{height}", foot, height)
        if args.suite in ("faults", "regression"):
            for scenario in [
                "unreachable",
                "cancel",
                "replay",
                "missed_touchdown",
                "stale_imu",
                "side_collision",
                "support_loss",
                "slip",
            ]:
                trial(
                    f"fault-{scenario}",
                    0,
                    0.03 if scenario == "missed_touchdown" else 0,
                    scenario,
                )
        if args.suite == "robustness":
            for profile in ("mass_plus5", "mass_minus5", "friction_low"):
                for foot in range(4):
                    trial(f"{profile}-{foot}", foot, 0.05, profile=profile)
        if args.suite == "regression":
            cases.append(
                (
                    "legacy-motion",
                    [
                        str(ROOT / "src/custom_dog_control/scripts/test_simulation.sh"),
                        "--scenario",
                        "motion",
                        "--domain-id",
                        str(args.domain),
                        "--output-dir",
                        str(args.output / "legacy-motion"),
                    ],
                    300,
                )
            )
        if args.suite == "matrix":
            cases.append(
                (
                    "matrix",
                    [
                        sys.executable,
                        str(ROOT / "src/qr_validation/scripts/run_matrix.py"),
                        "--trials-per-foot",
                        "25",
                        "--jobs",
                        "2",
                        "--output",
                        str(args.output / "matrix"),
                    ],
                    14400,
                )
            )
        for name, command, timeout in cases:
            print(f"Running {name}", flush=True)
            result = run_case(name, command, args.output, timeout)
            report["results"].append(result)
            save()
            print(f"{name}: {result['status']}", flush=True)
            if name.startswith("unit-") and result["status"] != "passed":
                report["status"] = "failed"
                report["error"] = "unit tests failed; simulation not started"
                return 1
        report["status"] = (
            "passed"
            if all(r["status"] == "passed" for r in report["results"])
            else "failed"
        )
        return 0 if report["status"] == "passed" else 1
    except KeyboardInterrupt:
        report["status"] = "interrupted"
        report["error"] = "user interrupted test suite; no further cases scheduled"
        return 130
    except Exception as exc:
        report["status"] = "failed"
        report["error"] = str(exc)
        return 1
    finally:
        save()


if __name__ == "__main__":
    raise SystemExit(main())
