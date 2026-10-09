#!/usr/bin/env python3
"""Cold-start step matrix; retain failed and rejected trials in the denominator."""

from acceptance import DEFAULT_CONFIG, load_thresholds
import hashlib
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import math
from pathlib import Path
from queue import Queue
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--thresholds", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trials-per-foot", type=int, default=25)
    parser.add_argument("--jobs", type=int, choices=[1, 2], default=1)
    args = parser.parse_args()
    thresholds = load_thresholds(args.thresholds)
    if args.trials_per_foot < 1:
        parser.error("trials-per-foot must be positive")
    args.output.mkdir(parents=True, exist_ok=False)
    domains = Queue()
    for domain in range(150, 150 + args.jobs):
        domains.put(domain)

    def run_trial(foot, index):
        domain = domains.get()
        directory = args.output / f"foot{foot}-{index:02d}"
        height = [0.0, 0.03, 0.05][index % 3]
        try:
            with (args.output / f"foot{foot}-{index:02d}.log").open("w") as log:
                run = subprocess.run(
                    [
                        sys.executable,
                        str(Path(__file__).with_name("run_precision.py")),
                        "--thresholds",
                        str(args.thresholds.resolve()),
                        "--foot",
                        str(foot),
                        "--height",
                        str(height),
                        "--seed",
                        str(foot * args.trials_per_foot + index),
                        "--domain",
                        str(domain),
                        "--output",
                        str(directory),
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
            result_file = directory / "result.json"
            result = (
                json.loads(result_file.read_text())
                if result_file.exists()
                else {
                    "status": "failed",
                    "error": "runner_failed",
                    "returncode": run.returncode,
                }
            )
            return dict(result, trial=f"foot{foot}-{index:02d}")
        finally:
            domains.put(domain)

    results = []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        pending = [
            pool.submit(run_trial, foot, index)
            for foot in range(4)
            for index in range(args.trials_per_foot)
        ]
        for future in as_completed(pending):
            result = future.result()
            results.append(result)
            (args.output / "partial.json").write_text(json.dumps(results, indent=2))
            print(
                f"{len(results)}/{4 * args.trials_per_foot}: {result['trial']} {result['status']} {result.get('error', '')}",
                flush=True,
            )

    errors = sorted(r["error_m"] for r in results if "error_m" in r)
    p95 = errors[max(0, math.ceil(0.95 * len(errors)) - 1)] if errors else None
    passed = sum(r["status"] == "passed" for r in results)
    complete_metrics = all(
        "error_m" in r for r in results if r.get("execution_success")
    )
    accepted = (
        passed / len(results) >= thresholds["minimum_success_rate"]
        and complete_metrics
        and len(errors) >= math.ceil(thresholds["minimum_success_rate"] * len(results))
        and p95 <= thresholds["p95_error_m"]
        and max(errors) <= thresholds["max_error_m"]
    )
    summary = {
        "thresholds": thresholds,
        "thresholds_sha256": hashlib.sha256(args.thresholds.read_bytes()).hexdigest(),
        "trials": len(results),
        "passed": passed,
        "jobs": args.jobs,
        "p95_error_m": p95,
        "max_error_m": max(errors) if errors else None,
        "all_error_metrics_available": complete_metrics,
        "error_metrics_count": len(errors),
        "error_metrics_scope": "completed executions; all startup/rejected/aborted trials remain in success-rate denominator",
        "step_matrix_accepted": accepted
        and args.trials_per_foot >= thresholds["minimum_trials_per_foot"],
        "scope": "Known M1 fixtures, not terrain randomization or full fault acceptance",
        "hardware_tested": False,
        "results": sorted(results, key=lambda r: r["trial"]),
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2))
    return 0 if summary["step_matrix_accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
