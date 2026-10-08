#!/usr/bin/env python3
"""Compare C++ ONNX inference with checkpoint golden actions on saved histories.

This script performs offline inference only. It never connects to robot hardware.
The C++ probe's latency is measured on the current host, not inferred for Jetson.
Requires numpy and a compiled policy_probe; --torchscript additionally uses torch.
"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", required=True, type=Path)
    parser.add_argument("--deployment-dir", required=True, type=Path)
    parser.add_argument("--work-dir", required=True, type=Path)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--torchscript", action="store_true", help="Additionally verify the full TorchScript export")
    args = parser.parse_args()
    manifest = json.loads((args.deployment_dir / "manifest.json").read_text())
    required_files = ["sample_observations.npz", "golden_actions.npz", "policy_combined.onnx"]
    if args.torchscript:
        required_files.append("policy.pt")
    for name in required_files:
        actual_hash = hashlib.sha256((args.deployment_dir / name).read_bytes()).hexdigest()
        if actual_hash != manifest["files"][name]:
            raise ValueError(f"Deployment checksum mismatch: {name}")
    args.work_dir.mkdir(parents=True, exist_ok=False)
    with np.load(args.deployment_dir / "sample_observations.npz", allow_pickle=False) as samples:
        observations = np.asarray(samples["obs_history"], dtype="<f4")
    if observations.ndim != 2 or observations.shape[1] != 270 or not len(observations):
        raise ValueError(f"Expected nonempty [N,270] observations; got {observations.shape}")
    if not np.isfinite(observations).all():
        raise ValueError("Nonfinite observation sample")
    input_path = args.work_dir / "observations.f32"
    output_path = args.work_dir / "actions_cpp.f32"
    observations.tofile(input_path)
    result = subprocess.run(
        [str(args.probe.resolve()), "--model", str((args.deployment_dir / "policy_combined.onnx").resolve()),
         "--input", str(input_path.resolve()), "--output", str(output_path.resolve()),
         "--threads", str(args.threads), "--repeat", str(args.repeat)],
        check=False, capture_output=True, text=True,
    )
    if result.returncode:
        raise SystemExit(f"C++ probe failed ({result.returncode}):\n{result.stderr}")
    benchmark = json.loads(result.stdout)
    actual = np.fromfile(output_path, dtype="<f4").reshape(-1, 12)
    with np.load(args.deployment_dir / "golden_actions.npz", allow_pickle=False) as golden:
        expected = np.asarray(golden["actions"], dtype=np.float32)
    if args.torchscript:
        import torch
        torch.set_num_threads(1)
        policy = torch.jit.load(str(args.deployment_dir / "policy.pt"), map_location="cpu").eval()
        with torch.inference_mode():
            scripted = policy(torch.from_numpy(observations)).cpu().numpy()
        np.testing.assert_allclose(scripted, expected, atol=1e-4, rtol=1e-5)
    if actual.shape != expected.shape:
        raise ValueError(f"Output shapes differ: C++ {actual.shape}, checkpoint reference {expected.shape}")
    error = np.abs(actual - expected)
    passed = bool(np.allclose(actual, expected, atol=1e-4, rtol=1e-5))
    report = {
        "passed": passed,
        "reference": "Original checkpoint deterministic golden actions, CPU FP32",
        "test_input": "Saved actual Isaac simulation observation histories",
        "sample_count": len(observations),
        "max_abs_action_error": float(error.max()),
        "mean_abs_action_error": float(error.mean()),
        "atol": 1e-4,
        "rtol": 1e-5,
        "onnx_sha256": hashlib.sha256((args.deployment_dir / "policy_combined.onnx").read_bytes()).hexdigest(),
        "benchmark": benchmark,
        "hardware_tested": False,
    }
    (args.work_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if not passed:
        raise SystemExit("C++ ONNX replay differs from checkpoint reference beyond tolerance")


if __name__ == "__main__":
    main()
