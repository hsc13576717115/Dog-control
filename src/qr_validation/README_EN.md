# qr_validation

[中文](README.md) | **English**

run_precision.py cold-starts a single step and independently observes poses, sole contacts and non-foot collisions. run_matrix.py retains all failures in its denominator and reports metric sample counts. Fault scenarios cover invalid targets, cancellation, replay, missing contact and stale IMU. A passed fault test means correct rejection/abort, not successful motion. Other dynamic fault cases remain outstanding.

See [QR development and validation](../../docs/qr/README_EN.md).

Use `python3 tools/validate_qr.py --suite regression --output artifacts/qr/regression-001`
from the sourced repository root. Supported suites: unit, smoke, faults,
regression and matrix. Python tests are registered with CTest. Independent
acceptance thresholds live in `config/acceptance.yaml`; their values and hash
are recorded in each step/matrix report.
