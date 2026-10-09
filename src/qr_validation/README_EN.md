# qr_validation

[中文](README.md) | **English**

run_precision.py cold-starts a single step and independently observes poses, sole contacts and non-foot collisions. run_matrix.py retains all failures in its denominator and reports metric sample counts. Fault scenarios cover invalid targets, cancellation, replay, missing contact and stale IMU. A passed fault test means correct rejection/abort, not successful motion. Other dynamic fault cases remain outstanding.

See [QR development and validation](../../docs/qr/README_EN.md).
