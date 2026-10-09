# QR simulation evaluation plugin

[中文](README.md)

`qr_contact_metrics` integrates mean tangential relative material-point speed over each
foot's Gazebo contact manifold at every physics step. Cumulative values are published at
100 Hz on `/evaluation/contact_metrics`, in FR/FL/RR/RL order. Evaluation subtracts the
pre-action baseline. The controller must never subscribe to this ground-truth channel.

Unlike foot-centre displacement, ideal rolling contributes zero sliding distance. Both
bodies' linear and angular velocities are included. Discrete contact integration and rigid
spherical soles still introduce numerical error; this is not a deformable hardware-sole
measurement. The plugin expects model `custom_dog` and its canonical foot link names.

Loaded by `qr_bringup/precision_step.launch.py`; exercised by `tools/validate_qr.py`.
No motor interfaces or hardware commands are present.
