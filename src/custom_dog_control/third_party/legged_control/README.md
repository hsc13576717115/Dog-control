# legged_control algorithm sources

The `legged_interface` directory is vendored from
`qiayuanl/legged_control` revision
`a7f381c0367e98e31c01336e678eef47e304d40d`.

It is compiled directly into `custom_dog_control_legged_interface`. The ROS 1
controller, messages, hardware abstraction, launch files and visualization are
not vendored. ROS 2 integration remains in `custom_dog_control`; the optimal
control problem, switched reference manager, swing trajectory planner,
constraints, costs and initializer remain the upstream algorithm sources.

The separately vendored `third_party/legged_wbc` sources come from the same
upstream revision. Solver-success and residual reporting are local safety
instrumentation; the WBC decision variables, constraints and weighted tasks
remain those of upstream `WeightedWbc`.
