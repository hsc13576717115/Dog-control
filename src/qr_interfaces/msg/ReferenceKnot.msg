# Relative body/ZYX offsets in PlanFootsteps request; absolute in approved plan.
# ZYX is yaw,pitch,roll (rad); not a quaternion. Foot centre is always in odom.
# Stop-to-stop quintic between knots. No contact declarations in this message.
float64 time_from_start
geometry_msgs/Point body
geometry_msgs/Vector3 euler_zyx
geometry_msgs/Point foot
