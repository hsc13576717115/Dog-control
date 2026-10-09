"""Evaluation-only Gazebo disturbances. Never imported by a controller node."""

from gazebo_msgs.srv import ApplyLinkWrench, SpawnEntity


def inject(node, scenario, foot):
    """Return injection metadata; success of a service is NOT a physical witness."""
    if not node.wait(lambda: node.status.phase == "SWING", 8):
        raise RuntimeError("swing_not_observed")
    if scenario == "side_collision":
        start = node.status.header.stamp.sec + node.status.header.stamp.nanosec * 1e-9
        if not node.wait(
            lambda: node.status.header.stamp.sec
            + node.status.header.stamp.nanosec * 1e-9
            >= start + 0.4,
            3,
        ):
            raise RuntimeError("swing_time_not_advancing")
        client = node.create_client(SpawnEntity, "/spawn_entity")
        request = SpawnEntity.Request()
        request.name = "evaluation_side_wall"
        # FR-only fixture: the foot swings from positive x toward x=0.
        request.xml = """<sdf version="1.6"><model name="evaluation_side_wall">
          <static>true</static><pose>0.075 -0.18 0.075 0 0 0</pose><link name="wall">
          <collision name="wall"><geometry><box><size>0.015 0.08 0.06</size></box></geometry></collision>
          <visual name="wall"><geometry><box><size>0.015 0.08 0.06</size></box></geometry></visual>
          </link></model></sdf>"""
        request.initial_pose.orientation.w = 1.0
        metadata = {"type": "spawn_static_side_wall", "target_foot": "FR"}
    else:
        client = node.create_client(ApplyLinkWrench, "/apply_link_wrench")
        request = ApplyLinkWrench.Request()
        request.link_name = "custom_dog::FL_foot"
        request.reference_frame = "world"
        p = node.truth[request.link_name].position
        request.reference_point = p
        if scenario == "support_loss":
            request.wrench.force.z = 120.0
        elif scenario == "slip":
            request.wrench.force.y = 100.0
        else:
            raise ValueError(scenario)
        request.start_time = node.status.header.stamp
        request.duration.sec = 0
        request.duration.nanosec = 350000000
        metadata = {
            "type": "finite_link_wrench",
            "target_foot": "FL",
            "duration_s": 0.35,
            "force_world_n": [
                request.wrench.force.x,
                request.wrench.force.y,
                request.wrench.force.z,
            ],
        }
    if not client.wait_for_service(timeout_sec=3):
        raise RuntimeError("physical_injector_service_unavailable")
    future = client.call_async(request)
    if not node.wait(future.done, 3) or not future.result().success:
        raise RuntimeError("physical_injection_rejected")
    metadata["sim_stamp"] = (
        node.status.header.stamp.sec + node.status.header.stamp.nanosec * 1e-9
    )
    return metadata


def witness(trace, scenario, stamp):
    samples = [s for s in trace if s["time"] >= stamp]
    if scenario == "side_collision":
        return any(
            "evaluation_side_wall" in name
            for s in samples
            for name in s.get("collision_names", {}).get("FR", [])
        )
    if scenario == "support_loss":
        return any(s["truth_contacts"].get("FL") is False for s in samples)
    values = [
        s["contact_slip_integral_m"][1]
        for s in samples
        if s.get("contact_slip_integral_m")
    ]
    return bool(values) and bool(max(values) - min(values) > 0.01)
