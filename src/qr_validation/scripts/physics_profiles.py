"""Modify simulator inertias after model validation; controller stays canonical."""

import math
from gazebo_msgs.srv import GetLinkProperties, SetLinkProperties
from std_srvs.srv import Empty


def apply_mass_profile(node, scale):
    def call(service, endpoint, request):
        client = node.create_client(service, endpoint)
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError("profile_service_unavailable: " + endpoint)
        future = client.call_async(request)
        if not node.wait(future.done, 3):
            raise RuntimeError("profile_service_timeout: " + endpoint)
        result = future.result()
        if hasattr(result, "success") and not result.success:
            raise RuntimeError("profile_service_failed: " + endpoint)
        return result

    records = []
    call(Empty, "/pause_physics", Empty.Request())
    try:
        names = sorted(n for n in node.truth if n.startswith("custom_dog::"))
        if len(names) < 17:
            raise RuntimeError("missing_robot_links_for_mass_profile")
        for name in names:
            req = GetLinkProperties.Request(link_name=name)
            before = call(GetLinkProperties, "/evaluation/get_link_properties", req)
            change = SetLinkProperties.Request(
                link_name=name, com=before.com, gravity_mode=before.gravity_mode
            )
            for field in ("mass", "ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                setattr(change, field, getattr(before, field) * scale)
            call(SetLinkProperties, "/evaluation/set_link_properties", change)
            after = call(GetLinkProperties, "/evaluation/get_link_properties", req)
            if not math.isclose(after.mass, before.mass * scale, rel_tol=1e-8):
                raise RuntimeError("physics_mass_readback_mismatch")
            records.append(dict(link=name, before_kg=before.mass, after_kg=after.mass))
    finally:
        call(Empty, "/unpause_physics", Empty.Request())
    return records
