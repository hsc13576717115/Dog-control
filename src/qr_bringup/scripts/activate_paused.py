#!/usr/bin/env python3
"""Activate Gazebo controllers using bounded paused steps before free-running."""

import subprocess
import time
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
from std_srvs.srv import Empty


def main():
    rclpy.init()
    node = Node("qr_paused_activation")
    try:
        switch = node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        unpause = node.create_client(Empty, "/unpause_physics")
        if not switch.wait_for_service(
            timeout_sec=20.0
        ) or not unpause.wait_for_service(timeout_sec=20.0):
            raise RuntimeError("Gazebo/controller services unavailable")
        request = SwitchController.Request()
        request.activate_controllers = [
            "joint_state_broadcaster",
            "nmpc_wbc_controller",
        ]
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 15
        future = switch.call_async(request)
        # The service response requires controller update(), which needs physics
        # steps. A wall timer followed by unpause races CLI/DDS discovery and can
        # let the unpowered robot fall for hundreds of simulation milliseconds.
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.05)
            if future.done():
                break
            subprocess.run(
                ["gz", "world", "-w", "qr_m1", "--step"],
                check=True,
                timeout=3,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done() or not future.result().ok:
            raise RuntimeError("Activation failed; world remains paused")
        resumed = unpause.call_async(Empty.Request())
        rclpy.spin_until_future_complete(node, resumed, timeout_sec=5.0)
        if not resumed.done() or resumed.exception():
            raise RuntimeError("Unpause failed")
        node.get_logger().info("Controllers active before free-running physics")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
