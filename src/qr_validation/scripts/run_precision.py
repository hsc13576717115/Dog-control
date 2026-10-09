#!/usr/bin/env python3
"""Cold-start, truth-independent QR precision acceptance. No real hardware launch."""

from acceptance import DEFAULT_CONFIG, load_thresholds
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from gazebo_msgs.msg import LinkStates, ContactsState
from qr_interfaces.msg import (
    RobotState,
    ExecutionStatus,
    FootContactArray,
    ContactMetrics,
)
from qr_interfaces.srv import PlanFootsteps
from qr_interfaces.action import ExecuteFootsteps
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu


class Probe(Node):
    def __init__(self, foot, scenario="normal"):
        super().__init__("qr_independent_evaluator")
        self.foot = foot
        self.state = None
        self.status = None
        self.truth = {}
        self.contacts = {}
        self.contact_times = {}
        self.collision_names = {}
        self.nonfoot_contacts = {}
        self.truth_received = 0.0
        self.trace = []
        self.metrics = None
        self.metrics_received = 0.0
        self.estimates = None
        self.scenario = scenario
        self.last_imu = None
        if scenario == "stale_imu":
            self.imu_pub = self.create_publisher(Imu, "/imu", qos_profile_sensor_data)
            self.create_subscription(
                Imu, "/qr/raw_imu", self.relay_imu, qos_profile_sensor_data
            )
        self.create_subscription(
            RobotState, "/qr/state", lambda m: setattr(self, "state", m), 10
        )
        self.create_subscription(ExecutionStatus, "/qr/execution", self.on_status, 10)
        self.create_subscription(
            LinkStates,
            "/evaluation/link_states",
            self.on_truth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            FootContactArray,
            "/qr/contact_estimates",
            lambda m: setattr(self, "estimates", m),
            10,
        )
        for leg in ["FR", "FL", "RR", "RL"]:
            self.create_subscription(
                ContactsState,
                f"/evaluation/contact_{leg}",
                lambda m, l=leg: self.on_contact(l, m),
                qos_profile_sensor_data,
            )
        for link in ["base"] + [
            leg + "_" + part
            for leg in ["FR", "FL", "RR", "RL"]
            for part in ["hip", "thigh", "calf"]
        ]:
            self.create_subscription(
                ContactsState,
                "/evaluation/nonfoot_" + link,
                lambda m, l=link: self.nonfoot_contacts.update({l: bool(m.states)}),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            ContactMetrics,
            "/evaluation/contact_metrics",
            self.on_metrics,
            qos_profile_sensor_data,
        )
        self.plan = self.create_client(PlanFootsteps, "/qr/plan_footsteps")
        self.action = ActionClient(self, ExecuteFootsteps, "/qr/execute_footsteps")

    def on_metrics(self, m):
        self.metrics = m
        self.metrics_received = time.monotonic()

    def on_truth(self, m):
        self.truth = {n: p for n, p in zip(m.name, m.pose)}
        self.truth_received = time.monotonic()

    def relay_imu(self, message):
        if (
            self.last_imu is None
            or self.status is None
            or self.status.phase not in ["SWING", "CONTROLLED_HOLD"]
        ):
            self.last_imu = message
        # Republish at the original rate with a frozen sample timestamp. A
        # recent receive time must not disguise stale physical information.
        self.imu_pub.publish(self.last_imu)

    def on_contact(self, leg, m):
        self.collision_names[leg] = [
            name for s in m.states for name in (s.collision1_name, s.collision2_name)
        ]
        self.contacts[leg] = bool(m.states)
        self.contact_times[leg] = time.monotonic()

    def on_status(self, m):
        self.status = m
        if self.state:
            self.trace.append(
                {
                    "time": m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec,
                    "phase": m.phase,
                    "ready": m.ready,
                    "error": m.error,
                    "feet": [[p.x, p.y, p.z] for p in self.state.feet],
                    "targets": [[p.x, p.y, p.z] for p in m.targets],
                    "estimated_contacts": list(self.state.contacts),
                    "forces": (
                        [
                            [
                                p.estimated_force.x,
                                p.estimated_force.y,
                                p.estimated_force.z,
                            ]
                            for p in self.estimates.feet
                        ]
                        if self.estimates
                        else []
                    ),
                    "truth_feet": {
                        l: [p.position.x, p.position.y, p.position.z]
                        for l, p in self.truth.items()
                        if l.endswith("_foot")
                    },
                    "truth_contacts": self.contacts.copy(),
                    "collision_names": self.collision_names.copy(),
                    "nonfoot_contacts": self.nonfoot_contacts.copy(),
                    "body_quaternion": [
                        self.state.pose.orientation.x,
                        self.state.pose.orientation.y,
                        self.state.pose.orientation.z,
                        self.state.pose.orientation.w,
                    ],
                    "truth_body_quaternion": next(
                        (
                            [
                                p.orientation.x,
                                p.orientation.y,
                                p.orientation.z,
                                p.orientation.w,
                            ]
                            for n, p in self.truth.items()
                            if n == "custom_dog::base"
                        ),
                        None,
                    ),
                    "contact_slip_integral_m": (
                        list(self.metrics.cumulative_slip_m) if self.metrics else None
                    ),
                    "contact_time_integral_s": (
                        list(self.metrics.cumulative_contact_s)
                        if self.metrics
                        else None
                    ),
                    "physics_sequence": (
                        self.metrics.physics_sequence if self.metrics else None
                    ),
                    "solve_ms": m.solve_time_ms,
                    "residual": m.equality_residual,
                }
            )

    def wait(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False


def stop(p):
    for sig, seconds in [(signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)]:
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            break
        try:
            p.wait(timeout=seconds)
            break
        except subprocess.TimeoutExpired:
            pass
    # ros2 launch usually reaps children; catch orphaned group members too.
    try:
        os.killpg(p.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--thresholds", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--model-config",
        type=Path,
        default=Path("src/custom_dog_control/config/precision_model.yaml"),
    )
    parser.add_argument("--height", type=float, choices=[0.0, 0.03, 0.05], default=0.0)
    parser.add_argument("--foot", type=int, choices=range(4), default=0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--domain", type=int, default=109)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--profile",
        choices=["nominal", "mass_plus5", "mass_minus5", "friction_low"],
        default="nominal",
    )
    parser.add_argument("--gui", action="store_true")
    parser.add_argument(
        "--scenario",
        choices=[
            "normal",
            "cancel",
            "unreachable",
            "replay",
            "missed_touchdown",
            "stale_imu",
            "side_collision",
            "support_loss",
            "slip",
        ],
        default="normal",
    )
    a = parser.parse_args()
    if a.scenario in ("side_collision", "support_loss", "slip") and (
        a.foot != 0 or a.height != 0
    ):
        parser.error("physical fault fixtures require FR on flat ground")
    thresholds = load_thresholds(a.thresholds)
    import yaml

    model_config = yaml.safe_load(a.model_config.read_text())["model"]
    foot_radius = float(model_config["foot_radius_m"])
    if not math.isfinite(foot_radius) or foot_radius <= 0:
        parser.error("invalid foot radius")
    if a.scenario == "missed_touchdown" and a.height == 0.0:
        parser.error("missed_touchdown requires --height 0.03 or 0.05")
    a.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(
        ROS_DOMAIN_ID=str(a.domain),
        ROS_LOCALHOST_ONLY="1",
        GAZEBO_MODEL_DATABASE_URI="",
    )
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    os.environ["GAZEBO_MASTER_URI"] = f"http://127.0.0.1:{port}"
    report = {
        "status": "failed",
        "height": a.height,
        "foot": a.foot,
        "scenario": a.scenario,
        "seed": a.seed,
        "physics_profile": a.profile,
        "scene_version": "qr-m1-v1",
        "reported_height": a.height,
        "actual_height": 0.0 if a.scenario == "missed_touchdown" else a.height,
        "using_ground_truth_for_control": False,
        "contact_input": "joint_effort_residual_and_kinematics",
        "hardware_tested": False,
        "thresholds": thresholds,
        "thresholds_sha256": hashlib.sha256(a.thresholds.read_bytes()).hexdigest(),
        "model_config_sha256": hashlib.sha256(a.model_config.read_bytes()).hexdigest(),
    }
    report["commit"] = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], text=True
    ).strip()
    report["worktree_diff_sha256"] = hashlib.sha256(
        subprocess.check_output(["git", "diff"])
    ).hexdigest()
    files = list(Path("src/custom_dog_control").rglob("*.cpp")) + list(
        Path("src/custom_dog_control").rglob("*.hpp")
    )
    files.extend(Path("src/custom_dog_control").rglob("*.def"))
    for folder in ["config", "urdf"]:
        files.extend(
            p
            for p in (Path("src/custom_dog_control") / folder).rglob("*")
            if p.is_file()
        )
    for directory in Path("src").glob("qr_*"):
        files.extend(
            p
            for p in directory.rglob("*")
            if p.is_file() and "__pycache__" not in str(p)
        )
    report["source_hashes"] = {
        str(p): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(set(files))
        if "third_party" not in str(p)
    }
    report["model_sha256"] = hashlib.sha256(
        Path(
            "external/model_ws/src/custom_dog_description/urdf/custom_dog.urdf"
        ).read_bytes()
    ).hexdigest()
    launch = None
    node = None
    try:
        log = (a.output / "launch.log").open("w")
        launch = subprocess.Popen(
            [
                "ros2",
                "launch",
                "qr_bringup",
                "precision_step.launch.py",
                f"height:={0. if a.scenario == 'missed_touchdown' else a.height}",
                f"reported_height:={a.height}",
                f"seed:={a.seed}",
                f"robustness_profile:={a.profile}",
                f"artifact_dir:={a.output.resolve() / 'inputs'}",
                f"model_config:={a.model_config.resolve()}",
                f"imu_fault_relay:={str(a.scenario == 'stale_imu').lower()}",
                f"gui:={str(a.gui).lower()}",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        rclpy.init()
        node = Probe(a.foot, a.scenario)
        if (
            not node.wait(
                lambda: node.status is not None
                and (node.status.ready or node.status.phase == "CONTROLLED_HOLD"),
                100,
            )
            or not node.status.ready
        ):
            raise RuntimeError(
                "standing_not_ready: "
                + (node.status.error if node.status else "no_status")
            )
        if node.state.using_ground_truth:
            raise RuntimeError("controller_using_ground_truth")
        if a.profile.startswith("mass_"):
            from physics_profiles import apply_mass_profile

            report["physics_mass_changes"] = apply_mass_profile(
                node, 1.05 if a.profile == "mass_plus5" else 0.95
            )
            node.wait(lambda: False, 0.5)
            if not node.status.ready:
                raise RuntimeError("perturbed_standing_not_ready")
        req = PlanFootsteps.Request()
        req.foot = a.foot
        req.surface_id = 1 if a.foot in (0, 2) else 2
        req.target = Point(
            x=0.0, y=-0.18 if req.surface_id == 1 else 0.18, z=a.height + foot_radius
        )
        if a.scenario == "unreachable":
            req.target.x = 2.0
        if not node.plan.wait_for_service(timeout_sec=3):
            raise RuntimeError("planner_unavailable")
        f = node.plan.call_async(req)
        if not node.wait(f.done, 10):
            raise RuntimeError("planning_timeout")
        plan = f.result()
        report["plan_accepted"] = plan.accepted
        report["plan_reason"] = plan.reason
        report["plan_error_code"] = plan.error_code
        if a.scenario == "unreachable":
            if plan.accepted or plan.reason != "outside_eroded_surface":
                raise RuntimeError("invalid_target_not_rejected")
            report["status"] = "passed"
            return 0
        if not plan.accepted:
            raise RuntimeError("plan_rejected: " + plan.reason)
        if not node.wait(lambda: node.metrics is not None, 3):
            raise RuntimeError("missing_contact_metrics")
        slip_start = list(node.metrics.cumulative_slip_m)
        contact_start = list(node.metrics.cumulative_contact_s)
        goal = ExecuteFootsteps.Goal()
        goal.plan = plan.plan
        if not node.action.wait_for_server(timeout_sec=3):
            raise RuntimeError("executor_unavailable")
        f = node.action.send_goal_async(goal)
        if not node.wait(f.done, 3) or not f.result().accepted:
            raise RuntimeError("action_rejected")
        handle = f.result()
        result = handle.get_result_async()
        if a.scenario in ("side_collision", "support_loss", "slip"):
            from physical_faults import inject

            report["physical_injection"] = inject(node, a.scenario, a.foot)
        if a.scenario == "cancel":
            if not node.wait(lambda: node.status.phase == "SWING", 8):
                raise RuntimeError("swing_not_observed")
            cancellation = handle.cancel_goal_async()
            if (
                not node.wait(cancellation.done, 3)
                or not cancellation.result().goals_canceling
            ):
                raise RuntimeError("cancel_not_accepted")
        if not node.wait(result.done, 20):
            raise RuntimeError("execution_timeout")
        response = result.result().result
        report["execution_error_code"] = response.error_code
        report["execution_success"] = response.success
        report["execution_reason"] = response.reason
        if a.scenario in ("side_collision", "support_loss", "slip"):
            from physical_faults import witness

            node.wait(lambda: False, 0.6)
            observed = witness(
                node.trace, a.scenario, report["physical_injection"]["sim_stamp"]
            )
            report["physical_fault_witnessed"] = observed
            if not observed:
                raise RuntimeError("injected_fault_not_physically_observed")
            if (
                response.success
                or node.status.ready
                or node.status.phase != "CONTROLLED_HOLD"
            ):
                raise RuntimeError("physical_fault_not_aborted: " + response.reason)
            if response.reason not in [
                "early_contact",
                "support_unconfirmed",
                "state_or_attitude",
                "touchdown_lost",
                "runtime_ik",
                "wbc_invalid_or_timeout",
                "total_torque_limit",
                "swing_tracking_error",
            ]:
                raise RuntimeError(
                    "unexpected_physical_fault_reason: " + response.reason
                )
            if a.scenario == "side_collision" and response.reason not in [
                "early_contact",
                "swing_tracking_error",
            ]:
                raise RuntimeError("side_collision_detection_late: " + response.reason)
            report["verified_scope"] = (
                "physical fault witnessed and execution aborted; stable recovery not certified"
            )
            report["status"] = "passed"
            return 0
        if a.scenario == "stale_imu":
            if response.success or response.reason != "invalid_state":
                raise RuntimeError("stale_imu_not_handled: " + response.reason)
            report["status"] = "passed"
            return 0
        if a.scenario == "missed_touchdown":
            if response.success or response.reason not in [
                "probe_travel_limit",
                "touchdown_timeout",
            ]:
                raise RuntimeError("missing_contact_not_handled: " + response.reason)
            report["status"] = "passed"
            return 0
        if a.scenario == "cancel":
            if response.success or response.reason != "canceled":
                raise RuntimeError("cancel_reported_incorrectly")
            node.wait(lambda: False, 0.5)
            if node.status.ready or node.status.phase != "CONTROLLED_HOLD":
                raise RuntimeError("cancel_did_not_hold")
            report["status"] = "passed"
            return 0
        if not response.success:
            raise RuntimeError("execution_failed: " + response.reason)
        node.wait(lambda: False, 0.5)
        name = "custom_dog::" + ["FR", "FL", "RR", "RL"][a.foot] + "_foot"
        if name not in node.truth or time.monotonic() - node.truth_received > 0.2:
            raise RuntimeError("missing_or_stale_independent_foot_truth")
        p = node.truth[name].position
        error = math.dist([p.x, p.y, p.z], [req.target.x, req.target.y, req.target.z])
        report["error_m"] = error
        active = [
            s
            for s in node.trace
            if s["phase"] in ["SHIFT", "SWING", "CONTACT_CONFIRM", "RESTORE", "DONE"]
        ]
        if not active or len(node.nonfoot_contacts) != 13:
            raise RuntimeError("missing_independent_collision_witnesses")
        if any(any(s["nonfoot_contacts"].values()) for s in active):
            raise RuntimeError("body_or_leg_collision_observed")
        slip = 0.0
        tilt = 0.0
        for leg in ["FR", "FL", "RR", "RL"]:
            if leg == ["FR", "FL", "RR", "RL"][a.foot]:
                continue
            points = [
                s["truth_feet"].get("custom_dog::" + leg + "_foot") for s in active
            ]
            points = [p for p in points if p is not None]
            if not points:
                raise RuntimeError("missing_support_truth")
            slip = max(slip, max(math.dist(points[0], p) for p in points))
        for s in active:
            if s["truth_body_quaternion"] is None:
                raise RuntimeError("missing_body_truth")
            x, y, z, w = s["truth_body_quaternion"]
            roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
            tilt = max(tilt, abs(roll) * 180 / math.pi, abs(pitch) * 180 / math.pi)
        if time.monotonic() - node.metrics_received > 0.2:
            raise RuntimeError("stale_contact_metrics")
        support_indices = [i for i in range(4) if i != a.foot]
        contact_slip = [
            node.metrics.cumulative_slip_m[i] - slip_start[i] for i in range(4)
        ]
        contact_duration = [
            node.metrics.cumulative_contact_s[i] - contact_start[i] for i in range(4)
        ]
        if any(contact_duration[i] <= 0 for i in support_indices):
            raise RuntimeError("contact_metrics_missing_support_manifold")
        report.update(
            support_contact_slip_m=max(contact_slip[i] for i in support_indices),
            contact_slip_per_foot_m=contact_slip,
            contact_duration_per_foot_s=contact_duration,
            support_foot_center_displacement_m=slip,
            support_slip_m=slip,
            max_tilt_deg=tilt,
            truth_contact_available=bool(node.contacts),
        )
        if any(
            not node.contacts.get(leg)
            or time.monotonic() - node.contact_times.get(leg, 0.0) > 0.2
            for leg in ["FR", "FL", "RR", "RL"]
        ):
            raise RuntimeError("independent_final_support_unconfirmed")
        if (
            error > thresholds["max_error_m"]
            or slip > thresholds["max_support_slip_m"]
            or report["support_contact_slip_m"] > thresholds["max_contact_slip_m"]
            or tilt > thresholds["max_tilt_deg"]
        ):
            raise RuntimeError("acceptance_threshold_exceeded")
        if a.scenario == "replay":
            repeated = node.action.send_goal_async(goal)
            if not node.wait(repeated.done, 3) or repeated.result().accepted:
                raise RuntimeError("replayed_plan_not_rejected")
            report["replay_rejected"] = True
        report["status"] = "passed"
    except Exception as e:
        report["error"] = str(e)
    finally:
        if node:
            (a.output / "trace.json").write_text(json.dumps(node.trace))
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if launch:
            stop(launch)
        report["generated_input_hashes"] = {
            p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted((a.output / "inputs").glob("*"))
            if p.is_file()
        }
        (a.output / "result.json").write_text(json.dumps(report, indent=2))
        print(
            json.dumps(
                {k: v for k, v in report.items() if k != "source_hashes"}, indent=2
            ),
            flush=True,
        )
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
