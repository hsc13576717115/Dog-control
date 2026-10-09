"""QR M1: full collision, IMU/joints only, no physical motor plugin."""

import atexit
import os
from pathlib import Path
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    height = float(LaunchConfiguration("height").perform(context))
    observed_height = LaunchConfiguration("reported_height").perform(context)
    observed_height = height if observed_height == "same" else float(observed_height)
    if observed_height not in (0.0, 0.03, 0.05):
        raise ValueError("reported_height must be same/0/0.03/0.05")
    control = Path(get_package_share_directory("custom_dog_control"))
    description = Path(get_package_share_directory("custom_dog_description"))
    course = Path(get_package_share_directory("qr_course"))
    sys.path.insert(0, str(course / "scripts"))
    from course import generate

    runtime = Path(tempfile.mkdtemp(prefix="qr_m1_"))
    atexit.register(lambda: shutil.rmtree(runtime, ignore_errors=True))
    generate(height, runtime / "fixture.world")
    cfg = yaml.safe_load((control / "config/controllers.yaml").read_text())
    cfg["nmpc_wbc_controller"]["ros__parameters"].update(
        precision_enabled=True,
        use_sim_ground_truth=False,
        simulation_passive_hold=False,
    )
    (runtime / "controllers.yaml").write_text(yaml.safe_dump(cfg))
    robot = xacro.process_file(
        str(control / "urdf/custom_dog.ros2_control.xacro"),
        mappings={
            "hardware_mode": "gazebo",
            "description_urdf": str(description / "urdf/custom_dog.urdf"),
            "controllers_file": str(runtime / "controllers.yaml"),
            "gazebo_control_plugin": "/opt/ros/humble/lib/libgazebo_ros2_control.so",
        },
    ).toxml()
    root = ET.fromstring(robot)
    if LaunchConfiguration("imu_fault_relay").perform(context) == "true":
        # Only the evaluator relays this sensor for stale-data tests. Normal
        # runs retain the direct /imu sensor path.
        root.find(".//plugin[@name='base_imu_plugin']/ros/remapping").text = (
            "~/out:=/qr/raw_imu"
        )
    # M1 starts from a standing joint configuration. Prone-to-stand is a separate test.
    nominal = cfg["nmpc_wbc_controller"]["ros__parameters"]["nominal_joint_positions"]
    for j, q in zip(root.findall("./ros2_control/joint"), nominal):
        j.find(
            './state_interface[@name="position"]/param[@name="initial_value"]'
        ).text = str(q)
    for leg in ["FR", "FL", "RR", "RL"]:
        foot = f"{leg}_foot"
        link = root.find(f'./link[@name="{foot}"]')
        for c in link.findall("collision"):
            c.set("name", f"{foot}_collision")
        joint = next(
            j for j in root.findall("joint") if j.find("child").get("link") == foot
        )
        fixed = ET.SubElement(root, "gazebo", reference=joint.get("name"))
        ET.SubElement(fixed, "preserveFixedJoint").text = "true"
        ext = ET.SubElement(root, "gazebo", reference=foot)
        sensor = ET.SubElement(ext, "sensor", name=f"evaluation_{leg}", type="contact")
        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = "250"
        contact = ET.SubElement(sensor, "contact")
        ET.SubElement(contact, "collision").text = f"{foot}_collision_collision"
        plugin = ET.SubElement(
            sensor,
            "plugin",
            name=f"evaluation_contact_{leg}",
            filename="libgazebo_ros_bumper.so",
        )
        ros = ET.SubElement(plugin, "ros")
        ET.SubElement(ros, "namespace").text = "/evaluation"
        ET.SubElement(ros, "remapping").text = f"bumper_states:=contact_{leg}"
        ET.SubElement(plugin, "frame_name").text = "world"
    # Independent body/leg collision witnesses. They do not feed the control
    # process, and a successful action cannot hide a shin scraping a fixture.
    for link in root.findall("link"):
        name = link.get("name")
        if name.endswith("_foot") or link.find("collision") is None:
            continue
        sensor_extension = ET.SubElement(root, "gazebo", reference=name)
        sensor = ET.SubElement(
            sensor_extension,
            "sensor",
            name=f"evaluation_nonfoot_{name}",
            type="contact",
        )
        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = "100"
        contact = ET.SubElement(sensor, "contact")
        for index, collision in enumerate(link.findall("collision")):
            collision_name = f"{name}_witness_{index}"
            collision.set("name", collision_name)
            ET.SubElement(contact, "collision").text = collision_name + "_collision"
        plugin = ET.SubElement(
            sensor, "plugin", name=f"witness_{name}", filename="libgazebo_ros_bumper.so"
        )
        ros = ET.SubElement(plugin, "ros")
        ET.SubElement(ros, "namespace").text = "/evaluation"
        ET.SubElement(ros, "remapping").text = f"bumper_states:=nonfoot_{name}"
        ET.SubElement(plugin, "frame_name").text = "world"
    robot = ET.tostring(root, encoding="unicode")
    (runtime / "robot.urdf").write_text(robot)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("gazebo_ros"))
                / "launch/gazebo.launch.py"
            )
        ),
        launch_arguments={
            "world": str(runtime / "fixture.world"),
            "gui": LaunchConfiguration("gui"),
            "pause": "true",
            "seed": LaunchConfiguration("seed"),
        }.items(),
    )
    state = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot, "use_sim_time": True}],
        output="screen",
    )
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "custom_dog",
            "-file",
            str(runtime / "robot.urdf"),
            "-z",
            ".29",
        ],
        output="screen",
    )
    controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "nmpc_wbc_controller",
            "--inactive",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )
    surfaces = ExecuteProcess(
        cmd=[
            "python3",
            str(course / "scripts/known_surfaces.py"),
            "--height",
            str(observed_height),
            "--ros-args",
            "-p",
            "use_sim_time:=true",
        ],
        output="screen",
    )
    activate = ExecuteProcess(
        cmd=[
            "python3",
            str(
                Path(get_package_share_directory("qr_bringup"))
                / "scripts/activate_paused.py"
            ),
        ],
        output="screen",
    )

    def after_configure(event, _):
        return [activate] if event.returncode == 0 else []

    def after_spawn(event, _):
        if event.returncode != 0:
            return []
        return [controllers]

    return [
        gazebo,
        state,
        RegisterEventHandler(
            OnProcessExit(target_action=controllers, on_exit=after_configure)
        ),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)),
        spawn,
        surfaces,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("height", default_value="0.0"),
            DeclareLaunchArgument("seed", default_value="0"),
            DeclareLaunchArgument(
                "reported_height",
                default_value="same",
                description="Validation-only known-map fault injection; same for normal runs",
            ),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("imu_fault_relay", default_value="false"),
            SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
            OpaqueFunction(function=setup),
        ]
    )
