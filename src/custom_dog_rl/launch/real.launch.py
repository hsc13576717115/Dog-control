"""Launch the RL controller with the existing serial hardware plugin.

No NMPC controller is loaded. Model identity is checked before creating any
hardware node. Calibration, standing and RL entry remain explicit ROS commands.
"""
import hashlib
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _cleanup(_context, path):
    if os.path.exists(path):
        os.remove(path)
    return []


def _launch(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    model = os.path.abspath(os.path.expanduser(value('model_path')))
    expected = value('expected_model_sha256')
    with open(model, 'rb') as stream:
        actual = hashlib.file_digest(stream, 'sha256').hexdigest() if hasattr(hashlib, 'file_digest') else hashlib.sha256(stream.read()).hexdigest()
    if actual != expected:
        raise RuntimeError('Model SHA256 mismatch; no hardware node was started')
    package = get_package_share_directory('custom_dog_rl')
    control = get_package_share_directory('custom_dog_control')
    description = value('description_urdf')
    if not description:
        description = os.path.join(get_package_share_directory('custom_dog_description'), 'urdf', 'custom_dog.urdf')
    description = os.path.abspath(os.path.expanduser(description))
    mappings = {name: value(name) for name in (
        'physical_estop_verified', 'fr_port', 'fl_port', 'rr_port', 'rl_port',
        'motor_directions', 'calibration_hip_deg', 'calibration_thigh_deg', 'calibration_calf_deg')}
    mappings.update(hardware_mode='real', description_urdf=description)
    robot = xacro.process_file(os.path.join(control, 'urdf', 'custom_dog.ros2_control.xacro'), mappings=mappings).toxml()
    with open(value('config_file'), encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    parameters = config['rl_controller']['ros__parameters']
    parameters.update(hardware_mode='real', model_path=model,
                      expected_model_sha256=expected,
                      enable_actuation=value('enable_actuation').lower() == 'true')
    handle, temporary = tempfile.mkstemp(prefix='customdog-rl-real-', suffix='.yaml')
    with os.fdopen(handle, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(config, stream)
    actions = [
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[temporary, {'robot_description': robot}], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot}], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['rl_controller', '--controller-manager-timeout', '30'], output='screen'),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=_cleanup, kwargs={"path": temporary})])),
    ]
    if value('start_imu').lower() == 'true':
        actions.append(Node(package='fdilink_ahrs', executable='ahrs_driver_node', name='ahrs_driver',
                            parameters=[{'if_debug_': False, 'serial_port_': value('imu_port'),
                                         'serial_baud_': int(value('imu_baud')),
                                         'imu_topic': parameters['imu_topic'],
                                         'imu_frame_id_': parameters['imu_frame'], 'device_type_': 1}],
                            output='screen'))
    return actions


def generate_launch_description():
    package = get_package_share_directory('custom_dog_rl')
    defaults = {
        'model_path': '',
        'expected_model_sha256': '10bcd8da253aba3e21dad7a417a73258a5753b30bc04ef63433ec706852fa54c',
        'config_file': os.path.join(package, 'config', 'rl_controller.yaml'),
        'description_urdf': '', 'enable_actuation': 'false', 'physical_estop_verified': 'false',
        'fr_port': '/dev/ttyS3', 'fl_port': '/dev/ttyS4', 'rr_port': '/dev/ttyS7', 'rl_port': '/dev/ttyS8',
        'motor_directions': '1 1 1 1 1 1 1 1 1 1 1 1',
        'calibration_hip_deg': '0.0', 'calibration_thigh_deg': '71.8', 'calibration_calf_deg': '-161.8',
        'start_imu': 'true', 'imu_port': '/dev/ttyUSB0', 'imu_baud': '921600',
    }
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value)
                              for name, value in defaults.items()] + [OpaqueFunction(function=_launch)])
