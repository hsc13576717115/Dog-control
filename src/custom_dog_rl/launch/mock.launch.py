"""ROS plumbing smoke test. GenericSystem is not a dynamics simulator.

The default namespace keeps synthetic feedback separate from a real robot.
No serial driver, NMPC library, URDF mesh, or GPU is required.
"""
import hashlib
import os
import tempfile
import xml.etree.ElementTree as ET

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
    with open(model, 'rb') as stream:
        actual = hashlib.sha256(stream.read()).hexdigest()
    if actual != value('expected_model_sha256'):
        raise RuntimeError('Model SHA256 mismatch')
    namespace = value('namespace').strip('/')
    if not namespace:
        raise RuntimeError('Mock launch requires a nonempty namespace to isolate fake sensors')
    package = get_package_share_directory('custom_dog_rl')
    with open(value('config_file'), encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    parameters = config['rl_controller']['ros__parameters']
    parameters.update(hardware_mode='mock', model_path=model,
                      expected_model_sha256=value('expected_model_sha256'),
                      enable_actuation=value('enable_actuation').lower() == 'true',
                      imu_topic='/' + namespace + '/imu')
    robot = ET.Element('robot', name='custom_dog_mock')
    ET.SubElement(robot, 'link', name='base')
    system = ET.SubElement(robot, 'ros2_control', name='MockSystem', type='system')
    hardware = ET.SubElement(system, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'mock_components/GenericSystem'
    for leg in ('FR', 'FL', 'RR', 'RL'):
        for kind, position in (('hip', -.1 if leg in ('FR', 'RR') else .1), ('thigh', .8), ('calf', -1.5)):
            name = leg + '_' + kind + '_joint'
            ET.SubElement(robot, 'link', name=name + '_link')
            joint = ET.SubElement(robot, 'joint', name=name, type='revolute')
            ET.SubElement(joint, 'parent', link='base')
            ET.SubElement(joint, 'child', link=name + '_link')
            ET.SubElement(joint, 'limit', lower='-4', upper='4', effort='45', velocity='30')
            controlled = ET.SubElement(system, 'joint', name=name)
            for field in ('position', 'velocity', 'effort', 'kp', 'kd'):
                ET.SubElement(controlled, 'command_interface', name=field)
            for field, initial in (('position', position), ('velocity', 0), ('effort', 0), ('temperature', 25), ('valid', 1)):
                state = ET.SubElement(controlled, 'state_interface', name=field)
                ET.SubElement(state, 'param', name='initial_value').text = str(initial)
    gpio = ET.SubElement(system, 'gpio', name='custom_dog')
    for field in ('calibrate', 'emergency_stop'):
        ET.SubElement(gpio, 'command_interface', name=field)
    for field, initial in (('calibrated', 1), ('communication_ok', 1), ('physical_estop', 0)):
        state = ET.SubElement(gpio, 'state_interface', name=field)
        ET.SubElement(state, 'param', name='initial_value').text = str(initial)
    scoped = {'/' + namespace + '/' + name: settings for name, settings in config.items()}
    handle, temporary = tempfile.mkstemp(prefix='customdog-rl-mock-', suffix='.yaml')
    with os.fdopen(handle, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(scoped, stream)
    manager = '/' + namespace + '/controller_manager'
    return [
        Node(package='controller_manager', executable='ros2_control_node', namespace=namespace,
             parameters=[temporary, {'robot_description': ET.tostring(robot, encoding='unicode')}], output='screen'),
        Node(package='controller_manager', executable='spawner', namespace=namespace,
             arguments=['joint_state_broadcaster', '-c', manager], output='screen'),
        Node(package='controller_manager', executable='spawner', namespace=namespace,
             arguments=['rl_controller', '-c', manager], output='screen'),
        Node(package='custom_dog_rl', executable='mock_imu.py', namespace=namespace,
             parameters=[{'frame_id': parameters['imu_frame']}], output='screen'),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=_cleanup, kwargs={"path": temporary})])),
    ]


def generate_launch_description():
    package = get_package_share_directory('custom_dog_rl')
    return LaunchDescription([
        DeclareLaunchArgument('model_path'),
        DeclareLaunchArgument('expected_model_sha256', default_value='10bcd8da253aba3e21dad7a417a73258a5753b30bc04ef63433ec706852fa54c'),
        DeclareLaunchArgument('config_file', default_value=os.path.join(package, 'config', 'rl_controller.yaml')),
        DeclareLaunchArgument('namespace', default_value='rl_mock'),
        DeclareLaunchArgument('enable_actuation', default_value='false'),
        OpaqueFunction(function=_launch),
    ])
