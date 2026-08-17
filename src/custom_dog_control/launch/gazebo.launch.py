import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _continue_after_success(next_actions, step):
    def callback(event, _context):
        if event.returncode == 0:
            return next_actions
        reason = f'{step} failed with exit code {event.returncode}'
        return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]

    return callback


def generate_launch_description():
    package_share = get_package_share_directory('custom_dog_control')
    description_share = get_package_share_directory('custom_dog_description')
    gazebo_share = get_package_share_directory('gazebo_ros')
    controllers = os.path.join(package_share, 'config', 'controllers.yaml')
    gazebo_control_plugin = os.path.join(
        get_package_prefix('gazebo_ros2_control'),
        'lib',
        'libgazebo_ros2_control.so',
    )
    xacro_file = os.path.join(
        package_share, 'urdf', 'custom_dog.ros2_control.xacro')
    point_foot_urdf = os.path.join(
        description_share, 'urdf', 'custom_dog_gazebo_point_foot.urdf')
    default_world = os.path.join(package_share, 'worlds', 'flat.world')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' hardware_mode:=gazebo',
            ' description_urdf:=', point_foot_urdf,
            ' controllers_file:=', controllers,
            ' gazebo_control_plugin:=', gazebo_control_plugin,
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'false',
            'gdb': LaunchConfiguration('gdb'),
            'gui': LaunchConfiguration('gui'),
            'pause': 'true',
        }.items(),
    )
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'custom_dog',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0',
            '-z', LaunchConfiguration('spawn_z'),
        ],
        output='screen',
    )
    inactive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'nmpc_wbc_controller',
            '--inactive',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60.0',
            '--service-call-timeout', '60.0',
        ],
        output='screen',
    )
    activate_controllers = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'switch_controllers',
            '--activate',
            'joint_state_broadcaster', 'nmpc_wbc_controller',
            '--strict', '--activate-asap',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )
    unpause_physics = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/unpause_physics',
            'std_srvs/srv/Empty', '{}',
        ],
        output='screen',
    )
    keyboard = Node(
        package='custom_dog_control',
        executable='keyboard_teleop.py',
        name='keyboard_teleop',
        condition=IfCondition(LaunchConfiguration('start_keyboard')),
        output='screen',
        emulate_tty=True,
    )
    load_joint_state_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=_continue_after_success(
                [inactive_controller_spawner], 'Robot spawn'),
        )
    )
    activate_after_configuration = RegisterEventHandler(
        OnProcessExit(
            target_action=inactive_controller_spawner,
            on_exit=_continue_after_success(
                [activate_controllers], 'Controller configuration'),
        )
    )
    unpause_after_switch_request = RegisterEventHandler(
        OnProcessStart(
            target_action=activate_controllers,
            on_start=[TimerAction(period=1.0, actions=[unpause_physics])],
        )
    )
    verify_activation = RegisterEventHandler(
        OnProcessExit(
            target_action=activate_controllers,
            on_exit=_continue_after_success(
                [
                    LogInfo(
                        msg='Controllers active and Gazebo physics running.'),
                    keyboard,
                ],
                'Controller activation',
            ),
        )
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            package_share, 'rviz', 'custom_dog_control.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    gazebo_plugin_path = os.pathsep.join(filter(None, [
        os.path.join('/opt/ros', os.environ.get('ROS_DISTRO', 'humble'), 'lib'),
        os.environ.get('GAZEBO_PLUGIN_PATH', ''),
    ]))
    gazebo_model_path = os.pathsep.join(filter(None, [
        os.path.dirname(description_share),
        '/usr/share/gazebo-11/models',
        os.environ.get('GAZEBO_MODEL_PATH', ''),
    ]))

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('start_keyboard', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('spawn_z', default_value='0.055'),
        DeclareLaunchArgument('gdb', default_value='false'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', gazebo_plugin_path),
        gazebo,
        state_publisher,
        spawn_robot,
        load_joint_state_after_spawn,
        activate_after_configuration,
        unpause_after_switch_request,
        verify_activation,
        rviz,
    ])
