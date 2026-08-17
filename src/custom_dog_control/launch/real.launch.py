import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('custom_dog_control')
    description_share = get_package_share_directory('custom_dog_description')
    controllers = os.path.join(package_share, 'config', 'controllers.yaml')
    real_override = os.path.join(
        package_share, 'config', 'real_controller.yaml')
    xacro_file = os.path.join(
        package_share, 'urdf', 'custom_dog.ros2_control.xacro')
    canonical_urdf = os.path.join(
        description_share, 'urdf', 'custom_dog.urdf')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' hardware_mode:=real',
            ' description_urdf:=', canonical_urdf,
            ' physical_estop_verified:=',
            LaunchConfiguration('physical_estop_verified'),
            ' fr_port:=', LaunchConfiguration('fr_port'),
            ' fl_port:=', LaunchConfiguration('fl_port'),
            ' rr_port:=', LaunchConfiguration('rr_port'),
            ' rl_port:=', LaunchConfiguration('rl_port'),
            ' calibration_hip_deg:=',
            LaunchConfiguration('calibration_hip_deg'),
            ' calibration_thigh_deg:=',
            LaunchConfiguration('calibration_thigh_deg'),
            ' calibration_calf_deg:=',
            LaunchConfiguration('calibration_calf_deg'),
        ]),
        value_type=str,
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controllers,
            real_override,
            {'robot_description': robot_description},
        ],
        output='screen',
    )
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )
    imu = Node(
        package='fdilink_ahrs',
        executable='ahrs_driver_node',
        name='ahrs_driver',
        condition=IfCondition(LaunchConfiguration('start_imu')),
        parameters=[{
            'if_debug_': False,
            'serial_port_': LaunchConfiguration('imu_port'),
            'serial_baud_': ParameterValue(
                LaunchConfiguration('imu_baud'), value_type=int),
            'imu_topic': '/imu',
            'imu_frame_id_': 'base',
            'device_type_': 1,
        }],
        output='screen',
    )
    joy = Node(
        package='custom_dog_control',
        executable='event2joy.py',
        name='event2joy_node',
        condition=IfCondition(LaunchConfiguration('start_joy')),
        parameters=[{
            'event_path': LaunchConfiguration('joy_event_path'),
            'publish_hz': 250.0,
        }],
        output='screen',
    )
    load_controllers = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['nmpc_wbc_controller'],
                output='screen',
            ),
        ],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            package_share, 'rviz', 'custom_dog_control.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'physical_estop_verified', default_value='false'),
        DeclareLaunchArgument('fr_port', default_value='/dev/ttyS3'),
        DeclareLaunchArgument('fl_port', default_value='/dev/ttyS4'),
        DeclareLaunchArgument('rr_port', default_value='/dev/ttyS7'),
        DeclareLaunchArgument('rl_port', default_value='/dev/ttyS8'),
        DeclareLaunchArgument('calibration_hip_deg', default_value='0.0'),
        DeclareLaunchArgument('calibration_thigh_deg', default_value='71.8'),
        DeclareLaunchArgument('calibration_calf_deg', default_value='-161.8'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('imu_baud', default_value='921600'),
        DeclareLaunchArgument('joy_event_path', default_value='auto'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument('start_joy', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        control_node,
        state_publisher,
        imu,
        joy,
        load_controllers,
        rviz,
    ])
