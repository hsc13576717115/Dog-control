from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    imu_serial_port = LaunchConfiguration('imu_serial_port')
    imu_serial_baud = LaunchConfiguration('imu_serial_baud')
    joy_event_path = LaunchConfiguration('joy_event_path')

    return LaunchDescription([
        DeclareLaunchArgument('imu_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('imu_serial_baud', default_value='921600'),
        DeclareLaunchArgument('joy_event_path', default_value='/dev/input/event6'),
        Node(
            package='fdilink_ahrs',
            executable='ahrs_driver_node',
            name='ahrs_driver',
            output='screen',
            parameters=[{
                'if_debug_': False,
                'serial_port_': imu_serial_port,
                'serial_baud_': ParameterValue(imu_serial_baud, value_type=int),
                'imu_topic': '/imu',
                'imu_frame_id_': 'gyro_link',
                'mag_pose_2d_topic': '/mag_pose_2d',
                'Magnetic_topic': '/magnetic',
                'Euler_angles_topic': '/euler_angles',
                'gps_topic': '/gps/fix',
                'twist_topic': '/system_speed',
                'NED_odom_topic': '/NED_odometry',
                'device_type_': 1,
            }],
        ),
        Node(
            package='qr_guide',
            executable='event2joy.py',
            name='event2joy_node',
            output='screen',
            parameters=[{
                'event_path': joy_event_path,
                'publish_hz': 1000.0,
            }],
        ),
        Node(
            package='qr_guide',
            executable='junior_ctrl',
            output='screen',
            emulate_tty=True,
        ),
    ])
