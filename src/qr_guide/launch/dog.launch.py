import os
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


LEG_ORDER = ('FR', 'FL', 'RR', 'RL')
LEG_COLOR_MATERIAL = {
    'FR': 'accent_orange',
    'FL': 'accent_cyan',
    'RR': 'accent_green',
    'RL': 'accent_pink',
}


def _format_xyz(values):
    return f'{values[0]:.6f} {values[1]:.6f} {values[2]:.6f}'


def _build_leg_block(leg_name, robot_cfg):
    hip_mount = robot_cfg['hip_mounts_in_body'][leg_name]
    l0 = robot_cfg['leg_geometry']['l0']
    l1 = robot_cfg['leg_geometry']['l1']
    l2 = robot_cfg['leg_geometry']['l2']
    foot_radius = robot_cfg['leg_geometry'].get('foot_radius_m', 0.025)
    joint_limits = robot_cfg['joint_limits']
    is_front = leg_name in ('FR', 'FL')
    is_right = leg_name in ('FR', 'RR')

    hip_axis_sign = -1.0 if is_front else 1.0
    plane_offset_y = l0 if is_right else -l0
    thigh_axis_sign = -1.0 if is_right else 1.0
    calf_axis_sign = 1.0 if is_right else -1.0
    hip_span = max(abs(plane_offset_y), 1e-3)
    material = LEG_COLOR_MATERIAL[leg_name]

    return f'''
  <link name="{leg_name}_hip_link">
    <visual>
      <origin xyz="0 {plane_offset_y / 2.0:.6f} 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.032000 {hip_span:.6f} 0.032000"/>
      </geometry>
      <material name="{material}"/>
    </visual>
  </link>

  <joint name="{leg_name}_hip" type="revolute">
    <parent link="base_link_est"/>
    <child link="{leg_name}_hip_link"/>
    <origin xyz="{_format_xyz(hip_mount)}" rpy="0 0 0"/>
    <axis xyz="{hip_axis_sign:.1f} 0 0"/>
    <limit lower="{joint_limits['q0'][0]:.6f}" upper="{joint_limits['q0'][1]:.6f}" effort="120.0" velocity="40.0"/>
  </joint>

  <link name="{leg_name}_thigh_link">
    <visual>
      <origin xyz="{l1 / 2.0:.6f} 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="{l1:.6f} 0.045000 0.045000"/>
      </geometry>
      <material name="metal_gray"/>
    </visual>
  </link>

  <joint name="{leg_name}_thigh" type="revolute">
    <parent link="{leg_name}_hip_link"/>
    <child link="{leg_name}_thigh_link"/>
    <origin xyz="0 {plane_offset_y:.6f} 0" rpy="0 0 0"/>
    <axis xyz="0 {thigh_axis_sign:.1f} 0"/>
    <limit lower="{joint_limits['q1'][0]:.6f}" upper="{joint_limits['q1'][1]:.6f}" effort="120.0" velocity="40.0"/>
  </joint>

  <link name="{leg_name}_calf_link">
    <visual>
      <origin xyz="{l2 / 2.0:.6f} 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="{l2:.6f} 0.038000 0.038000"/>
      </geometry>
      <material name="metal_gray"/>
    </visual>
  </link>

  <joint name="{leg_name}_calf" type="revolute">
    <parent link="{leg_name}_thigh_link"/>
    <child link="{leg_name}_calf_link"/>
    <origin xyz="{l1:.6f} 0 0" rpy="0 -1.57079632679 0"/>
    <axis xyz="0 {calf_axis_sign:.1f} 0"/>
    <limit lower="{joint_limits['q2'][0]:.6f}" upper="{joint_limits['q2'][1]:.6f}" effort="120.0" velocity="40.0"/>
  </joint>

  <link name="{leg_name}_foot_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="{foot_radius:.6f}"/>
      </geometry>
      <material name="foot_gold"/>
    </visual>
  </link>

  <joint name="{leg_name}_foot" type="fixed">
    <parent link="{leg_name}_calf_link"/>
    <child link="{leg_name}_foot_link"/>
    <origin xyz="{l2:.6f} 0 0" rpy="0 0 0"/>
  </joint>'''


def _build_robot_description(qr_guide_share):
    config_path = Path(qr_guide_share) / 'config' / 'custom_quadruped.yaml'
    template_path = Path(qr_guide_share) / 'urdf' / 'custom_quadruped.urdf.template'

    robot_cfg = yaml.safe_load(config_path.read_text(encoding='utf-8'))['robot']
    template = template_path.read_text(encoding='utf-8')
    leg_blocks = '\n'.join(_build_leg_block(leg_name, robot_cfg) for leg_name in LEG_ORDER)

    return template.format(
        robot_name=robot_cfg['name'],
        body_x=f'{robot_cfg["body_size_m"][0]:.6f}',
        body_y=f'{robot_cfg["body_size_m"][1]:.6f}',
        body_z=f'{robot_cfg["body_size_m"][2]:.6f}',
        leg_blocks=leg_blocks,
    )


def generate_launch_description():
    qr_guide_share = get_package_share_directory('qr_guide')
    rviz_config = os.path.join(qr_guide_share, 'rviz', 'qr_guide_visualization.rviz')
    robot_description = _build_robot_description(qr_guide_share)

    imu_serial_port = LaunchConfiguration('imu_serial_port')
    imu_serial_baud = LaunchConfiguration('imu_serial_baud')
    joy_event_path = LaunchConfiguration('joy_event_path')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('imu_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('imu_serial_baud', default_value='921600'),
        DeclareLaunchArgument('joy_event_path', default_value='/dev/input/event6'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
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
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='qr_guide_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
