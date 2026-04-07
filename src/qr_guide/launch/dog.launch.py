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
    'FR': 'metal_gray',
    'FL': 'metal_gray',
    'RR': 'metal_gray',
    'RL': 'metal_gray',
}
JOINT_MATERIAL = 'metal_gray'


def _format_xyz(values):
    return f'{values[0]:.6f} {values[1]:.6f} {values[2]:.6f}'


def _capsule_visual(center_xyz, length, radius, axis, material):
    if axis == 'x':
        cylinder_rpy = '0 1.57079632679 0'
        end_a = (-length / 2.0, 0.0, 0.0)
        end_b = (length / 2.0, 0.0, 0.0)
    elif axis == 'y':
        cylinder_rpy = '1.57079632679 0 0'
        end_a = (0.0, -length / 2.0, 0.0)
        end_b = (0.0, length / 2.0, 0.0)
    else:
        cylinder_rpy = '0 0 0'
        end_a = (0.0, 0.0, -length / 2.0)
        end_b = (0.0, 0.0, length / 2.0)

    cx, cy, cz = center_xyz
    ax, ay, az = end_a
    bx, by, bz = end_b

    return f'''
    <visual>
      <origin xyz="{cx:.6f} {cy:.6f} {cz:.6f}" rpy="{cylinder_rpy}"/>
      <geometry>
        <cylinder radius="{radius:.6f}" length="{length:.6f}"/>
      </geometry>
      <material name="{material}"/>
    </visual>
    <visual>
      <origin xyz="{cx + ax:.6f} {cy + ay:.6f} {cz + az:.6f}" rpy="0 0 0"/>
      <geometry>
        <sphere radius="{radius:.6f}"/>
      </geometry>
      <material name="{material}"/>
    </visual>
    <visual>
      <origin xyz="{cx + bx:.6f} {cy + by:.6f} {cz + bz:.6f}" rpy="0 0 0"/>
      <geometry>
        <sphere radius="{radius:.6f}"/>
      </geometry>
      <material name="{material}"/>
    </visual>'''


def _joint_visual(origin_xyz, radius, material):
    ox, oy, oz = origin_xyz
    return f'''
    <visual>
      <origin xyz="{ox:.6f} {oy:.6f} {oz:.6f}" rpy="0 0 0"/>
      <geometry>
        <sphere radius="{radius:.6f}"/>
      </geometry>
      <material name="{material}"/>
    </visual>'''


def _box_visual(center_xyz, size_xyz, material):
    cx, cy, cz = center_xyz
    sx, sy, sz = size_xyz
    return f'''
    <visual>
      <origin xyz="{cx:.6f} {cy:.6f} {cz:.6f}" rpy="0 0 0"/>
      <geometry>
        <box size="{sx:.6f} {sy:.6f} {sz:.6f}"/>
      </geometry>
      <material name="{material}"/>
    </visual>'''


def _cylinder_visual(center_xyz, radius, length, axis, material):
    cx, cy, cz = center_xyz
    if axis == 'x':
        rpy = '0 1.57079632679 0'
    elif axis == 'y':
        rpy = '1.57079632679 0 0'
    else:
        rpy = '0 0 0'
    return f'''
    <visual>
      <origin xyz="{cx:.6f} {cy:.6f} {cz:.6f}" rpy="{rpy}"/>
      <geometry>
        <cylinder radius="{radius:.6f}" length="{length:.6f}"/>
      </geometry>
      <material name="{material}"/>
    </visual>'''


def _build_body_visuals(robot_cfg):
    body_x, body_y, body_z = robot_cfg['body_size_m']
    upper_x = body_x * 0.72
    upper_y = body_y * 0.70
    upper_z = body_z * 0.44
    lower_x = body_x * 0.62
    lower_y = body_y * 0.44
    lower_z = body_z * 0.24
    side_radius = body_z * 0.18
    shoulder_length = body_x * 0.70
    front_bumper_x = body_x * 0.16
    front_bumper_y = body_y * 0.50
    front_bumper_z = body_z * 0.20
    rear_pack_x = body_x * 0.18
    rear_pack_y = body_y * 0.42
    rear_pack_z = body_z * 0.18
    lidar_radius = body_y * 0.10
    lidar_height = body_z * 0.16
    lidar_cap_radius = body_y * 0.13
    lamp_radius = body_z * 0.065
    camera_bar_x = body_x * 0.16
    camera_bar_y = body_y * 0.26
    camera_bar_z = body_z * 0.10

    parts = []
    parts.append(_box_visual((0.0, 0.0, 0.0), (upper_x, upper_y, upper_z), 'body_navy'))
    parts.append(_cylinder_visual((0.0, body_y * 0.24, 0.0), side_radius, shoulder_length, 'x', 'body_navy'))
    parts.append(_cylinder_visual((0.0, -body_y * 0.24, 0.0), side_radius, shoulder_length, 'x', 'body_navy'))
    parts.append(_box_visual((0.0, 0.0, -body_z * 0.20), (lower_x, lower_y, lower_z), 'metal_gray'))
    parts.append(_box_visual((body_x * 0.34, 0.0, body_z * 0.01),
                             (front_bumper_x, front_bumper_y, front_bumper_z),
                             'body_navy'))
    parts.append(_box_visual((-body_x * 0.33, 0.0, -body_z * 0.01),
                             (rear_pack_x, rear_pack_y, rear_pack_z),
                             'body_navy'))
    parts.append(_box_visual((body_x * 0.37, 0.0, body_z * 0.085),
                             (camera_bar_x, camera_bar_y, camera_bar_z),
                             'metal_gray'))
    parts.append(_joint_visual((body_x * 0.39, body_y * 0.085, body_z * 0.082), lamp_radius, 'accent_cyan'))
    parts.append(_joint_visual((body_x * 0.39, -body_y * 0.085, body_z * 0.082), lamp_radius, 'accent_cyan'))
    parts.append(_cylinder_visual((0.0, 0.0, body_z * 0.34), lidar_radius, lidar_height, 'z', 'metal_gray'))
    parts.append(_joint_visual((0.0, 0.0, body_z * 0.43), lidar_cap_radius, 'accent_green'))
    return ''.join(parts)


def _build_leg_block(leg_name, robot_cfg):
    hip_mount_raw = robot_cfg['hip_mounts_in_body'][leg_name]
    hip_mount = [hip_mount_raw[0], -hip_mount_raw[1], hip_mount_raw[2]]
    l0 = robot_cfg['leg_geometry']['l0']
    l1 = robot_cfg['leg_geometry']['l1']
    l2 = robot_cfg['leg_geometry']['l2']
    foot_radius = robot_cfg['leg_geometry'].get('foot_radius_m', 0.025)
    joint_limits = robot_cfg['joint_limits']
    is_front = leg_name in ('FR', 'FL')
    is_right = leg_name in ('FR', 'RR')

    # 控制器内部仍然使用 y 向机器人右侧；这里单独反射到 RViz/URDF 的 y 向左可视化坐标。
    hip_axis_sign = 1.0 if is_front else -1.0
    plane_offset_y = -l0 if is_right else l0
    thigh_axis_sign = -1.0 if is_right else 1.0
    calf_axis_sign = 1.0 if is_right else -1.0
    hip_span = max(abs(plane_offset_y), 1e-3)
    material = LEG_COLOR_MATERIAL[leg_name]
    hip_radius = 0.0105
    rod_radius = 0.0088
    joint_radius = 0.0130
    foot_visual_radius = max(foot_radius * 0.78, 0.014)

    return f'''
  <link name="{leg_name}_hip_link">
{_capsule_visual((0.0, plane_offset_y / 2.0, 0.0), hip_span, hip_radius, 'y', 'body_navy')}
{_joint_visual((0.0, 0.0, 0.0), joint_radius, 'body_navy')}
{_joint_visual((0.0, plane_offset_y, 0.0), joint_radius, JOINT_MATERIAL)}
  </link>

  <joint name="{leg_name}_hip" type="revolute">
    <parent link="base_link_est"/>
    <child link="{leg_name}_hip_link"/>
    <origin xyz="{_format_xyz(hip_mount)}" rpy="0 0 0"/>
    <axis xyz="{hip_axis_sign:.1f} 0 0"/>
    <limit lower="{joint_limits['q0'][0]:.6f}" upper="{joint_limits['q0'][1]:.6f}" effort="120.0" velocity="40.0"/>
  </joint>

  <link name="{leg_name}_thigh_link">
{_capsule_visual((l1 / 2.0, 0.0, 0.0), l1, rod_radius, 'x', 'metal_gray')}
{_joint_visual((0.0, 0.0, 0.0), joint_radius, 'body_navy')}
{_joint_visual((l1, 0.0, 0.0), joint_radius, JOINT_MATERIAL)}
  </link>

  <joint name="{leg_name}_thigh" type="revolute">
    <parent link="{leg_name}_hip_link"/>
    <child link="{leg_name}_thigh_link"/>
    <origin xyz="0 {plane_offset_y:.6f} 0" rpy="0 0 0"/>
    <axis xyz="0 {thigh_axis_sign:.1f} 0"/>
    <limit lower="{joint_limits['q1'][0]:.6f}" upper="{joint_limits['q1'][1]:.6f}" effort="120.0" velocity="40.0"/>
  </joint>

  <link name="{leg_name}_calf_link">
{_capsule_visual((l2 / 2.0, 0.0, 0.0), l2, rod_radius * 0.82, 'x', 'metal_gray')}
{_joint_visual((0.0, 0.0, 0.0), joint_radius * 0.96, JOINT_MATERIAL)}
{_joint_visual((l2, 0.0, 0.0), joint_radius * 0.90, 'foot_gold')}
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
        <sphere radius="{foot_visual_radius:.6f}"/>
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
        body_visuals=_build_body_visuals(robot_cfg),
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
