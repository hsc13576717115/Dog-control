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


def _format_xyz(values):
    return f'{values[0]:.6f} {values[1]:.6f} {values[2]:.6f}'


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


def _disk_visual(center_xyz, radius, thickness, axis, material):
    return _cylinder_visual(center_xyz, radius, thickness, axis, material)


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


def _rounded_box_visuals(center_xyz, size_xyz, corner_radius, shell_material, cap_material=None):
    cx, cy, cz = center_xyz
    sx, sy, sz = size_xyz
    radius = min(corner_radius, sx * 0.49, sy * 0.49, sz * 0.49)
    cap_material = cap_material or shell_material

    x_core = max(sx - 2.0 * radius, 1e-6)
    y_core = max(sy - 2.0 * radius, 1e-6)
    z_core = max(sz - 2.0 * radius, 1e-6)
    x_edge = sx * 0.5 - radius
    y_edge = sy * 0.5 - radius
    z_edge = sz * 0.5 - radius

    parts = [
        _box_visual((cx, cy, cz), (x_core, sy, z_core), shell_material),
        _box_visual((cx, cy, cz), (sx, y_core, z_core), shell_material),
        _box_visual((cx, cy, cz), (x_core, y_core, sz), shell_material),
    ]

    if x_core > 1e-6:
        for y_sign in (-1.0, 1.0):
            for z_sign in (-1.0, 1.0):
                parts.append(
                    _cylinder_visual(
                        (cx, cy + y_sign * y_edge, cz + z_sign * z_edge),
                        radius,
                        x_core,
                        'x',
                        shell_material,
                    )
                )

    if y_core > 1e-6:
        for x_sign in (-1.0, 1.0):
            for z_sign in (-1.0, 1.0):
                parts.append(
                    _cylinder_visual(
                        (cx + x_sign * x_edge, cy, cz + z_sign * z_edge),
                        radius,
                        y_core,
                        'y',
                        shell_material,
                    )
                )

    if z_core > 1e-6:
        for x_sign in (-1.0, 1.0):
            for y_sign in (-1.0, 1.0):
                parts.append(
                    _cylinder_visual(
                        (cx + x_sign * x_edge, cy + y_sign * y_edge, cz),
                        radius,
                        z_core,
                        'z',
                        shell_material,
                    )
                )

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            for z_sign in (-1.0, 1.0):
                parts.append(
                    _joint_visual(
                        (cx + x_sign * x_edge, cy + y_sign * y_edge, cz + z_sign * z_edge),
                        radius,
                        cap_material,
                    )
                )

    return ''.join(parts)


def _axis_point(axis, value):
    if axis == 'x':
        return (value, 0.0, 0.0)
    if axis == 'y':
        return (0.0, value, 0.0)
    return (0.0, 0.0, value)


def _capsule_visuals_along_axis(axis, start, end, radius, shell_material, cap_material=None):
    cap_material = cap_material or shell_material
    total_length = abs(end - start)
    if total_length < 1e-9:
        return _joint_visual(_axis_point(axis, start), radius, cap_material)

    parts = []
    cylinder_length = max(total_length - 2.0 * radius, 0.0)
    if cylinder_length > 1e-6:
        center = 0.5 * (start + end)
        parts.append(_cylinder_visual(_axis_point(axis, center), radius, cylinder_length, axis, shell_material))
    parts.append(_joint_visual(_axis_point(axis, start), radius, cap_material))
    parts.append(_joint_visual(_axis_point(axis, end), radius, cap_material))
    return ''.join(parts)


def _rod_visual_along_axis(axis, start, end, radius, material):
    total_length = abs(end - start)
    if total_length < 1e-9:
        return ''
    center = 0.5 * (start + end)
    return _cylinder_visual(_axis_point(axis, center), radius, total_length, axis, material)


def _build_body_visuals(robot_cfg):
    body_x, body_y, body_z = robot_cfg['body_size_m']
    main_corner_radius = min(body_y, body_z) * 0.16
    top_corner_radius = min(body_y, body_z) * 0.11
    return ''.join([
        _rounded_box_visuals((0.0, 0.0, 0.0), (body_x, body_y, body_z), main_corner_radius, 'pure_white'),
        _rounded_box_visuals(
            (body_x * 0.10, 0.0, body_z * 0.13),
            (body_x * 0.68, body_y * 0.78, body_z * 0.34),
            top_corner_radius,
            'pure_white',
            'pure_white',
        ),
        _rounded_box_visuals(
            (body_x * 0.30, 0.0, body_z * 0.02),
            (body_x * 0.16, body_y * 0.46, body_z * 0.20),
            top_corner_radius * 0.65,
            'pure_white',
        ),
        _joint_visual((body_x * 0.41, body_y * 0.11, body_z * 0.10), body_z * 0.055, 'accent_cyan'),
        _joint_visual((body_x * 0.41, -body_y * 0.11, body_z * 0.10), body_z * 0.055, 'accent_cyan'),
    ])


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
    hip_motor_radius = min(max(abs(plane_offset_y) * 0.24, 0.016), 0.024)
    thigh_motor_radius = min(max(l1 * 0.085, 0.016), 0.022)
    knee_motor_radius = min(max(l2 * 0.075, 0.014), 0.020)
    thigh_radius = min(max(l1 * 0.042, 0.008), 0.012)
    calf_radius = min(max(l2 * 0.032, 0.006), 0.010)
    foot_pad_radius = max(foot_radius * 0.82, 0.012)
    foot_pad_height = max(foot_radius * 0.70, 0.014)
    hip_disk_thickness = max(hip_motor_radius * 0.55, 0.010)
    joint_disk_thickness = max(thigh_motor_radius * 0.52, 0.010)
    knee_disk_thickness = max(knee_motor_radius * 0.52, 0.010)
    foot_pad_thickness = max(foot_pad_height * 0.72, 0.010)

    return f'''
  <link name="{leg_name}_hip_link">
{_disk_visual((0.0, 0.0, 0.0), hip_motor_radius, hip_disk_thickness, 'x', 'body_black')}
{_rod_visual_along_axis('y', 0.0, plane_offset_y, thigh_radius, 'pure_white')}
{_disk_visual((0.0, plane_offset_y, 0.0), thigh_motor_radius, joint_disk_thickness, 'y', 'body_black')}
  </link>

  <joint name="{leg_name}_hip" type="revolute">
    <parent link="base_link_est"/>
    <child link="{leg_name}_hip_link"/>
    <origin xyz="{_format_xyz(hip_mount)}" rpy="0 0 0"/>
    <axis xyz="{hip_axis_sign:.1f} 0 0"/>
    <limit lower="{joint_limits['q0'][0]:.6f}" upper="{joint_limits['q0'][1]:.6f}" effort="120.0" velocity="40.0"/>
  </joint>

  <link name="{leg_name}_thigh_link">
{_disk_visual((0.0, 0.0, 0.0), thigh_motor_radius, joint_disk_thickness, 'y', 'body_black')}
{_rod_visual_along_axis('x', 0.0, l1, thigh_radius, 'pure_white')}
{_disk_visual((l1, 0.0, 0.0), knee_motor_radius, knee_disk_thickness, 'y', 'body_black')}
  </link>

  <joint name="{leg_name}_thigh" type="revolute">
    <parent link="{leg_name}_hip_link"/>
    <child link="{leg_name}_thigh_link"/>
    <origin xyz="0 {plane_offset_y:.6f} 0" rpy="0 0 0"/>
    <axis xyz="0 {thigh_axis_sign:.1f} 0"/>
    <limit lower="{joint_limits['q1'][0]:.6f}" upper="{joint_limits['q1'][1]:.6f}" effort="120.0" velocity="40.0"/>
  </joint>

  <link name="{leg_name}_calf_link">
{_disk_visual((0.0, 0.0, 0.0), knee_motor_radius, knee_disk_thickness, 'y', 'body_black')}
{_rod_visual_along_axis('x', 0.0, l2, calf_radius, 'pure_white')}
  </link>

  <joint name="{leg_name}_calf" type="revolute">
    <parent link="{leg_name}_thigh_link"/>
    <child link="{leg_name}_calf_link"/>
    <origin xyz="{l1:.6f} 0 0" rpy="0 -1.57079632679 0"/>
    <axis xyz="0 {calf_axis_sign:.1f} 0"/>
    <limit lower="{joint_limits['q2'][0]:.6f}" upper="{joint_limits['q2'][1]:.6f}" effort="120.0" velocity="40.0"/>
  </joint>

  <link name="{leg_name}_foot_link">
{_joint_visual((0.0, 0.0, 0.0), foot_pad_radius, 'pure_white')}
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
