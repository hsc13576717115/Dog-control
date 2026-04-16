from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 简单启动 imu_tf_node，用于将 /imu 姿态广播成 TF 方便可视化检查。
    imu_tf=Node(
        package="fdilink_ahrs",
        executable="imu_tf_node",
        parameters=[{'imu_topic':'/imu',
        'world_frame_id':'/world',
        'imu_frame_id':'/gyro_link',
        'position_x':1,
        'position_y':1,
        'position_z':1,
        }],
    )

    launch_description =LaunchDescription([imu_tf])
    return launch_description
