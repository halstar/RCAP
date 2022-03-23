from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('mpu9250driver'),
        'config',
        'imu_calibration.yaml'
        )

    mpu9250driver_node = Node(
        package='mpu9250driver',
        executable='mpu9250driver',
        name='mpu9250driver_node',
        parameters=[config]
    )

    ld.add_action(mpu9250driver_node)
    return ld
