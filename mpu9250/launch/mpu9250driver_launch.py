from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros
import os

def generate_launch_description():

    config_path = os.path.join(get_package_share_directory('mpu9250driver'), 'config', 'imu_calibration.yaml')

    mpu9250driver = launch_ros.actions.Node(
        package    = 'mpu9250driver',
        executable = 'mpu9250driver',
        name       = 'mpu9250driver',
        parameters = [config_path]
    )

    return launch.LaunchDescription([
        mpu9250driver
    ])
