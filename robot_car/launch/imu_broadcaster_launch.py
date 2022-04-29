import launch
import launch_ros

def generate_launch_description():

    imu_broadcaster = launch_ros.actions.Node(
        package    = 'robot_car',
        executable = 'imu_broadcaster',
        name       = 'imu_broadcaster'
    )

    return launch.LaunchDescription([
        imu_broadcaster
    ])
