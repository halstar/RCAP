import launch
import launch_ros

def generate_launch_description():

    drive_controller = launch_ros.actions.Node(
        package    = 'robot_car',
        executable = 'drive_controller.py',
        name       = 'drive_controller'
    )

    return launch.LaunchDescription([
        drive_controller
    ])
