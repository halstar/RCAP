import launch
import launch_ros

def generate_launch_description():

    vel_forwarder = launch_ros.actions.Node(
        package    = 'robot_car',
        executable = 'vel_forwarder.py',
        name       = 'vel_forwarder'
    )

    return launch.LaunchDescription([
        vel_forwarder
    ])
