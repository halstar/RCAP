import launch
import launch_ros

def generate_launch_description():

    wall_follower = launch_ros.actions.Node(
        package    = 'robot_car',
        executable = 'wall_follower.py',
        name       = 'wall_follower'
    )

    return launch.LaunchDescription([
        wall_follower
    ])
