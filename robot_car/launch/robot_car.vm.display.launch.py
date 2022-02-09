import launch
from launch.substitutions import LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    robot_car_pkg_share = launch_ros.substitutions.FindPackageShare(package = 'robot_car').find('robot_car')
    rviz_config_path    = os.path.join(robot_car_pkg_share, 'rviz/urdf_config.physical.rviz')

    rviz_node = launch_ros.actions.Node(
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name = 'rvizconfig'  , default_value = rviz_config_path, description = 'Absolute path to rviz config file'),
        

        rviz_node
    ])

