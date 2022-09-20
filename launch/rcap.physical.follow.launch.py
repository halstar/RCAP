import launch
from   launch.substitutions              import Command, LaunchConfiguration
from   launch.actions                    import IncludeLaunchDescription
from   launch.launch_description_sources import PythonLaunchDescriptionSource
from   ament_index_python.packages       import get_package_share_directory
import launch_ros
import os

def generate_launch_description():
    robot_car_pkg_share    = launch_ros.substitutions.FindPackageShare(package = 'robot_car').find('robot_car')
    slam_toolbox_pkg_share = launch_ros.substitutions.FindPackageShare(package = 'slam_toolbox').find('slam_toolbox')
    model_path             = os.path.join(robot_car_pkg_share, 'description/robot_car.physical.urdf')

    robot_state_publisher_node = launch_ros.actions.Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    robot_localization_node = launch_ros.actions.Node(
        package    = 'robot_localization',
        executable = 'ekf_node',
        name       = 'ekf_filter_node',
        output     = 'screen',
        parameters = [os.path.join(robot_car_pkg_share, 'config/ekf.physical.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )    
    slam_toolbox_node = launch_ros.actions.Node(
        package    = 'slam_toolbox',
        executable = 'async_slam_toolbox_node',
        name       = 'slam_toolbox',
        output     = 'screen',
        parameters = [os.path.join(slam_toolbox_pkg_share, 'config', 'mapper_params_online_async.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )
    drive_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
           get_package_share_directory('robot_car'), 'launch'), '/drive_controller_launch.py'])
    )
    wall_follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
           get_package_share_directory('robot_car'), 'launch'), '/wall_follower_launch.py'])
    )
    mpu9250_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
           get_package_share_directory('mpu9250driver'), 'launch'), '/mpu9250driver_launch.py'])
    )
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
           get_package_share_directory('rplidar'), 'launch'), '/rplidar_launch.py'])
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name = 'model'       , default_value = model_path, description = 'Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name = 'use_sim_time', default_value = 'False'   , description = 'Flag to enable use_sim_time'     ),

        robot_state_publisher_node,
        robot_localization_node,
        slam_toolbox_node,
        drive_controller_launch,
        wall_follower_launch,
        mpu9250_launch,
        rplidar_launch
    ])
