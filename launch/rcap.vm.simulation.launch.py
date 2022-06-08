import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions       import TimerAction
import launch_ros
import os

def generate_launch_description():
    robot_car_pkg_share      = launch_ros.substitutions.FindPackageShare(package = 'robot_car').find('robot_car')
    rcap_pkg_share           = launch_ros.substitutions.FindPackageShare(package = 'rcap'     ).find('rcap')
    slam_toolbox_pkg_share   = launch_ros.substitutions.FindPackageShare(package = 'slam_toolbox').find('slam_toolbox')
    model_path               = os.path.join(robot_car_pkg_share, 'description/robot_car.vm.simulation.urdf')
    world_path               = os.path.join(robot_car_pkg_share, 'world/my_world.sdf'),
    rviz_config_path         = os.path.join(rcap_pkg_share     , 'rviz/config.simulation.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    spawn_gazebo_entity = launch_ros.actions.Node(
        package    = 'gazebo_ros',
        executable =  'spawn_entity.py',
        arguments  = ['-entity', 'robot_car', '-topic', 'robot_description'],
        output     = 'screen'
    )
    robot_localization_node = launch_ros.actions.Node(
        package    = 'robot_localization',
        executable = 'ekf_node',
        name       = 'ekf_filter_node',
        output     = 'screen',
        parameters = [os.path.join(robot_car_pkg_share, 'config/ekf.simulation.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    slam_toolbox_node = launch_ros.actions.Node(
        package    = 'slam_toolbox',
        executable = 'async_slam_toolbox_node',
        name       = 'slam_toolbox',
        output     = 'screen',
        parameters = [os.path.join(slam_toolbox_pkg_share, 'config', 'mapper_params_online_async.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )
    rviz_node = launch_ros.actions.Node(
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name = 'model'       , default_value = model_path      , description = 'Absolute path to robot urdf file' ),
        launch.actions.DeclareLaunchArgument(name = 'rvizconfig'  , default_value = rviz_config_path, description = 'Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name = 'use_sim_time', default_value = 'True'          , description = 'Flag to enable use_sim_time'      ),
        launch.actions.ExecuteProcess       (cmd  = ['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path], output = 'screen'           ),
        
        robot_state_publisher_node,
        spawn_gazebo_entity,
        robot_localization_node,
        slam_toolbox_node,
        TimerAction(period = 7.0, actions = [rviz_node])
    ])
