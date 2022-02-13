import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    robot_car_pkg_share = launch_ros.substitutions.FindPackageShare(package = 'robot_car').find('robot_car')
    model_path          = os.path.join(robot_car_pkg_share, 'description/robot_car.vm.rqt.steering.urdf')
    world_path          = os.path.join(robot_car_pkg_share, 'world/my_world.sdf'),

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
    steering_node = launch_ros.actions.Node(
        package    = 'rqt_robot_steering',
        executable = 'rqt_robot_steering',
        name       = 'rqt_robot_steering',
        output     = 'screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name = 'model'       , default_value = model_path, description = 'Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name = 'use_sim_time', default_value = 'True'    , description = 'Flag to enable use_sim_time'     ),
        launch.actions.ExecuteProcess       (cmd  = ['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path], output = 'screen'    ),
        
        robot_state_publisher_node,
        spawn_gazebo_entity,
        steering_node,
    ])
