import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    robot_car_pkg_share = launch_ros.substitutions.FindPackageShare(package = 'robot_car').find('robot_car')
    rcap_pkg_share      = launch_ros.substitutions.FindPackageShare(package = 'rcap'     ).find('rcap')
    model_path          = os.path.join(robot_car_pkg_share, 'description/robot_car.vm.joint.state.gui.urdf')
    rviz_config_path    = os.path.join(rcap_pkg_share     , 'rviz/config.joint.state.gui.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package    ='joint_state_publisher_gui',
        executable ='joint_state_publisher_gui',
        name       ='joint_state_publisher_gui'
    )
    rviz_node = launch_ros.actions.Node(
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name = 'model'     , default_value = model_path      , description = 'Absolute path to robot URDF file' ),
        launch.actions.DeclareLaunchArgument(name = 'rvizconfig', default_value = rviz_config_path, description = 'Absolute path to RVIZ config file'),
        
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
