from launch               import LaunchDescription
from launch.actions       import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions   import Node


def generate_launch_description():
    serial_port      = LaunchConfiguration('serial_port'     , default='/dev/ttyUSB0')
    serial_baudrate  = LaunchConfiguration('serial_baudrate' , default='115200')
    frame_id         = LaunchConfiguration('frame_id'        , default='lidar_link')
    inverted         = LaunchConfiguration('inverted'        , default='False')
    angle_compensate = LaunchConfiguration('angle_compensate', default='True')

    return LaunchDescription([

        DeclareLaunchArgument(
            'serial_port',
            default_value = serial_port,
            description   = 'USB port connected to LIDAR'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value = serial_baudrate,
            description   = 'Baudrate to use'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value = frame_id,
            description   = 'Output frame identifier'),

        DeclareLaunchArgument(
            'inverted',
            default_value = inverted,
            description   = 'Whether invert or not scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value = angle_compensate,
            description   = 'Whether to compensate or not scan angle'),


        Node(
            package    = 'rplidar',
            executable = 'rplidar_scan_publisher',
            name       = 'rplidar_scan_publisher',
            parameters = [{'serial_port'   : serial_port, 
                         'serial_baudrate' : serial_baudrate, 
                         'frame_id'        : frame_id,
                         'inverted'        : inverted, 
                         'angle_compensate': angle_compensate}],
            output     = 'screen'),
    ])

