#!/usr/bin/env python3

import rclpy
import threading
import serial
import time
import math
import tf_transformations

from rclpy.node        import Node
from sensor_msgs.msg   import JointState
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg      import Odometry
from tf2_ros           import TransformBroadcaster

SPEED_TO_ANGLE_RATIO = 15.50

X_SPEED_FACTOR_CMD_VEL = 57.00
Y_SPEED_FACTOR_CMD_VEL = 60.00
Z_SPEED_FACTOR_CMD_VEL = 10.50

X_SPEED_FACTOR_ODOM = 0.0019
Y_SPEED_FACTOR_ODOM = 0.0018
Z_SPEED_FACTOR_ODOM = 0.0103

class DriveController(Node):

    def __init__(self):
        super().__init__('drive_controller')

        self.get_logger().info('Starting Drive Controller Node');

        self.saved_time_tx_in_ms = time.time() * 1000
        self.saved_time_rx_in_s  = time.time()

        self.start_time = 0.0

        self.last_command_bytes           = 0
        self.last_wheel_front_left_speed  = 0
        self.last_wheel_front_right_speed = 0
        self.last_wheel_rear_left_speed   = 0
        self.last_wheel_rear_right_speed  = 0

        self.serial_port          = serial.Serial()
        self.serial_port.port     = '/dev/serial0'
        self.serial_port.baudrate = 115200
        self.serial_port.timeout  = 60
        self.serial_port.open()

        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.apply_velocity, 1)
        self.cmd_vel_subscriber  # Prevent unused variable warning

        self.joints_publisher = self.create_publisher(JointState, 'joint_states', 1)

        self.odom_publisher   = self.create_publisher(Odometry, 'odom', 50)
        self.odom_broadcaster = TransformBroadcaster(self)

        self.wheel_front_left_rotation  = 0.0
        self.wheel_front_right_rotation = 0.0
        self.wheel_rear_left_rotation   = 0.0
        self.wheel_rear_right_rotation  = 0.0
        
        self.linear_x_position  = 0.0
        self.linear_y_position  = 0.0
        self.angular_z_position = 0.0

        self.read_thread = threading.Thread(target = self.read_thread_function, args = ())
        self.read_thread.start()

        return

    def apply_velocity(self, msg):

        msg.linear.x  *= X_SPEED_FACTOR_CMD_VEL
        msg.linear.y  *= Y_SPEED_FACTOR_CMD_VEL
        msg.angular.z *= Z_SPEED_FACTOR_CMD_VEL

        self.get_logger().debug('Applying velocity command @ ' + str(time.time()))

#        if self.start_time == 0.0:

#            self.start_time = time.time()

        wheel_front_left  = msg.linear.x - msg.linear.y - msg.angular.z
        wheel_front_right = msg.linear.x + msg.linear.y + msg.angular.z
        wheel_rear_left   = msg.linear.x + msg.linear.y - msg.angular.z
        wheel_rear_right  = msg.linear.x - msg.linear.y + msg.angular.z

        command_string = 'C{:.0f} {:.0f} {:.0f} {:.0f}\r'.format(wheel_front_right, wheel_front_left, wheel_rear_right, wheel_rear_left)
        command_bytes  = bytes(command_string, encoding = 'ascii')

        if command_bytes != self.last_command_bytes:
            self.get_logger().debug('Sending : ' + str(command_bytes))
            self.serial_port.write   (command_bytes)
            self.last_command_bytes = command_bytes

#        elif time.time() - self.start_time > 3.0:

#            command_string = 'C0 0 0 0\r'
#            command_bytes  = bytes(command_string, encoding = 'ascii')
#            self.serial_port.write(command_bytes)
#            self.get_logger().info('Sending : ' + str(command_bytes))

        return

    def get_rotation_in_rad(self, wheel_rotation):

        return wheel_rotation % (2 * math.pi) - math.pi

    def publish_wheels_state(self, wheel_front_left_speed, wheel_front_right_speed, wheel_rear_left_speed, wheel_rear_right_speed):

        self.wheel_front_left_rotation  += wheel_front_left_speed  / SPEED_TO_ANGLE_RATIO
        self.wheel_front_right_rotation += wheel_front_right_speed / SPEED_TO_ANGLE_RATIO
        self.wheel_rear_left_rotation   += wheel_rear_left_speed   / SPEED_TO_ANGLE_RATIO
        self.wheel_rear_right_rotation  += wheel_rear_right_speed  / SPEED_TO_ANGLE_RATIO

        joint_states = JointState()
        
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name         = ['wheel_front_left_joint' ,
                                     'wheel_front_right_joint',
                                     'wheel_back_left_joint'  ,
                                     'wheel_back_right_joint']
        joint_states.position     = [self.get_rotation_in_rad(self.wheel_front_left_rotation ),
                                     self.get_rotation_in_rad(self.wheel_front_right_rotation),
                                     self.get_rotation_in_rad(self.wheel_rear_left_rotation  ),
                                     self.get_rotation_in_rad(self.wheel_rear_right_rotation )]

        self.joints_publisher.publish(joint_states)

        return

    def publish_odom(self, wheel_front_left_speed, wheel_front_right_speed, wheel_rear_left_speed, wheel_rear_right_speed):

        linear_x_velocity  = wheel_front_left_speed + wheel_front_right_speed + wheel_rear_left_speed + wheel_rear_right_speed
        linear_y_velocity  = -wheel_front_left_speed + wheel_front_right_speed + wheel_rear_left_speed - wheel_rear_right_speed
        angular_z_velocity = -wheel_front_left_speed + wheel_front_right_speed - wheel_rear_left_speed + wheel_rear_right_speed

        linear_x_velocity  *= X_SPEED_FACTOR_ODOM
        linear_y_velocity  *= Y_SPEED_FACTOR_ODOM
        angular_z_velocity *= Z_SPEED_FACTOR_ODOM

        if (wheel_front_left_speed  != self.last_wheel_front_left_speed
         or wheel_front_right_speed != self.last_wheel_front_right_speed
         or wheel_rear_left_speed   != self.last_wheel_rear_left_speed
         or wheel_rear_right_speed  != self.last_wheel_rear_right_speed):

            self.get_logger().debug('Received: {} {} {} {}'.format(int(wheel_front_left_speed ),
                                                                   int(wheel_front_right_speed),
                                                                   int(wheel_rear_left_speed  ),
                                                                   int(wheel_rear_right_speed )))

            self.get_logger().debug('X: :{:.2f} / Y: {:.2f} / Z: {:.2f}'.format(linear_x_velocity, linear_y_velocity, angular_z_velocity))

            self.last_wheel_front_left_speed  = wheel_front_left_speed
            self.last_wheel_front_right_speed = wheel_front_right_speed
            self.last_wheel_rear_left_speed   = wheel_rear_left_speed
            self.last_wheel_rear_right_speed  = wheel_rear_right_speed

        current_time_rx_in_s    = time.time()
        delta_time_in_s         = current_time_rx_in_s - self.saved_time_rx_in_s
        self.saved_time_rx_in_s = current_time_rx_in_s

        delta_x = delta_time_in_s * (linear_x_velocity * math.cos(self.angular_z_position) - linear_y_velocity * math.sin(self.angular_z_position))
        delta_y = delta_time_in_s * (linear_x_velocity * math.sin(self.angular_z_position) + linear_y_velocity * math.cos(self.angular_z_position))
        delta_z = delta_time_in_s *  angular_z_velocity

        self.linear_x_position  += delta_x
        self.linear_y_position  += delta_y
        self.angular_z_position += delta_z

        tf_quat  = tf_transformations.quaternion_from_euler(0, 0, self.angular_z_position)
        msg_quat = Quaternion(x=tf_quat[0], y=tf_quat[1], z=tf_quat[2], w=tf_quat[3])

        # Publish odom transform only when localization node is not used.
        # When localization node is running, it publishes that transform.

        # odom_transform                 = TransformStamped()
        # odom_transform.header.frame_id = 'odom'
        # odom_transform.child_frame_id  = 'base_link'
        # odom_transform.header.stamp    = self.get_clock().now().to_msg()

        # odom_transform.transform.translation.x = self.linear_x_position
        # odom_transform.transform.translation.y = self.linear_y_position
        # odom_transform.transform.translation.z = 0.0
        # odom_transform.transform.rotation      = msg_quat

        # self.get_logger().debug('Sending transform')
        # self.odom_broadcaster.sendTransform(odom_transform)

        odometry                 = Odometry()
        odometry.header.frame_id = "odom"
        odometry.child_frame_id  = "base_link"
        odometry.header.stamp    = self.get_clock().now().to_msg()

        odometry.pose.pose.position.x  = self.linear_x_position
        odometry.pose.pose.position.y  = self.linear_y_position
        odometry.pose.pose.position.z  = 0.0
        odometry.pose.pose.orientation = msg_quat

        odometry.twist.twist.linear.x  = linear_x_velocity
        odometry.twist.twist.linear.y  = linear_y_velocity
        odometry.twist.twist.angular.z = angular_z_velocity

        self.get_logger().debug('Publishing odometry')

        self.odom_publisher.publish(odometry)

    def read_thread_function(self):

        self.get_logger().info('Starting reading thread');

        char = None
        msg  = ''

        while True:

            char = self.serial_port.read(1)

            if char == b'\r':
                split_msg = msg[1:].split()
                if msg[0] == 'S' and len(split_msg) == 4:
                    self.publish_wheels_state(int(split_msg[1]), int(split_msg[0]), int(split_msg[3]), int(split_msg[2]))
                    self.publish_odom        (int(split_msg[1]), int(split_msg[0]), int(split_msg[3]), int(split_msg[2]))
                    pass
                else:
                    self.get_logger().info('Discarding malformed message')
                msg = ''
            elif char == b'\n':
                pass
            else:

                msg += char.decode('utf-8', 'ignore')

        return

def main(args=None):

    rclpy.init(args=args)

    drive_controller = DriveController()

    try:
         rclpy.spin(drive_controller)
    except KeyboardInterrupt:
         print('Stopped by keyboard interrupt')
    except BaseException:
         print('Stopped by exception')
         raise
    finally:
         drive_controller.destroy_node()
         rclpy.shutdown() 

if __name__ == '__main__':
    main()


