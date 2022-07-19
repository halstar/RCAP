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

WHEEL_RADIUS            = 0.040
WHEEL_SEPARATION_WIDTH  = 0.195
WHEEL_SEPARATION_LENGTH = 0.175
SPEED_TO_ANGLE_RATIO    = 15.50
SPEED_TO_ODOM_RATIO     =  0.45

X_SPEED_FACTOR = 2.10
Y_SPEED_FACTOR = 3.00
Z_SPEED_FACTOR = 1.15

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

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.handle_message, 1)
        self.subscription  # Prevent unused variable warning

        self.publisher = self.create_publisher(JointState, 'joint_states', 1)

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

    def handle_message(self, msg):

        msg.linear.x  *= X_SPEED_FACTOR
        msg.linear.y  *= Y_SPEED_FACTOR
        msg.angular.z *= Z_SPEED_FACTOR

        self.get_logger().debug('Handling message @ ' + str(time.time()))

#        if self.start_time == 0.0:

#            self.start_time = time.time()

        wheel_front_left  = (1 / WHEEL_RADIUS) * (msg.linear.x - msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z)
        wheel_front_right = (1 / WHEEL_RADIUS) * (msg.linear.x + msg.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z)
        wheel_rear_left   = (1 / WHEEL_RADIUS) * (msg.linear.x + msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z)
        wheel_rear_right  = (1 / WHEEL_RADIUS) * (msg.linear.x - msg.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z)

        command_string = 'C{:.0f} {:.0f} {:.0f} {:.0f}\r'.format(wheel_front_right, wheel_front_left, wheel_rear_right, wheel_rear_left)

        command_bytes  = bytes(command_string, encoding = 'ascii')
        self.serial_port.write(command_bytes)

        if command_bytes != self.last_command_bytes:
            self.get_logger().info('Sending : ' + str(command_bytes))
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

        self.publisher.publish(joint_states)

        return

    def publish_odom(self, wheel_front_left_speed, wheel_front_right_speed, wheel_rear_left_speed, wheel_rear_right_speed):

        wheel_front_left_speed  *= SPEED_TO_ODOM_RATIO
        wheel_front_right_speed *= SPEED_TO_ODOM_RATIO
        wheel_rear_left_speed   *= SPEED_TO_ODOM_RATIO
        wheel_rear_right_speed  *= SPEED_TO_ODOM_RATIO

        if (wheel_front_left_speed  != self.last_wheel_front_left_speed  
        or wheel_front_right_speed != self.last_wheel_front_right_speed
        or wheel_rear_left_speed   != self.last_wheel_rear_left_speed  
        or wheel_rear_right_speed  != self.last_wheel_rear_right_speed):

            self.get_logger().info('Received: {} {} {} {}'.format(int(wheel_front_left_speed ),
                                                                  int(wheel_front_right_speed),
                                                                  int(wheel_rear_left_speed  ),
                                                                  int(wheel_rear_right_speed )))

            self.last_wheel_front_left_speed  = wheel_front_left_speed
            self.last_wheel_front_right_speed = wheel_front_right_speed
            self.last_wheel_rear_left_speed   = wheel_rear_left_speed
            self.last_wheel_rear_right_speed  = wheel_rear_right_speed


        linear_x_velocity  = ( wheel_front_left_speed + wheel_front_right_speed + wheel_rear_left_speed + wheel_rear_right_speed) * (WHEEL_RADIUS / 4)
        linear_y_velocity  = (-wheel_front_left_speed + wheel_front_right_speed + wheel_rear_left_speed - wheel_rear_right_speed) * (WHEEL_RADIUS / 4)
        angular_z_velocity = (-wheel_front_left_speed + wheel_front_right_speed - wheel_rear_left_speed + wheel_rear_right_speed) * (WHEEL_RADIUS / (4 * (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)))

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

        odom_transform                 = TransformStamped()
        odom_transform.header.frame_id = 'odom'
        odom_transform.child_frame_id  = 'base_link'
        odom_transform.header.stamp    = self.get_clock().now().to_msg()

        odom_transform.transform.translation.x = self.linear_x_position
        odom_transform.transform.translation.y = self.linear_y_position
        odom_transform.transform.translation.z = 0.0
        odom_transform.transform.rotation      = msg_quat

        self.get_logger().debug('Sending transform')

        self.odom_broadcaster.sendTransform(odom_transform)

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

