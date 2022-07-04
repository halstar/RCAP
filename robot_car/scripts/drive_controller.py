#!/usr/bin/env python3

import rclpy
import threading
import serial
import time
import math

from rclpy.node        import Node
from sensor_msgs.msg   import JointState
from geometry_msgs.msg import Twist


WHEEL_RADIUS            = 0.040
WHEEL_SEPARATION_WIDTH  = 0.195
WHEEL_SEPARATION_LENGTH = 0.175
SPEED_TO_ANGLE_RATIO    = 15.50


class DriveController(Node):

    def __init__(self):
        super().__init__('drive_controller')

        self.get_logger().info('Starting Drive Controller Node');

        self.saved_time_tx_in_ms = time.time() * 1000
        self.saved_time_rx_in_s  = time.time()

        self.serial_port          = serial.Serial()
        self.serial_port.port     = '/dev/serial0'
        self.serial_port.baudrate = 115200
        self.serial_port.timeout  = 60
        self.serial_port.open()

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.handle_message, 1)
        self.subscription  # Prevent unused variable warning

        self.publisher = self.create_publisher(JointState, 'joint_states', 1)

        self.odom_publisher   = self.create_publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

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

        current_time_in_ms = time.time() * 1000

        if current_time_in_ms - self.saved_time_tx_in_ms > 250:

            self.saved_time_tx_in_ms = current_time_in_ms

            wheel_front_left  = (1 / WHEEL_RADIUS) * (msg.linear.x - msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z) 
            wheel_front_right = (1 / WHEEL_RADIUS) * (msg.linear.x + msg.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z)
            wheel_rear_left   = (1 / WHEEL_RADIUS) * (msg.linear.x + msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z)                
            wheel_rear_right  = (1 / WHEEL_RADIUS) * (msg.linear.x - msg.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z)

            command_string = 'C{:.0f} {:.0f} {:.0f} {:.0f}\r'.format(wheel_front_right, wheel_front_left, wheel_rear_right, wheel_rear_left)
            command_bytes  = bytes(command_string, encoding = 'ascii')
    
            self.serial_port.write(command_bytes)

            self.get_logger().info('Sending : ' + str(command_bytes))

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

        linear_x_velocity  = ( wheel_front_left_speed + wheel_front_right_speed + wheel_rear_left_speed + wheel_rear_right_speed) * (WHEEL_RADIUS / 4)
        linear_y_velocity  = (-wheel_front_left_speed + wheel_front_right_speed + wheel_rear_left_speed - wheel_rear_right_speed) * (WHEEL_RADIUS / 4)
        angular_z_velocity = (-wheel_front_left_speed + wheel_front_right_speed - wheel_rear_left_speed + wheel_rear_right_speed) * (WHEEL_RADIUS / (4 * (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)))

        current_time_rx_in_s = time.time()
        delta_time_in_s      = current_time_rx_in_s - self.saved_time_rx_in_ms

        delta_x = (linear_x_velocity  * cos(angular_z_velocity) - linear_y_velocity * sin(angular_z_velocity)) * delta_time_in_s
        delta_y = (linear_x_velocity  * sin(angular_z_velocity) + linear_y_velocity * cos(angular_z_velocity)) * delta_time_in_s
        delta_z = angular_z_velocity * delta_time_in_s

        self.linear_x_position  += delta_x
        self.linear_y_position  += delta_y
        self.angular_z_position += delta_z

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, angular_z_velocity)

        self.odom_broadcaster.sendTransform((self.linear_x_position, self.linear_y_position, 0.),
                                            odom_quat,
                                            current_time_rx_in_s,
                                            "base_link",
                                            "odom")

        odometry = Odometry()

        odometry.header.frame_id = "odom"
        odometry.header.stamp    = self.get_clock().now().to_msg()
        odometry.pose.pose       = Pose(Point(self.linear_x_position, self.linear_y_position, 0.), Quaternion(*odom_quat))

        odometry.child_frame_id  = "base_link"
        odometry.twist.twist     = Twist(Vector3(linear_x_velocity, linear_y_velocity, 0), Vector3(0, 0, angular_z_velocity))

        self.odom_publisher.publish(odometry)

    def read_thread_function(self):

        self.get_logger().info('Starting reading thread');

        char = None
        msg  = ''

        while True:

            char = self.serial_port.read(1)

            if char == b'\r':
                self.get_logger().info('Received: ' + msg)                
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

