#!/usr/bin/env python3

import rclpy
import threading
import serial
import time
import math
import tf_transformations

from rclpy.node        import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from tf2_ros           import TransformBroadcaster


WHEEL_RADIUS            = 0.040
WHEEL_SEPARATION_WIDTH  = 0.195
WHEEL_SEPARATION_LENGTH = 0.175
SPEED_TO_ANGLE_RATIO    = 15.50


class DriveController(Node):

    def __init__(self):
        super().__init__('drive_controller')

        self.get_logger().info('Starting Drive Controller Node');

        self.saved_time_in_ms = time.time() * 1000

        self.serial_port          = serial.Serial()
        self.serial_port.port     = '/dev/serial0'
        self.serial_port.baudrate = 115200
        self.serial_port.timeout  = 60
        self.serial_port.open()

        self.read_thread = threading.Thread(target = self.read_thread_function, args = ())
        self.read_thread.start()

        self.get_logger().info('Starting writing thread');

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.handle_message,
            1)
        self.subscription  # prevent unused variable warning

        self.broadcaster  = TransformBroadcaster(self)

        return

    def handle_message(self, msg):

        current_time_in_ms   = time.time() * 1000

        if current_time_in_ms - self.saved_time_in_ms > 250:

            self.saved_time_in_ms = current_time_in_ms

            wheel_front_left  = (1 / WHEEL_RADIUS) * (msg.linear.x - msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z) 
            wheel_front_right = (1 / WHEEL_RADIUS) * (msg.linear.x + msg.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z)
            wheel_rear_left   = (1 / WHEEL_RADIUS) * (msg.linear.x + msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z)                
            wheel_rear_right  = (1 / WHEEL_RADIUS) * (msg.linear.x - msg.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z)

            command_string = 'C{:.0f} {:.0f} {:.0f} {:.0f}\r'.format(wheel_front_right, wheel_front_left, wheel_rear_right, wheel_rear_left)
            command_bytes  = bytes(command_string, encoding = 'ascii')
    
            self.serial_port.write(command_bytes)

            self.get_logger().info('Sending : ' + str(command_bytes))

            self.wheel_front_left_rotation  = 0.0
            self.wheel_front_right_rotation = 0.0
            self.wwheel_rear_left_rotation  = 0.0
            self.wwheel_rear_right_rotation = 0.0

        return


    def broadcast_wheels_tf(self, wheel_front_left_speed, wheel_front_right_speed, wheel_rear_left_speed, wheel_rear_right_speed):

        transform_stamped = TransformStamped()

        # Common parameters
        transform_stamped.header.stamp    = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'base_link'

        transform_stamped.transform.translation.x = 0.0
        transform_stamped.transform.translation.y = 0.0
        transform_stamped.transform.translation.z = 0.0

        transform_stamped.transform.rotation.x = 0.0
        transform_stamped.transform.rotation.y = 0.0

        # Specific parameters
        self.wheel_front_left_rotation        += wheel_front_left_speed / SPEED_TO_ANGLE_RATIO
        transform_stamped.child_frame_id       = 'wheel_front_left_link'        
        transform_stamped.transform.rotation.z = self.wheel_front_left_rotation % math.pi
        self.broadcaster.sendTransform(transform_stamped)

        self.get_logger().info('Broadcasting: ' + str(transform_stamped.transform.rotation.z))


        self.wheel_front_right_rotation       += wheel_front_right_speed / SPEED_TO_ANGLE_RATIO
        transform_stamped.child_frame_id       = 'wheel_front_right_link'
        transform_stamped.transform.rotation.z = self.wheel_front_right_rotation % math.pi
        self.broadcaster.sendTransform(transform_stamped)

        self.wwheel_rear_left_rotation        += wheel_rear_left_speed / SPEED_TO_ANGLE_RATIO
        transform_stamped.child_frame_id       = 'wheel_rear_left_link'
        transform_stamped.transform.rotation.z = self.wwheel_rear_left_rotation % math.pi
        self.broadcaster.sendTransform(transform_stamped)

        self.wwheel_rear_right_rotation       += wheel_rear_right_speed / SPEED_TO_ANGLE_RATIO
        transform_stamped.child_frame_id       = 'wheel_rear_right_link'
        transform_stamped.transform.rotation.z = self.wwheel_rear_right_rotation % math.pi
        self.broadcaster.sendTransform(transform_stamped)

        return

    def read_thread_function(self):

        self.get_logger().info('Starting reading thread');

        char = None
        msg  = ''

        while True:

            char = self.serial_port.read(1)

        if char == b'\r':
            self.get_logger().info('Received: ' + msg)
            split_msg = msg.split()
            broadcast_wheels_tf(int(split_msg[0][1:]), int(split_msg[1]), int(split_msg[2]), int(split_msg[3]))
            msg = ''
        elif char == b'\n':
            pass
        else:
            msg += char.decode('utf-8', 'ignore')

        return

def main(args=None):

    rclpy.init(args=args)

    drive_controller = DriveController()

    rclpy.spin(drive_controller)

    drive_controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
