#!/usr/bin/env python3

import rclpy
import threading
import serial
import time

from rclpy.node        import Node
from geometry_msgs.msg import Twist

WHEEL_RADIUS            = 0.040
WHEEL_SEPARATION_WIDTH  = 0.195
WHEEL_SEPARATION_LENGTH = 0.175

class VelocityForwarder(Node):

    def __init__(self):
        super().__init__('vel_forwarder')

        self.get_logger().info('Starting Velocity Forwarder Node');

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

    def handle_message(self, msg):

        current_time_in_ms   = time.time() * 1000

        if current_time_in_ms - self.saved_time_in_ms > 250:

            self.saved_time_in_ms = current_time_in_ms

            wheel_front_left  = (1 / WHEEL_RADIUS) * (msg.linear.x - msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z) * 2
            wheel_front_right = (1 / WHEEL_RADIUS) * (msg.linear.x + msg.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z) * 2
            wheel_rear_left   = (1 / WHEEL_RADIUS) * (msg.linear.x + msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z) * 2                
            wheel_rear_right  = (1 / WHEEL_RADIUS) * (msg.linear.x - msg.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z) * 2

            command_string = 'C{:.0f} {:.0f} {:.0f} {:.0f}\r'.format(wheel_front_right, wheel_front_left, wheel_rear_right, wheel_rear_left)
            command_bytes  = bytes(command_string, encoding = 'ascii')
    
            self.serial_port.write(command_bytes)

            self.get_logger().info('Sending : ' + str(command_bytes))


    def read_thread_function(self):

      self.get_logger().info('Starting reading thread');

      char = None
      msg  = ''

      while True:

        char = self.serial_port.read(1)

        if char == b'\r':
            self.get_logger().info('Received: ' + msg)
            msg = ''
        elif char == b'\n':
            pass
        else:
            msg += char.decode('utf-8', 'ignore')

      return

def main(args=None):

    rclpy.init(args=args)

    vel_forwarder = VelocityForwarder()

    rclpy.spin(vel_forwarder)

    vel_forwarder.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
