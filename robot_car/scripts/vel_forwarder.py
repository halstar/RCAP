#!/usr/bin/env python3

import rclpy
import threading
import serial

from rclpy.node        import Node
from geometry_msgs.msg import Twist

WHEEL_RADIUS            = 0.040
WHEEL_SEPARATION_WIDTH  = 0.195
WHEEL_SEPARATION_LENGTH = 0.175

class VelocityForwarder(Node):

    def __init__(self):
        super().__init__('vel_forwarder')

        self.get_logger().info('Starting Velocity Forwarder Node');


        self.serial_port          = serial.Serial()
        self.serial_port.port     = '/dev/serial0'
        self.serial_port.baudrate = 115200
        self.serial_port.timeout  = 60
        self.serial_port.open()


        self.read_thread = threading.Thread(target = self.read_thread_function, args = ())
        self.read_thread.start()

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.handle_message,
            1)
        self.subscription  # prevent unused variable warning

    def handle_message(self, msg):

        wheel_front_left  = (1 / WHEEL_RADIUS) * (msg.linear.x - msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z)
        wheel_front_right = (1 / WHEEL_RADIUS) * (msg.linear.x + msg.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z)
        wheel_rear_left   = (1 / WHEEL_RADIUS) * (msg.linear.x + msg.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z)
        wheel_rear_right  = (1 / WHEEL_RADIUS) * (msg.linear.x - msg.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * msg.angular.z)

        self.get_logger().info('Sending  - F.L.: {:.2f} - F.R.: {:.2f} - R.L.: {:.2f} - R.R.: {:.2f}'.format(wheel_front_left, wheel_front_right, wheel_rear_left, wheel_rear_right))

        self.serial_port.write(b'CU {:.1f}  {:.1f} {:.1f}  {:.1f}\r'.format(wheel_front_left, wheel_front_right, wheel_rear_left, wheel_rear_right))


    def read_thread_function():

      char = None
      msg  = ''

      while True:

        char = self.serial_port.read(1)

        if char == b'\r':
          self.get_logger().info('Received - ' + msg)
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