#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class VelocityForwarder(Node):

    def __init__(self):
        super().__init__('vel_forwarder')

        self.get_logger().info("Starting Velocity Forwarder Node");

        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    vel_forwarder = VelocityForwarder()

    rclpy.spin(vel_forwarder)

    vel_forwarder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()