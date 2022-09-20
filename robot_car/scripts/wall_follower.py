#!/usr/bin/env python3

import rclpy
import time

from rclpy.node        import Node
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist

PROCESS_PERIOD = 0.010
LINEAR_SPEED   = 0.200
TURN_SPEED     = 1.000


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        self.get_logger().info('Starting Wall Follower Node');

        self.cycle = 0
        self.timer = self.create_timer(PROCESS_PERIOD, self.main_callback)

        self.qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                               history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                               depth=10)

        self.laser_scan_subscriber = self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=self.qos_policy)
        self.laser_scan_subscriber  # Prevent unused variable warning

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        return

    def main_callback(self):

        # self.get_logger().info('Entering main');

        # if self.cycle == 0:

        #     self.stop()

        # elif self.cycle == 100:

        #     self.go_forward()

        # elif self.cycle == 300:

        #     self.turn_left()

        # elif self.cycle == 500:

        #     self.go_forward()

        # elif self.cycle == 700:

        #     self.go_backward()

        # elif self.cycle == 900:

        #     self.turn_right()

        # elif self.cycle == 1100:

        #     self.stop()

        # else:

        #     pass

        self.cycle += 1

        return

    def process_scan(self, msg):

        self.get_logger().info('Got a new scan: ' + str(msg))

        return

    def go_forward(self):

        twist = Twist()

        twist.linear.x = LINEAR_SPEED
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist)

        self.get_logger().info('Going forward')

    def stop(self):

        twist = Twist()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist)

        self.get_logger().info('Stopping')

    def go_backward(self):

        twist = Twist()

        twist.linear.x = -LINEAR_SPEED
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist)

        self.get_logger().info('Going backward')

    def turn_left(self):

        twist = Twist()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = TURN_SPEED

        self.cmd_vel_publisher.publish(twist)

        self.get_logger().info('Turning left')

    def turn_right(self):

        twist = Twist()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -TURN_SPEED

        self.cmd_vel_publisher.publish(twist)

        self.get_logger().info('Turning right')


def main(args=None):

    rclpy.init(args=args)

    wall_follower = WallFollower()

    try:
         rclpy.spin(wall_follower)
    except KeyboardInterrupt:
         print('Stopped by keyboard interrupt')
    except BaseException:
         print('Stopped by exception')
         raise
    finally:
         wall_follower.destroy_node()
         rclpy.shutdown() 

if __name__ == '__main__':
    main()


