#!/usr/bin/env python3

import rclpy
import time

from rclpy.node        import Node
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist

PROCESS_PERIOD    = 0.01
LINEAR_SPEED      = 0.20
TURN_SPEED        = 1.00
LOW_SPEED_FACTOR  = 1.00
HIGH_SPEED_FACTOR = 2.00

STATE_FIND_A_WALL = 0
STATE_FOLLOW_WALL = 1

ACTION_STOP             = 0
ACTION_GO_FORWARD       = 1
ACTION_GO_BACKWARD      = 2
ACTION_TURN_LEFT        = 3
ACTION_TURN_RIGHT       = 4
ACTION_GO_FORWARD_LEFT  = 5
ACTION_GO_FORWARD_RIGHT = 6

SIDE_FOLLOW_DISTANCE    = 0.20
SIDE_FOLLOW_TOLERANCE   = 0.02
GET_AROUND_DISTANCE     = 0.40
FRONT_OBSTACLE_DISTANCE = 0.40


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        self.get_logger().info('>>> Starting Wall Follower Node');

        self.cycle = 0
        self.timer = self.create_timer(PROCESS_PERIOD, self.main_callback)

        self.qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                               history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                               depth=10)

        self.laser_scan_subscriber = self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=self.qos_policy)
        self.laser_scan_subscriber  # Prevent unused variable warning

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.previous_state      = -1
        self.current_state       = STATE_FIND_A_WALL
        self.previous_action     = -1
        self.current_action      = ACTION_STOP
        self.obstacles           = {}
        self.obstacle_is_on_left = False
        self.speed_factor        = LOW_SPEED_FACTOR

        return

    def main_callback(self):

        if self.current_action != self.previous_action:

            if self.current_action == ACTION_STOP:

                self.stop()

            elif self.current_action == ACTION_GO_FORWARD:

                self.go_forward()

            elif self.current_action == ACTION_GO_BACKWARD:

                self.go_backward()

            elif self.current_action == ACTION_TURN_LEFT:

                self.turn_left()

            elif self.current_action == ACTION_TURN_RIGHT:

                self.turn_right()

            elif self.current_action == ACTION_GO_FORWARD_LEFT:

                self.go_forward_left()

            elif self.current_action == ACTION_GO_FORWARD_RIGHT:

                self.go_forward_right()

            elif self.current_action == ACTION_GO_BACKWARD:

                self.go_backward()

            else:

                self.stop()
                self.get_logger().error('Got an unreachable action');

            self.previous_action = self.current_action


        self.cycle += 1

        return

    def process_scan(self, msg):

        # self.get_logger().info('Got a new scan: ' + str(msg))

        self.obstacle_distance = \
        {
            'front'      : min(msg.ranges[337:359] + msg.ranges[0:21]),
            'front_left' : min(msg.ranges[ 22: 66]),
            'left'       : min(msg.ranges[ 67:111]),
            'back_left'  : min(msg.ranges[112:156]),
            'back'       : min(msg.ranges[157:201]),
            'back_right' : min(msg.ranges[202:246]),
            'right'      : min(msg.ranges[247:291]),
            'front_right': min(msg.ranges[292:336]),
        }

        self.get_logger().info(str(self.obstacle_distance ))

        

        if self.current_state == STATE_FIND_A_WALL:

            if self.current_state != self.previous_state:

                self.get_logger().info('>>> Entering STATE_FIND_A_WALL')

                self.speed_factor   = HIGH_SPEED_FACTOR
                self.previous_state = self.current_state

            if self.obstacle_distance['left'] < SIDE_FOLLOW_DISTANCE + SIDE_FOLLOW_TOLERANCE:

                self.obstacle_is_on_left = True

                self.current_state = STATE_FOLLOW_WALL

                self.get_logger().info('>>> Found a wall on the left')

            elif self.obstacle_distance['right'] < SIDE_FOLLOW_DISTANCE + SIDE_FOLLOW_TOLERANCE:

                self.obstacle_is_on_right = True

                self.current_state = STATE_FOLLOW_WALL

                self.get_logger().info('>>> Found a wall on the right')

            else:

                self.current_action = ACTION_GO_FORWARD_RIGHT

        elif self.current_state == STATE_FOLLOW_WALL:

            if self.current_state != self.previous_state:

                self.get_logger().info('>>> Entering STATE_FOLLOW_WALL')

                self.speed_factor   = LOW_SPEED_FACTOR
                self.previous_state = self.current_state

            if self.obstacle_distance['front'] < FRONT_OBSTACLE_DISTANCE / 2:

                self.current_action = ACTION_GO_BACKWARD

            elif self.obstacle_is_on_left == True:

                if (self.obstacle_distance['front'] < FRONT_OBSTACLE_DISTANCE) or (self.obstacle_distance['front_left'] < FRONT_OBSTACLE_DISTANCE):

                    self.current_action = ACTION_GO_FORWARD_RIGHT

                elif self.obstacle_distance['front_left'] > GET_AROUND_DISTANCE:

                    self.current_action = ACTION_GO_FORWARD_LEFT

                elif self.obstacle_distance['left'] < SIDE_FOLLOW_DISTANCE - SIDE_FOLLOW_TOLERANCE:

                    self.current_action = ACTION_GO_FORWARD_RIGHT

                elif self.obstacle_distance['left'] > SIDE_FOLLOW_DISTANCE + SIDE_FOLLOW_TOLERANCE:

                    self.current_action = ACTION_GO_FORWARD_LEFT

                else:

                    self.current_action = ACTION_GO_FORWARD

            else:

                if (self.obstacle_distance['front'] < FRONT_OBSTACLE_DISTANCE) or (self.obstacle_distance['front_right'] < FRONT_OBSTACLE_DISTANCE):

                    self.current_action = ACTION_GO_FORWARD_LEFT

                elif self.obstacle_distance['front_right'] > GET_AROUND_DISTANCE:

                    self.current_action = ACTION_GO_FORWARD_RIGHT

                elif self.obstacle_distance['right'] < SIDE_FOLLOW_DISTANCE - SIDE_FOLLOW_TOLERANCE:

                    self.current_action = ACTION_GO_FORWARD_LEFT

                elif self.obstacle_distance['left'] > SIDE_FOLLOW_DISTANCE + SIDE_FOLLOW_TOLERANCE:

                    self.current_action = ACTION_GO_FORWARD_RIGHT

                else:

                    self.current_action = ACTION_GO_FORWARD

        else:

            self.current_action = ACTION_STOP
            self.get_logger().error('Reached an unreachable state');

        return

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

    def go_forward(self):

        twist = Twist()

        twist.linear.x = LINEAR_SPEED * self.speed_factor
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist)

        self.get_logger().info('Going forward')


    def go_forward_left(self):

        twist = Twist()

        twist.linear.x = LINEAR_SPEED * self.speed_factor
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = TURN_SPEED * self.speed_factor

        self.cmd_vel_publisher.publish(twist)

        self.get_logger().info('Going forward left')

    def go_forward_right(self):

        twist = Twist()

        twist.linear.x = LINEAR_SPEED * self.speed_factor
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -TURN_SPEED * self.speed_factor

        self.cmd_vel_publisher.publish(twist)

        self.get_logger().info('Going forward right')

    def go_backward(self):

        twist = Twist()

        twist.linear.x = -LINEAR_SPEED * self.speed_factor
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
        twist.angular.z = TURN_SPEED * self.speed_factor

        self.cmd_vel_publisher.publish(twist)

        self.get_logger().info('Turning left')

    def turn_right(self):

        twist = Twist()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -TURN_SPEED * self.speed_factor

        self.cmd_vel_publisher.publish(twist)

        self.get_logger().info('Turning right')


    def find_a_wall(self):

        pass

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


