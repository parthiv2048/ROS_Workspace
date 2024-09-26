#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

MAX_SPEED = 0.3
DIST_FROM_WALL = 1

class WallFollower:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.v = 0
        self.w = 0
        # Proportional constant for distance error
        self.kp = 2
        # Derivative constant for distance error
        self.kd = 0.5
        # Proportional constant for angular error
        self.pa = 1
        self.prev_dist_error = 0
    
    def filter_range(self, ranges, range_min, range_max):
        filtered_range = [r for r in ranges if not(math.isnan(r)) and not(math.isinf(r)) and r > range_min and r < range_max]
        if len(filtered_range) == 0:
            return [0]
        return filtered_range

    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        """
        # By default drive at MAX_SPEED
        self.v = MAX_SPEED
        # Get all distances between -10 degrees and 10 degrees
        front_distances = [r for r in (msg.ranges[:10] + msg.ranges[350:]) if not(math.isnan(r)) and not(math.isinf(r)) and r > msg.range_min and r < msg.range_max]
        if len(front_distances) != 0:
            # Calculate average front distance
            front_dist = sum(front_distances) / len(front_distances)
            # Stop if very close to a wall
            if front_dist < 0.1 * DIST_FROM_WALL:
                self.v = 0
            # Slow down if close to wall
            elif front_dist < DIST_FROM_WALL:
                self.v = 0.6 * MAX_SPEED

        # Scan all distances on the right side of the robot
        shortest_dist_angle = 180
        for angle in range(180, 360):
            current_dist = msg.ranges[angle]
            if math.isnan(current_dist) or math.isinf(current_dist) or current_dist < msg.range_min or current_dist > msg.range_max:
                continue
            # Find the angle with the lowest distance
            if current_dist < msg.ranges[shortest_dist_angle]:
                shortest_dist_angle = angle
        # Find the lowest distance
        shortest_dist = msg.ranges[shortest_dist_angle]
        # Convert angle to radians
        shortest_dist_angle_rad = shortest_dist_angle * math.pi / 180
        # Shift angle by -2pi because ROS uses angles from pi to -pi
        shortest_dist_angle_rad -= 2 * math.pi
        print("shortest_dist_angle_rad:", shortest_dist_angle_rad, "shortest_dist:", shortest_dist)
        # Calculate distance error, derivative of distance error, and angular error needed for PD controller
        dist_error = DIST_FROM_WALL - shortest_dist
        dist_derivative = dist_error - self.prev_dist_error
        angular_error = shortest_dist_angle_rad - ((-math.pi)/2)
        self.prev_dist_error = dist_error
        # PD controller using both distance and angle error
        self.w = self.kp * dist_error + self.kd * dist_derivative + self.pa * angular_error

    def follow_wall(self):
        """
        Makes the robot follow a wall.
        """
        twist = Twist()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            twist.angular.z = self.w
            twist.linear.x = self.v
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower().follow_wall()