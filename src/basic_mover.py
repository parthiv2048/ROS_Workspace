#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion

# BasicMover
class BasicMover:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        # Current heading of the robot.
        self.x = None
        self.y = None
        self.cur_yaw = None

    def my_odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        self.x = msg.x
        self.y = msg.y
        self.cur_yaw = msg.z

    def turn_to_heading(self, target_yaw):
        """
        Turns the robot to heading `target_yaw`.
        """
        while self.cur_yaw is None:
            pass
        
        twist = Twist()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            cur_yaw_deg = self.cur_yaw * 180 / math.pi
            angular_vel = (target_yaw - cur_yaw_deg) % 360
            if angular_vel > 180:
                angular_vel -= 360
            
            if angular_vel > 0:
                stop_at_pos = False
            elif angular_vel < 0:
                stop_at_pos = True

            if ((abs(angular_vel) < 0.5)
                    or (angular_vel > 0 and stop_at_pos)
                    or (angular_vel < 0 and not(stop_at_pos))):
                twist.angular.z = 0
                self.cmd_vel_pub.publish(twist)
                break
            
            twist.angular.z = 0.5 * (angular_vel * math.pi / 180)
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
    def move_forward(self, target_dist):
        """Moves the robot forward by `target_dist`."""

        while self.x is None:
            pass
        
        while self.y is None:
            pass

        twist = Twist()
        rate = rospy.Rate(10)
        init_x = self.x
        init_y = self.y
        max_speed = 0.3
        total_time = 2 * target_dist / max_speed
        start_time = rospy.get_time()

        while rospy.get_time() == 0:
            start_time = rospy.get_time()

        while not rospy.is_shutdown():
            dist_trav = math.hypot(self.x - init_x, self.y - init_y)
            time_passed = rospy.get_time() - start_time

            if time_passed < (total_time / 2.0):
                twist.linear.x = (time_passed / (total_time / 2.0)) * max_speed
            elif time_passed < total_time:
                twist.linear.x = (2 - (time_passed / (total_time / 2.0))) * max_speed
            
            if dist_trav >= target_dist:
                twist.linear.x = 0
                self.cmd_vel_pub.publish(twist)
                break
            
            self.cmd_vel_pub.publish(twist)
            
            rate.sleep()

    def out_and_back(self, target_dist):
        """
        This function:
        1. moves the robot forward by `target_dist`;
        2. turns the robot by 180 degrees; and
        3. moves the robot forward by `target_dist`.
        """
        self.move_forward(target_dist)
        self.turn_to_heading(180)
        self.move_forward(target_dist)

    def draw_square(self, side_length):
        """
        This function moves the robot in a square with `side_length` meter sides.
        """

        while self.cur_yaw is None:
            pass
        
        for i in range(3):
            self.move_forward(side_length)
            self.turn_to_heading((self.cur_yaw * 180 / math.pi) + 90)
        self.move_forward(side_length)

    def move_in_a_circle(self, r):
        """Moves the robot in a circle with radius `r`"""

        while self.x is None:
            pass
        
        while self.y is None:
            pass

        twist = Twist()
        rate = rospy.Rate(10)
        init_x = self.x
        init_y = self.y
        max_speed = 0.3
        total_time = 4 * math.pi * r / max_speed
        start_time = rospy.get_time()

        while rospy.get_time() == 0:
            start_time = rospy.get_time()

        while not rospy.is_shutdown():
            dist_trav = math.hypot(self.x - init_x, self.y - init_y)
            time_passed = rospy.get_time() - start_time
            if time_passed < (total_time / 2.0):
                twist.linear.x = (time_passed / (total_time / 2.0)) * max_speed
                twist.angular.z = (time_passed / (total_time / 2.0)) * max_speed / r
            elif time_passed < total_time:
                twist.linear.x = (2 - (time_passed / (total_time / 2.0))) * max_speed
                twist.angular.z = (2 - (time_passed / (total_time / 2.0))) * max_speed / r
            
            if time_passed > (total_time / 2) and dist_trav < 0.01:
                twist.linear.x = 0
                twist.angular.z = 0
                self.cmd_vel_pub.publish(twist)
                break
            
            self.cmd_vel_pub.publish(twist)
            
            rate.sleep()
        
    def rotate_in_place(self):
        """For debugging."""
        twist = Twist()
        
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            twist.angular.z = 0.1
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('basic_mover')
    BasicMover().out_and_back(1)
    BasicMover().draw_square(1)
    BasicMover().move_in_a_circle(1)
    # BasicMover().rotate_in_place()
