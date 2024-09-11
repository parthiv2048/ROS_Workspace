#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion

MAX_SPEED = 0.15

# BasicMover
class Solution:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        # self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        # # Current heading of the robot.
        # self.cur_yaw = None

    def my_odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        raise NotImplementedError

    def turn_to_heading(self, target_yaw):
        """
        Turns the robot to heading `target_yaw`.
        """
        raise NotImplementedError
        
    def move_forward(self, target_dist):
        """Moves the robot forward by `target_dist`."""
        raise NotImplementedError

    def out_and_back(self, target_dist):
        """
        This function:
        1. moves the robot forward by `target_dist`;
        2. turns the robot by 180 degrees; and
        3. moves the robot forward by `target_dist`.
        """
        raise NotImplementedError

    def draw_square(self, side_length):
        """
        This function moves the robot in a square with `side_length` meter sides.
        """ 
        raise NotImplementedError

    def move_in_a_circle(self, r):
        """Moves the robot in a circle with radius `r`"""
        raise NotImplementedError
        
    def rotate_in_place(self):
        """For debugging."""
        twist = Twist()
        
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            twist.angular.z = 0.1
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
    
    def run(self):
        self.twist = Twist()
        rate = rospy.Rate(10)

        while rospy.get_time() == 0:
            start_time = rospy.get_time()

        while not rospy.is_shutdown():
            # self.twist.angular.z = 0.4
            current_time = rospy.get_time()
            time_dif = current_time - start_time
            if time_dif < 30:
                self.twist.linear.x = (time_dif / 30.0) * MAX_SPEED
                print("Time passed:", time_dif, "Speed:", (time_dif / 30.0) * MAX_SPEED)
            elif time_dif < 60:
                self.twist.linear.x = (2 - (time_dif / 30.0)) * MAX_SPEED
                print("Time passed:", time_dif, "Speed:", (2 - (time_dif / 30.0)) * MAX_SPEED)
            elif time_dif > 60:
                pass
            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('basic_mover')
    # BasicMover().out_and_back(1)
    # BasicMover().draw_square(1)
    # BasicMover().move_in_a_circle(1)
    # BasicMover().rotate_in_place()
    Solution().run()
