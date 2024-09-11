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
        self.cur_yaw = None
        self.cur_dist = None

    def my_odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        self.cur_yaw = msg.z
        self.cur_dist = msg.x

    def turn_to_heading(self, target_yaw):
        """
        Turns the robot to heading `target_yaw`.
        """
        while self.cur_yaw is None:
            pass
        
        twist = Twist()
        rate = rospy.Rate(10)

        target_rad = target_yaw * math.pi / 180

        while not rospy.is_shutdown():
            if math.isclose(target_rad, self.cur_yaw):
                break
            twist.angular.z = 0.5 * (target_rad - self.cur_yaw)
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
    def move_forward(self, target_dist):
        """Moves the robot forward by `target_dist`."""
        while self.cur_dist is None:
            pass

        twist = Twist()
        rate = rospy.Rate(10)
        initial_dist = self.cur_dist
        # final_dist = initial_dist + target_dist
        max_speed = 0.2

        while not rospy.is_shutdown():
            dist_trav = self.cur_dist - initial_dist
            if dist_trav < (target_dist / 2.0):
                twist.linear.x = max_speed
            elif dist_trav > (target_dist / 2.0):
                twist.linear.x = (target_dist - dist_trav) * max_speed
            if dist_trav >= target_dist:
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

if __name__ == '__main__':
    rospy.init_node('basic_mover')
    # BasicMover().turn_to_heading(90)
    # BasicMover().move_forward(3)
    BasicMover().out_and_back(1)
    # BasicMover().draw_square(1)
    # BasicMover().move_in_a_circle(1)
    # BasicMover().rotate_in_place()
