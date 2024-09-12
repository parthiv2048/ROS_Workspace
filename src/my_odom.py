#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point 
from tf.transformations import euler_from_quaternion

class MyOdom:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.my_odom_pub = rospy.Publisher('my_odom', Point, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def odom_cb(self, msg):
        """Callback function for `odom_sub`."""
        cur_pose = msg.pose.pose
        self.update_dist(cur_pose)
        self.update_yaw(cur_pose.orientation)
        self.publish_data()

    def update_dist(self, cur_pose):
        """
        Helper to `odom_cb`.
        Updates `self.x` to the current x position of robot and
        Updates `self.y` to the current y position of robot.
        """
        self.x = cur_pose.position.x
        self.y = cur_pose.position.y

    def update_yaw(self, cur_orientation):
        """
        Helper to `odom_cb`.
        Updates `self.yaw` to current heading of robot.
        """
        orientation_list = [cur_orientation.x, cur_orientation.y, cur_orientation.z, cur_orientation.w]
        (cur_roll, cur_pitch, cur_yaw) = euler_from_quaternion(orientation_list)
        self.yaw = cur_yaw

    def publish_data(self):
        """
        Publish `self.x` and `self.y` and `self.yaw` on the `my_odom` topic.
        """
        # The `Point` object should be used simply as a data container for
        # `self.dist` and `self.yaw` so we can publish it on `my_odom`.
        cur_point = Point()
        cur_point.x = self.x
        cur_point.y = self.y
        cur_point.z = self.yaw
        self.my_odom_pub.publish(cur_point)
        
if __name__ == '__main__':
    rospy.init_node('my_odom')
    MyOdom()
    rospy.spin()
