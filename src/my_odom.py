#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point 
from tf.transformations import euler_from_quaternion

class MyOdom:
    def __init__(self):
        data_odom = None
        while data_odom is None:
            try:
                data_odom = rospy.wait_for_message('/odom', Odometry, timeout=1)
            except:
                rospy.loginfo("Current odom not ready yet, retrying for setting up init pose")
        self.old_pose = data_odom.pose.pose
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.my_odom_pub = rospy.Publisher('my_odom', Point, queue_size=1)
        self.dist = 0.0
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
        Updates `self.dist` to the distance between `self.old_pose` and
        `cur_pose`.
        """
        self.dist = cur_pose.position.x - self.old_pose.position.x

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
        Publish `self.dist` and `self.yaw` on the `my_odom` topic.
        """
        # The `Point` object should be used simply as a data container for
        # `self.dist` and `self.yaw` so we can publish it on `my_odom`.
        cur_point = Point()
        cur_point.x = self.dist
        cur_point.z = self.yaw
        self.my_odom_pub.publish(cur_point)
        
if __name__ == '__main__':
    rospy.init_node('my_odom')
    MyOdom()
    rospy.spin()
