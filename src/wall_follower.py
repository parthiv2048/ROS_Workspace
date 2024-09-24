#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from PID import PID

V_MAX = 0.4
R_SAFE = 0.2
D_W = 1
D_EFFECT = 3

class WallFollower:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.v = 0.2
        self.w = 0
        self.angular_pid = PID(0.4, 0.05, 0.2)
        self.vel_pid = None
        self.d_wall = 1
    
    def filter_range(self, ranges, range_min):
        filtered_range = [r for r in ranges if not(math.isnan(r)) and not(math.isinf(r)) and r > range_min]
        if len(filtered_range) == 0:
            return [0]
        return filtered_range

    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        """

        # Get points needed to construct virtual wall
        # angles = [-6, -9, -18, -45, -60, -90]
        angles = [-45, -60, -90, -120, -135]
        P_w = []
        dist_0 = min(self.filter_range(msg.ranges[0:5] + msg.ranges[355:], msg.range_min))
        P_w.append([dist_0 * math.cos(0), dist_0 * math.sin(0)])
        for ang in angles:
            adj_ang = 360 + ang
            dist = min(self.filter_range(msg.ranges[adj_ang-5:adj_ang+5], msg.range_min))
            if dist > D_EFFECT:
                continue
            P_w.append([dist * math.cos(ang * (math.pi / 180)), dist * math.sin(ang * (math.pi / 180))])
        
        # print(P_w)
        
        # dist_6 = min(self.filter_range(msg.ranges[1:11], msg.range_min, msg.range_max))
        # P_w.append([dist_6 * math.cos(-6 * (math.pi / 180)), dist_6 * math.sin(-6 * (math.pi / 180))])
        # dist_9 = min(self.filter_range(msg.ranges[4:14], msg.range_min, msg.range_max))
        # P_w.append([dist_9 * math.cos(-9 * (math.pi / 180)), dist_9 * math.sin(-9 * (math.pi / 180))])
        # dist_18 = min(self.filter_range(msg.ranges[337:347], msg.range_min, msg.range_max))
        # P_w.append([dist_18 * math.cos(-18 * (math.pi / 180)), dist_18 * math.sin(-18 * (math.pi / 180))])
        # dist_45 = min(self.filter_range(msg.ranges[310:320], msg.range_min, msg.range_max))
        # P_w.append([dist_45 * math.cos(-45 * (math.pi / 180)), dist_45 * math.sin(-45 * (math.pi / 180))])
        # dist_60 = min(self.filter_range(msg.ranges[295:305], msg.range_min, msg.range_max))
        # P_w.append([dist_60 * math.cos(-60 * (math.pi / 180)), dist_60 * math.sin(-60 * (math.pi / 180))])
        # dist_90 = min(self.filter_range(msg.ranges[265:275], msg.range_min, msg.range_max))
        # P_w.append([dist_90 * math.cos(-90 * (math.pi / 180)), dist_90 * math.sin(-90 * (math.pi / 180))])

        # Calculate distance ahead
        dist_0 = min(dist_0, D_EFFECT)
        dist_neg_18 = min(min(self.filter_range(msg.ranges[337:347], msg.range_min)), D_EFFECT)
        dist_18 = min(min(self.filter_range(msg.ranges[13:23], msg.range_min)), D_EFFECT)
        d_0 = dist_0 * math.cos(0) + dist_neg_18 * math.cos(-18 * (math.pi / 180)) + dist_18 * math.cos(18 * (math.pi / 180))
        d_0 /= 3

        # Calculate Linear Velocity
        self.v = ((d_0 - R_SAFE) / (msg.range_max - R_SAFE)) * V_MAX

        # Construct Virtual Wall y = ax + b
        sum_x = 0
        sum_x2 = 0
        sum_y = 0
        sum_xy = 0
        for p in P_w:
            sum_x += p[0]
            sum_x2 += p[0] * p[0]
            sum_y += p[1]
            sum_xy += p[0] * p[1]
        
        y_bar = sum_y / len(P_w)
        x_bar = sum_x / len(P_w)
        
        a = (sum_xy - (sum_x * sum_y / len(P_w))) / (sum_x2 - ((sum_x * sum_x) / len(P_w)))
        b = y_bar - (a * x_bar)

        # Get distance between robot and virtual wall
        d = abs(b) / math.hypot(a, 1)
        # Get angle between robot and wall
        theta = math.atan(a)
        for point in P_w:
            print("X:", point[0], "Y:", point[1])
        print()
        print("y =", a,"* x +", b)
        print()
        print("d:", d)
        self.w = self.angular_pid.compute(D_W, d)

        # print("V:", self.v, "W:", self.w)

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