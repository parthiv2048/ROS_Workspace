#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion

# Video Link - https://drive.google.com/file/d/19ZA3Q_ObsgSSiAriUOe4U9kmO3MLbKID/view?usp=sharing

MAX_ANGULAR_SPEED = math.pi / 4
MIN_ANGULAR_SPEED = 0.0005
MAX_SPEED = 0.3

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
        # msg.z is the angular z i.e. yaw of the robot
        self.cur_yaw = msg.z
    
    # Find out the shortest angular distance between self.cur_yaw and target_yaw
    def calculate_shortest_angle_diff(self, target_yaw, current_yaw):
        # Convert current_yaw to make sure it only has positive values
        current_yaw = current_yaw if current_yaw > 0 else (current_yaw + 2*math.pi)
        # Shorten target_yaw to make sure it doesn't exceed 2pi
        target_yaw = target_yaw % (2*math.pi)
        # Find the angular difference, and then shift it by 2pi and -2pi
        # The shortest distance must exist between these 3 values
        direct_diff = target_yaw - current_yaw
        angle_diffs = [direct_diff, direct_diff + 2*math.pi, direct_diff - 2*math.pi]
        # The following finds the shortest angular distance between current_yaw and target_yaw
        # A loop is used because while the comparison is done between absolute values, 
        # we still need the sign of the difference to know which direction to rotate
        shortest_diff_abs = abs(direct_diff)
        shortest_diff = direct_diff
        for angle_diff in angle_diffs:
            if abs(angle_diff) < shortest_diff_abs:
                shortest_diff_abs = abs(angle_diff)
                shortest_diff = angle_diff
        
        return shortest_diff

    def turn_to_heading(self, target_yaw):
        """
        Turns the robot to heading `target_yaw`.
        """
        while self.cur_yaw is None:
            pass
        
        shortest_diff = self.calculate_shortest_angle_diff(target_yaw, self.cur_yaw)

        # Find the total time needed to travel using angular distance (shortest_diff) and
        # assume we rotationally accelerate to MAX_ANGULAR_SPEED in the first half and
        # then decelerate from MAX_ANGULAR_SPEED to 0 in the second half
        total_time = 2 * abs(shortest_diff) / MAX_ANGULAR_SPEED
        dir = 1 if shortest_diff > 0 else -1
        
        twist = Twist()
        rate = rospy.Rate(20)

        start_time = rospy.get_time()
        prev_shortest_diff = shortest_diff

        while not rospy.is_shutdown():
            time_passed = rospy.get_time() - start_time
            shortest_diff = self.calculate_shortest_angle_diff(target_yaw, self.cur_yaw)

            # This is to counter a very specific bug that occurs with the simulation
            # Sometimes, when the yaw is hovering around pi, the self.cur_yaw will switch from pi to -pi at random
            # To stop this, we detect if it happens and when it does, the shortest_diff stays the same
            if abs(shortest_diff - prev_shortest_diff) > math.pi:
                shortest_diff = prev_shortest_diff
            prev_shortest_diff = shortest_diff
            
            if time_passed < (total_time / 2.0):
                # In the first half, rotationally accelerate until you reach MAX_ANGULAR_SPEED
                # The copysign is used to make sure the robot is rotating in the correct direction
                twist.angular.z = (time_passed / (total_time / 2.0)) * MAX_ANGULAR_SPEED * dir
            elif time_passed < total_time:
                # In the second half, rotationally decelerate from MAX_ANGULAR_SPEED until you reach 0
                twist.angular.z = (2 - (time_passed / (total_time / 2.0))) * MAX_ANGULAR_SPEED * dir
            
            # Sometimes the robot rotates so slowly that some bugs start to happen,
            # so we limit the lowest angular speed to MIN_ANGULAR_SPEED
            twist.angular.z = dir * max(abs(twist.angular.z), MIN_ANGULAR_SPEED)
            
            # Stop immediately if we've gone overboard or we're close enough
            if shortest_diff * dir < 0 or abs(shortest_diff) < 0.001:
                twist.angular.z = 0
                self.cmd_vel_pub.publish(twist)
                break
            
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
        # Find the total time needed to travel using target_dist and
        # assume we accelerate to MAX_SPEED in the first half and then decelerate to 0 in the second half
        total_time = 2 * target_dist / MAX_SPEED
        start_time = rospy.get_time()

        while not rospy.is_shutdown():
            dist_trav = math.hypot(self.x - init_x, self.y - init_y)
            time_passed = rospy.get_time() - start_time

            if time_passed < (total_time / 2.0):
                # In the first half, accelerate until you reach MAX_SPEED
                twist.linear.x = (time_passed / (total_time / 2.0)) * MAX_SPEED
            elif time_passed < total_time:
                # In the second half, decelerate from MAX_SPEED until you reach 0
                twist.linear.x = (2 - (time_passed / (total_time / 2.0))) * MAX_SPEED
            
            # If the robot has gone overboard, immediately stop
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
        self.turn_to_heading(self.cur_yaw + math.pi)
        self.move_forward(target_dist)

    def draw_square(self, side_length):
        """
        This function moves the robot in a square with `side_length` meter sides.
        """

        while self.cur_yaw is None:
            pass
        
        for i in range(3):
            self.move_forward(side_length)
            # By adding pi/2 radians to your current yaw, you're turning 90 degrees
            self.turn_to_heading(self.cur_yaw + (math.pi / 2))
        
        # The last side of the square
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
        # Find the total time needed to travel using the circumference of the circle with radius r, and
        # assume we accelerate to MAX_SPEED in the first half and then decelerate to 0 in the second half
        total_time = 4 * math.pi * r / MAX_SPEED
        start_time = rospy.get_time()

        while rospy.get_time() == 0:
            start_time = rospy.get_time()

        while not rospy.is_shutdown():
            dist_trav = math.hypot(self.x - init_x, self.y - init_y)
            time_passed = rospy.get_time() - start_time
            if time_passed < (total_time / 2.0):
                # In the first half, accelerate (both linearly and rotationally) until you reach MAX_SPEED
                twist.linear.x = (time_passed / (total_time / 2.0)) * MAX_SPEED
                twist.angular.z = (time_passed / (total_time / 2.0)) * MAX_SPEED / r
            elif time_passed < total_time:
                # In the second half, decelerate (both linearly and rotationally) from MAX_SPEED until you reach 0
                twist.linear.x = (2 - (time_passed / (total_time / 2.0))) * MAX_SPEED
                twist.angular.z = (2 - (time_passed / (total_time / 2.0))) * MAX_SPEED / r
            
            # This checks if the robot has returned to its original position which 
            # means that a full circle has been completed so the robot immediately comes to a stop.
            # The first condition involving time_passed is used just to make sure the robot doesn't immediately stop because
            # in the beginning, the robot will be very close to its original position
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
    # BasicMover().draw_square(1)
    # BasicMover().move_in_a_circle(1)
    # BasicMover().rotate_in_place()
