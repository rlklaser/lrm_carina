#!/usr/bin/python
import roslib; roslib.load_manifest('odom_calibration')
import rospy
from math import *
#from irobot_create_2_1.srv import *
from position_tracker.msg import Position
from geometry_msgs.msg import Twist

SPEED = 0.5
TURN_FACTOR = 1

def set_speed(current, goal, from_dir):

    if (current >= goal and from_dir == -1) or \
            (current <= goal and from_dir == 1):
        return 0
    speed = fabs(current-goal)

    if speed > SPEED:
        speed = SPEED
    elif speed < 0.1:
        speed = 0.1
    return speed

class Calibrator:
    def __init__(self):
        self.step_number = 0
        self.pub = rospy.Publisher('cmd_vel', Twist)

    def act(self, position):
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

        if self.step_number == 0:
            twist.linear.x = set_speed(position.x, 1.0, -1)
            if twist.linear.x == 0:
                self.step_number = 1
        elif self.step_number == 1:
            twist.angular.z = TURN_FACTOR*set_speed(position.theta, pi/2, -1)
            if twist.angular.z == 0:
                self.step_number = 2
        elif self.step_number == 2:
            twist.linear.x = set_speed(position.y, 1.0, -1)
            if twist.linear.x == 0:
                self.step_number = 3
        elif self.step_number == 3:
            theta = position.theta
            if theta < 0: # set_speed doesn't understand the -pi to
                          # +pi system.  This is a workaround.
                theta += (2*pi)
            twist.angular.z = TURN_FACTOR*set_speed(theta, pi, -1)
            if twist.angular.z == 0:
                self.step_number = 4
        elif self.step_number == 4:
            twist.linear.x = set_speed(position.x, 0.0, 1)
            if twist.linear.x == 0:
                self.step_number = 5
        elif self.step_number == 5:
            twist.angular.z = TURN_FACTOR*set_speed(position.theta, -pi/2, -1)
            if twist.angular.z == 0:
                self.step_number = 6
        elif self.step_number == 6:
            twist.linear.x = set_speed(position.y, 0.0, 1)
            if twist.linear.x == 0:
                self.step_number = 7
        elif self.step_number == 7:
            twist.angular.z = TURN_FACTOR*set_speed(position.theta, 0.0, -1)
            if twist.angular.z == 0:
                self.step_number = 8
        self.pub.publish(twist)

if __name__ == '__main__':
    
    rospy.init_node('odom_calibration')
    calibrator = Calibrator()

    rospy.Subscriber('position', Position, calibrator.act)

    rospy.spin()
