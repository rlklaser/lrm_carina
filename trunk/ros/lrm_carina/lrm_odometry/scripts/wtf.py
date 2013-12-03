#! /usr/bin/env python

import roslib; roslib.load_manifest('lrm_odometry')
import sys
import rospy
from lrm_odometry.srv import *

if __name__ == '__main__':
    rospy.init_node('spawner', anonymous=True)
    print 'looking for node robot_pose_ekf...'
    rospy.wait_for_service('robot_pose_ekf/get_status')

    s = rospy.ServiceProxy('robot_pose_ekf/get_status', GetStatus)
    resp = s.call(GetStatusRequest())
    print resp.status
