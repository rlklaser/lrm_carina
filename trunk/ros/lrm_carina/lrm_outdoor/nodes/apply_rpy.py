#!/usr/bin/env python

import roslib; roslib.load_manifest('declination')
import rospy

from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3Stamped


class DeclinationApply:
    def __init__(self):
        rospy.init_node("declination_applier")
        self.decl = rospy.get_param("~default", None)

        self.sub = rospy.Subscriber("rpy_mag", Vector3Stamped, self._rpy)
        self.pub = rospy.Publisher("rpy", Vector3Stamped)

        rospy.Subscriber("declination", Float32, self._decl_cb)

    def _decl_cb(self, msg):
        self.decl = msg.data

    def _rpy(self, msg):
        if self.decl:
            msg.vector.z += self.decl
            self.pub.publish(msg)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    DeclinationApply().spin()
