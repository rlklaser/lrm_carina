#!/usr/bin/python

import roslib; roslib.load_manifest('lrm_outdoor')
import rospy
from geomag.geomag import GeoMag

if __name__ == "__main__":
  rospy.init_node("test_declination")
  gm = GeoMag("../src/geomag/WMM.COF")
  #mag = gm.calc(43.411454, -80.472708)
  mag = gm.calc(-22.411454, -44.472708)
  print mag.dec
