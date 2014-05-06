#!/usr/bin/python

import rospy
from tf import TransformListener



def run():
    
    tt = tf.Transformer(True, rospy.Duration(10.0))
    
    tfl = TransformListener()
    #print tf.allFramesAsString()
    print tfl.getFrameStrings()
    if tfl.frameExists("/base_link") and tfl.frameExists("/base_footprint"):
        t = tfl.getLatestCommonTime("/base_link", "/base_footprint")
        position, quaternion = tfl.lookupTransform("/base_link", "/base_footprint", t)
        print position, quaternion
    else:
        print "no"
            
            
if __name__ == "__main__":
    rospy.init_node('getframes')
    
    run()
    #try:
    #    rospy.sleep(1)
    #    run()
    #except rospy.ROSInterruptException: pass
    
