#!/usr/bin/python

import roslib
import rosbag
import sys
import os
import pickle
import std_msgs
import numpy as np
import numpy.core.umath as umath

roslib.load_manifest('lrm_msgs')
import lrm_msgs

from lrm_msgs import Encoders

roslib.load_manifest('move_base_msgs')
import move_base_msgs

import subprocess, yaml


  
def callback_encoders(data):
    print data
    
def callback_feedback(data):
    print data
    
def callback_status(data):
    print data

def callbakc_result(data):
    print data
    
def run():
    rospy.init_node('trial_metrics_node', anonymous=True)
    
    rospy.Subscriber("/encoders_controller/encoders", lrm_msgs.Encoders, callback_encoders)
    
    print "o"
    
    
    
    
if __name__ == "__main__":
    run()
   
 