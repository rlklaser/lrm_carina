#!/usr/bin/python
import roslib
import rospy
import sys

if __name__ == '__main__': 
#    rospy.init_node('conv_to_road')
    file = sys.argv[1]
    f = open(file, 'r')
    ox = 0.0
    oy = 0.0
    while True:
        line = f.readline()
        if not line:
            break
        s = line.split()
        #print s
        x = float(s[0])
        y = float(s[1])
        if(oy==0 and ox==0):
            ox = x
            oy = y
        x-=ox
        y-=oy
        print '<point>%f %f 0</point>' % (x, y)
    