#!/usr/bin/env python
import roslib; roslib.load_manifest('rosbag')
import rosbag
import sys
import os
 
if len(sys.argv) < 3:
    print 'Please specify the bag file to parse followed by topics'
    print 'For example:'
    print '    ', sys.argv[0], 'test.bag', '/wirelessPowerRx/amps.data','/wirelessPowerRx/volts5.header.stamp'
    print
 
    print """
    Note that these fields are output together on a single line each
    time the first field is received.
    """
    exit(1)
 
#First arg is the bag to look at
bagfile = sys.argv[1]
 
#The topic names
topics = []
#The fields of each topic to print out, indexed by topic name
topicFields = {}
for i in range(2,len(sys.argv)):
    #Split the input into the topic (left of the first .) and field to look at (right of first .)
    splitVals = sys.argv[i].split('.',1)
    topics.append(splitVals[0])
    topicFields[splitVals[0]] = splitVals[1]
    print splitVals
 
 
lastValue = {}
 
#Print the values found in latValue in a comma separated format
def printValuesCSV():
    print(lastValue[topics[0]]),
    sys.stdout.softspace=False
    for i in range(1,len(topics)):
        sys.stdout.write(', ')
        #sys.stdout.write('%.2f'%(lastValue[topics[i]]))
        print(lastValue[topics[i]]),
        sys.stdout.softspace=False
    print
     
print 'Working directory: ' + os.getcwd()
print 'Bag file: ' + bagfile
print 'Outputting the following topics and fields: '
print topicFields
 
#Init the lastValue dictionary to zeros
for topic in topics:
    lastValue[topic] = 0.0
 
 
#Go through the bag file
bag = rosbag.Bag(bagfile)
for topic, msg, t in bag.read_messages(topics=topics):
    lastValue[topic] = eval('msg.' + topicFields[topic])
    #Every time we see an instance of the first element, print out everything
    if topic == topics[0]:
        printValuesCSV()
 
bag.close();
