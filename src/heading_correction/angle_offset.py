#!/usr/bin/env python
import rospy
import numpy as np
import sys
import random
import math
import std_msgs
import slam
from std_msgs.msg import Float64
from slam.msg import Float_header
import time
import message_filters

def angle_diff(msg1, msg2):
    angle_diff = msg1.data - msg2.data
    
    pub.publish(angle_diff)

if __name__ == '__main__':
    rospy.init_node("angle_difference")
    pub = rospy.Publisher("/angle_diff", std_msgs.msg.Float64)
    sub_1 = message_filters.Subscriber("/angle_1", slam.msg.Float_header)
    sub_2 = message_filters.Subscriber("/angle_2", slam.msg.Float_header)

    ts = message_filters.TimeSynchronizer([sub_1, sub_2], 10)
    ts.registerCallback(angle_diff)
    rospy.spin()

