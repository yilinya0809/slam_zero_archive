#!/usr/bin/env python
import rospy
import numpy as np
import sys
import random
import math
import std_msgs
import slam
from std_msgs.msg import Float64
from slam.msg import Data
from slam.msg import Kinematics
import time
import message_filters

def kinematics(msg1, msg2):
    slam.msg.Kinematics.x = msg1.x;
    slam.msg.Kinematics.y = msg1.y;
    slam.msg.Kinematics.ax = msg1.local_ax;
    slam.msg.Kinematics.ay = msg1.local_ay;
    slam.msg.Kinematics.theta = msg1.theta;
    slam.msg.Kinematics.s = msg2.s;

    pub.publish(kinematics)

if __name__ == '__main__':
    rospy.init_node("message_filter")
    pub = rospy.Publisher("/current_kinematics", slam.msg.Kinematics)
    sub_1 = message_filters.Subscriber("/filtered_data", slam.msg.Data)
    sub_2 = message_filters.Subscriber("/vehicle_state", path_planning.msg.???)

    ts = message_filters.TimeSynchronizer([sub_1, sub_2], 10)
    ts.registerCallback(kinematics)
    rospy.spin()

