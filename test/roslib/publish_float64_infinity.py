#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

rospy.init_node("publish_float64_infitity")
pub = rospy.Publisher("/listen_double", Float64, queue_size=10, latch=True)
pub.publish(float('inf'))

rospy.spin()
