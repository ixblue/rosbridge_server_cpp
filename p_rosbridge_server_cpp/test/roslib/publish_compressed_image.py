#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage

rospy.init_node("publish_compressed_image")
pub = rospy.Publisher("/compressed_image", CompressedImage, queue_size=10, latch=True)
msg = CompressedImage()
msg.header.stamp = rospy.Time.now()
msg.header.frame_id = "frame"
msg.format = "jpeg"
msg.data = [0,1,2,3,4,5,6,7,8,9]
pub.publish(msg)

rospy.spin()
