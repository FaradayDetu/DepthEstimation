#!/usr/bin/env python

from sensor_msgs.msg import Image
import rospy
def callback(data):
	pass
def start():
        rospy.Subscriber("/stereo_dnn_ros/network/output", Image, callback)
        rospy.init_node('test')
        rospy.spin()
if __name__ == '__main__':
	start()
