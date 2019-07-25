#!/usr/bin/env python

import kalman_filter
import random
import rospy
import boundingBox
from std_msgs.msg import String
import re


class kalman_test(object):
	def __init__(self):
		self.kf=kalman_filter.kalman_filter(1023,0.125,32,0)
		self.box=boundingBox.boundingBox()

	def callback(self,data):
		in_data=str(data)
		in_data=in_data[7:-1]
		if(re.search("BoundingBox:",in_data)):
			index=in_data.find("BoundingBox:")				
			self.box.set(in_data)
			#m=(self.box.getHeight()*0.04)/20
			#h=(18.520833333*(1+m))/m
			h=254.6/(self.box.getHeight()*0.04)			
			h_filtered=self.kf.getValue(h)
			print "From BB ",h," ",h_filtered," ",self.box.getHeight()*0.04
		elif(re.search("NoBoundingBox",in_data)):
			h=self.kf.getX()
			h_filtered=self.kf.getValue(h)
			print "No BB    ",h," ",h_filtered
					

	def runs(self):
		rospy.Subscriber("/detectnet/detections", String, self.callback)
		rospy.spin()
		

if __name__ == '__main__':
	rospy.init_node("joystick_runner", anonymous=True)
	node = kalman_test()
	node.runs() 
