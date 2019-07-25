#!/usr/bin/env python

import os
import time
import re
import rospy
import numpy as np
import subprocess 
import carController
import boundingBox
import kalman_filter
import coordinates
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy


class runner(object):
	def __init__(self):
		self.c=carController.carController()
		self.box=boundingBox.boundingBox()
		self.lastbox1=boundingBox.boundingBox()
		self.lastbox2=boundingBox.boundingBox()
		self.lastbox3=boundingBox.boundingBox()
		self.lastbox4=boundingBox.boundingBox()
		self.lastbox5=boundingBox.boundingBox()
		self.empty=boundingBox.boundingBox()
		self.empty.setPrediction(1)
		self.car=boundingBox.carCharacteristics()
		self.flag=0
		self.pred=""
		#min_area=10000.0
		#max_area=42000.0
		self.check=0
		self.time=rospy.get_time()
		self.rtime=0.0
		self.input=""
		self.count=0
		self.sub=rospy.Subscriber("/detectnet/detections", String, self.callback)
		self.manualControl=0
		self.sub2=rospy.Subscriber("joy", Joy, self.callback2)
		self.sub3=rospy.Subscriber("depthwitherror",Float32MultiArray, self.callback3)
		self.kf=kalman_filter.kalman_filter(1023,0.125,32,0)
                
	
	def callback(self, data):
		if(self.manualControl==0):		
			self.input=str(data)
			self.input=self.input[7:-1]
			#print self.input	
			self.check=1
			self.lastbox1.set(self.lastbox2)
			self.lastbox2.set(self.lastbox3)
			self.lastbox3.set(self.lastbox4)
			self.lastbox4.set(self.lastbox5)	
			if self.check==1:		
				if(re.search("BoundingBox:",self.input)):						
					index=self.input.find("BoundingBox:")
					#print self.input				
					self.lastbox5.set(self.input)
					self.box.set(self.lastbox5)
					if(self.box.getConfidence()<0.4):
						flag=2
						self.lastbox5.set(self.empty)
						self.box.set(self.lastbox4)				
					else:
						flag=1				
				elif(re.search("NoBoundingBox",self.input)):
					self.lastbox5.set(self.empty)
					flag=2
				else:
					print "Ideally, should not reach here"				
			else:
				flag=3		
			box_decision=0
			no_box_decision=0	
			if(self.lastbox1.getPrediction()==1):
				no_box_decision+=1
			elif(self.lastbox1.getPrediction()==2):
				box_decision+=1
			if(self.lastbox2.getPrediction()==1):
				no_box_decision+=1
			elif(self.lastbox2.getPrediction()==2):
				box_decision+=1
			if(self.lastbox3.getPrediction()==1):
				no_box_decision+=1
			elif(self.lastbox3.getPrediction()==2):
				box_decision+=1
			if(self.lastbox4.getPrediction()==1):
				no_box_decision+=1
			elif(self.lastbox4.getPrediction()==2):
				box_decision+=1	
			if(self.lastbox5.getPrediction()==1):
				no_box_decision+=1
			elif(self.lastbox5.getPrediction()==2):
				box_decision+=1
			if(box_decision>=3):
				self.car.calculateMaxSteer(self.box)
			self.car.setSteer()
			self.car.setThrottle(self.box)
			print(int(self.car.getActualSteer()),int(self.car.getActualThrottle()))
			self.c.set(int(self.car.getActualSteer()),int(self.car.getActualThrottle()))
			
	
	def callback2(self,data):
		if(data.buttons[6]==1):
			self.manualControl=1
			print " LB "
			self.sub.unregister()
		if(data.buttons[7]==1):
			self.manualControl=0
			print " RB "
			self.count=0
			self.sub=rospy.Subscriber("/detectnet/detections", String, self.callback)
		if(self.manualControl==1):		
			if(data.buttons[11]==1):
				print "Emergency stop"
				self.car.stop()
			if(data.buttons[0]==1):
				self.car.changeThrottle(-3)
			if(data.buttons[4]==1):
				self.car.changeThrottle(3)
			if(data.axes[0]!=0):
				self.car.changeSteer(100*data.axes[0])
			self.c.set(int(self.car.getActualSteer()),int(self.car.getActualThrottle()))	
			print self.car.getDetails()

	def callback3(self,data):
		if self.manualControl==0:		
			if(data.data[0]>160 or data.data[0]==-1 or data.data[0]<45):
				self.car.stop()
			else:
				self.car.calculateMaxThrottle(data.data[0])
	
	def running(self):
		print "Hello"
		rate = rospy.Rate(2)
		while not rospy.is_shutdown():
			if self.manualControl==0:
				self.car.setSteer()
				self.car.setThrottle(self.box)
				print(int(self.car.getActualSteer()),int(self.car.getActualThrottle()))
				self.c.set(int(self.car.getActualSteer()),int(self.car.getActualThrottle()))
			self.check=0
			rate.sleep()
		rospy.spin()
		

if __name__ == '__main__':
	rospy.init_node("runner", anonymous=True)
	my_node = runner()
	my_node.running()
