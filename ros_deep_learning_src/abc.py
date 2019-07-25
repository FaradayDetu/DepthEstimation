#!/usr/bin/env python

import rospy
from std_msgs.msg import String

pub = rospy.Publisher('counter', String, queue_size=10)

def abc():
	rospy.init_node('abc', anonymous=True)				
	rate = rospy.Rate(10)	
	while not rospy.is_shutdown():
		pub.publish("1")	
		rate.sleep()

if __name__ == '__main__':
	try:
		abc()
	except rospy.ROSInterruptException:
		pass
