#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

def steerpublisher():
	pub = rospy.Publisher('steer_input', Int16, queue_size=10)
	rospy.init_node('steerpublisher', anonymous=True)
	rate = rospy.Rate(10)
	steer = 0
	while not rospy.is_shutdown():
		steer+=1
		rospy.loginfo(steer)
		pub.publish(Int16(steer))
		rate.sleep()

if __name__ == '__main__':
	try:
		steerpublisher()
	except rospy.ROSInterruptException:
		pass
