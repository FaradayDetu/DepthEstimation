#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

def throttlepublisher():
	pub = rospy.Publisher('throttle_input', Int16, queue_size=10)
	rospy.init_node('throttlepublisher', anonymous=True)
	rate = rospy.Rate(10)
	throttle = 0
	while not rospy.is_shutdown():
		throttle+=1
		rospy.loginfo(throttle)
		pub.publish(Int16(throttle))
		rate.sleep()

if __name__ == '__main__':
	try:
		throttlepublisher()
	except rospy.ROSInterruptException:
		pass
