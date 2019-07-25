#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

steer_pub = rospy.Publisher('steer', Float32, queue_size=10)
throttle_pub = rospy.Publisher('throttle', Float32, queue_size=10)	

def callback1(data):	
	
	if(data.linear.y==1):
		throttle_pub.publish(200.0)
		steer_pub.publish(200.0)
	else:
		steer_pub.publish(data.linear.x)
	if(data.linear.y!=1 and data.angular.x==1):
		throttle_pub.publish(data.angular.z+400.0)
	elif(data.linear.y!=1 and data.angular.y==1):
		throttle_pub.publish(data.angular.z+700.0)
	elif(data.linear.y!=1):
		throttle_pub.publish(data.angular.z)

def controller():
	rospy.init_node('controller', anonymous=True) 
	rospy.Subscriber("vel", Twist, callback1)
    	rospy.spin()

if __name__ == '__main__':
	try:
		controller()
	except rospy.ROSInterruptException:
		pass
