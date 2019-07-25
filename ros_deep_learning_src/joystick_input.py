#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data):
        twist = Twist()
        twist.linear.x = -100*data.axes[0]
        twist.angular.z = -100*data.axes[1]
        pub.publish(twist)
	print data
def start():
        global pub
        pub = rospy.Publisher('vel', Twist, queue_size=10)
        rospy.Subscriber("joy", Joy, callback)
        rospy.init_node('joystick_input')
        rospy.spin()

if __name__ == '__main__':
        start()
