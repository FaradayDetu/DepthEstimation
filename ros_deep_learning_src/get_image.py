#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_right_pub = rospy.Publisher("image_topic_r_publisher",Image,queue_size=10)
    self.image_left_pub = rospy.Publisher("image_topic_l_publisher",Image,queue_size=10)

    self.bridge = CvBridge()

    self.image_right_sub = rospy.Subscriber("/zed/zed_node/right/image_rect_color",Image,self.callback_right)
    self.image_left_sub = rospy.Subscriber("/zed/zed_node/left/image_rect_color",Image,self.callback_left)

  def callback_left(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      #cv_image_edited = cv2.resize(cv_image,(int(1280),int(720)))
    except CvBridgeError as e:
      print(e)

    try:
      self.image_left_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      
    except CvBridgeError as e:
      print(e)

  def callback_right(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
     # cv_image_edited = cv2.resize(cv_image,(int(1280),int(720)))
    except CvBridgeError as e:
      print(e)

    try:
      self.image_right_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      
    except CvBridgeError as e:
      print(e)


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
