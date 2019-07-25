#!/usr/bin/env python

from __future__ import print_function
import re
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class box_conversor:

  def __init__(self):
    self.cropped_disp_bottle_pub_resize = rospy.Publisher("cropped_disp_bottle_pub_resize",Image)
    self.cropped_disp_bottle_pub_ori = rospy.Publisher("cropped_disp_bottle_pub_ori",Image)
    self.bridge = CvBridge()
    self.image_right_sub = rospy.Subscriber("image_topic_r_publisher",Image,self.callback_right)
    self.image_disp = rospy.Subscriber("/stereo_dnn_ros/network/output",Image,self.callback_disp)
    self.bounding_box_sub = rospy.Subscriber("/detectnet/detections",String,self.callback_bbox)
    
  def callback_disp(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "8UC1")
      self.disp_img = cv_image
      self.dimensions_disp = cv_image.shape
    except CvBridgeError as e:
      print(e)
  
  def callback_right(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.dimensions_right = cv_image.shape
    except CvBridgeError as e:
      print(e)

  def callback_bbox(self,data):
    try:

      x=str(data)
      x=x[7:-1]
      y=re.split(":",x)
      x1=float(y[1])
      y1=float(y[2])
      x2=float(y[3])
      y2=float(y[4])
      y_ratio = float(self.dimensions_disp[0])/float(self.dimensions_right[0])
      x_ratio = float(self.dimensions_disp[1])/float(self.dimensions_right[1])
      new_y1 = y1*y_ratio
      new_y2 = y2*y_ratio
      new_x1 = x1*x_ratio
      new_x2 = x2*x_ratio

      #im_color = cv2.applyColorMap(self.disp_img, cv2.COLORMAP_JET)
      #im_color = cv2.cvtColor(src, bwsrc, cv::COLOR_RGB2GRAY);
      #cv2.imshow("Image window1", im_color)
      #cv2.rectangle(self.disp_img,(int(new_x1),int(new_y1)),(int(new_x2),int(new_y2)),(0,255,0),3)
      #crop_img = im_color[new_y1:new_y2, new_x1:new_x2]

      crop_img = self.disp_img[new_y1:new_y2, new_x1:new_x2]
      img_final = cv2.resize(crop_img,(int(50),int(200)))

      self.cropped_disp_bottle_pub_resize.publish(self.bridge.cv2_to_imgmsg(img_final, "8UC1"))
      self.cropped_disp_bottle_pub_ori.publish(self.bridge.cv2_to_imgmsg(crop_img, "8UC1"))
      print("Success!")
    except:
      print("No data gathered")





def main(args):
  rospy.init_node('box_conversor', anonymous=True)
  bc = box_conversor()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
