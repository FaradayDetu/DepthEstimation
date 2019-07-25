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
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import numpy as np
import csv
#import sklearn
#from sklearn.mixture import GaussianMixture
class box_conversor:

  def __init__(self):
    
    self.bridge = CvBridge()
    self.image_left_sub = rospy.Subscriber("image_topic_l_publisher",Image,self.callback_left)
    self.image_disp = rospy.Subscriber("/stereo_dnn_ros/network/output",Image,self.callback_disp)
    self.bounding_box_sub = rospy.Subscriber("/detectnet/detections",String,self.callback_bbox)
    self.pub = rospy.Publisher('depth', Float32, queue_size=10)
    self.pub2 = rospy.Publisher('depthwitherror', Float32MultiArray , queue_size=10)
    self.count=0
    self.x_last=""
    self.start_time=rospy.get_time()

  def GetDepthMaxPeak(self,img):
    n, bins = np.histogram((1./img).flatten())
    depth = 2536.1205*np.float(bins[np.where(n == n.max())]) + 11.50
    error = (((bins[np.where(n == n.max())]*43.1381)**2) + (1.78**2) + (2536.1205*(bins[-1]/100))**2)**0.5
    return(depth,error)

  def GMModel_ori(self,img):
    #px_array = (1./img).flatten()
    #data=px_array[:,np.newaxis]
    #gmm = GaussianMixture(n_components = 2, covariance_type = 'spherical')
    #gmm.fit(data)

    #mu1= gmm.means_[0,0]
    #mu2= gmm.means_[1,0]
    #var1, var2 = gmm.covariances_
    gmm = GaussianMixture(n_components = 2, covariance_type = 'spherical')
    gmm.fit((1./img).flatten()[:,np.newaxis])
    obj_list = [(gmm.means_[0,0],gmm.covariances_[0]),(gmm.means_[1,0],gmm.covariances_[1])]

    x = np.array(obj_list)[np.argsort(np.array(obj_list)[:,1])[0],:][0]
    x_err = np.array(obj_list)[np.argsort(np.array(obj_list)[:,1])[0],:][1]

    depth = 2555.7183*x + 0.5693
    error = (((x*97.771)**2) + (4.412**2) + ((2555.7183*x_err)**2))**0.5
    
    return(depth,error)

  def GMModel_resi(self,img):
    #px_array = (1./img).flatten()
    #data=px_array[:,np.newaxis]
    #gmm = GaussianMixture(n_components = 2, covariance_type = 'spherical')
    #gmm.fit(data)

    #mu1= gmm.means_[0,0]
    #mu2= gmm.means_[1,0]
    #var1, var2 = gmm.covariances_
    gmm = GaussianMixture(n_components = 2, covariance_type = 'spherical')
    gmm.fit((1./img).flatten()[:,np.newaxis])
    obj_list = [(gmm.means_[0,0],gmm.covariances_[0]),(gmm.means_[1,0],gmm.covariances_[1])]

    x = np.array(obj_list)[np.argsort(np.array(obj_list)[:,1])[0],:][0]
    x_err = np.array(obj_list)[np.argsort(np.array(obj_list)[:,1])[0],:][1]

    depth = 2298.0726*x + 7.7627
    error = (((x*162.168)**2) + (7.357**2) + ((2298.0726*x_err)**2))**0.5
    
    return(depth,error)
      
    
  def callback_disp(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "8UC1")
      self.disp_img = cv_image
      self.dimensions_disp = cv_image.shape
    except CvBridgeError as e:
      print(e)
  
  def callback_left(self,data):
    try:
      self.cv_image_left = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.dimensions_left = self.cv_image_left.shape
    except CvBridgeError as e:
      print(e)

  def callback_bbox(self,data):
    
    try:
           
      if(re.search("NoBoundingBox",str(data)) and self.count>=2):
          error=0
	  depth=-1
      else:
	  if(re.search("NoBoundingBox",str(data)) and self.count<2):
	        self.count+=1
		x=self.x_last
	  elif(re.search("BoundingBox:",str(data))):
		x=str(data)
		self.x_last=x
		self.count=0
	  if(len(x)>4):	
		  x=x[7:-1]
		  y=re.split(":",x)
		  x1=float(y[1])
		  y1=float(y[2])
		  x2=float(y[3])
		  y2=float(y[4])
		  y_ratio = float(self.dimensions_disp[0])/float(self.dimensions_left[0])
		  x_ratio = float(self.dimensions_disp[1])/float(self.dimensions_left[1])
		  new_y1 = y1*y_ratio
		  new_y2 = y2*y_ratio
		  new_x1 = x1*x_ratio
		  new_x2 = x2*x_ratio

	      #cv2.imshow("Image window1", self.cv_image_left)
	      #cv2.rectangle(self.cv_image_left,(int(x1),int(y1)),(int(x2),int(y2)),(0,255,0),3)
	      #im_color = cv2.applyColorMap(self.disp_img, cv2.COLORMAP_JET)
	      #im_color = cv2.cvtColor(src, bwsrc, cv::COLOR_RGB2GRAY);
	      #cv2.imshow("Image window1", im_color)
	      #cv2.rectangle(self.disp_img,(int(new_x1),int(new_y1)),(int(new_x2),int(new_y2)),(0,255,0),3)
	      #cv2.waitKey(1)

	      #cv2.rectangle(self.cv_image_right,(int(new_x1),int(new_y1)),(int(new_x2),int(new_y2)),(0,255,0),3)
	   

		  crop_img = self.disp_img[new_y1:new_y2, new_x1:new_x2]
	          #img_resized = cv2.resize(crop_img,(int(50),int(200)))

		  depth,error = self.GetDepthMaxPeak(crop_img)
                  #depth,error = self.GMModel_ori(crop_img)
                  #depth,error = self.GMModel_resi(img_resized)
	  else:
		depth=-1
		error=0
      print(depth)
      self.pub2.publish(Float32MultiArray(data=[depth,error]))

    except CvBridgeError as e:
      print(e)
    if(depth!=-1):
                if (len(sys.argv)>1):
			with open('/home/nvidia/catkin_ws/error-statistics/%s.csv' % sys.argv[1], 'a') as f:	    			
				writer = csv.writer(f)
	    			writer.writerow([depth,float(error),rospy.get_time()-self.start_time])



def main(args):
  rospy.init_node('box_conversor', anonymous=True)
  bc = box_conversor()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
