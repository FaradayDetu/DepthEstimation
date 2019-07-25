import re
import math
import matplotlib.pyplot as plt
import numpy as np

class boundingBox:
	def __init__(self):
		self.x1=0.0
		self.x2=0.0
		self.y1=0.0
		self.y2=0.0
		self.confidence=0.0
		self.xc=0.0
		self.yc=0.0		
		self.w=0.0
		self.h=0.0
		self.area=0.0
		self.number=0
		self.prediction=1
                self.first_time = True

	def set(self,x):
		if(type(x)==str):		
			y=re.split(":",x)
			self.x1=float(y[1])
			self.y1=float(y[2])
			self.x2=float(y[3])
			self.y2=float(y[4])
			self.confidence=float(y[5])		
			self.w=self.x2-self.x1
			self.h=self.y2-self.y1
			self.area=self.h*self.w
			self.xc=(self.x2+self.x1)/2
			self.prediction=2
		else:
			self.x1=x.x1
			self.x2=x.x2
			self.y1=x.y1
			self.y2=x.y2
			self.confidence=x.confidence
			self.xc=x.xc
			self.yc=x.yc		
			self.w=x.w
			self.h=x.h
			self.area=x.area
			self.number=x.number
			self.prediction=x.prediction

	def getxc(self):
		return self.xc

	def getPrediction(self):
		return self.prediction

	def setPrediction(self,x):
		self.prediction=x

	def getArea(self):
		return self.area
	
	def getConfidence(self):
		return self.confidence

	def getCoordinates(self):
		return str(self.x1)+","+str(self.y1)+","+str(self.x2)+","+str(self.y2)
	
	def getHeight(self):
		return self.h
		
class carCharacteristics:
	def __init__(self):
		self.targetSteer=0
		self.targetThrottle=0
		self.actualSteer=0
		self.actualThrottle=0
                self.first_time = True

	def calculateMaxSteer(self,box):
		steer=int((box.getxc()-550)/5.5)
		steer=steer*(-1)
		if(steer>100):
			steer=100
		elif(steer<-100):
			steer=-100
		self.targetSteer=steer
	
	def calculateMaxThrottle(self,x):
		m=0.05
                q=15
                k=10
                a=65.5
		self.targetThrottle=int(q+(k/(1+np.exp(-m*(x-a)))))
                if self.first_time:
                    self.first_time = False
                    fig, ax = plt.subplots()
                    axis =  np.arange(0, 100.0, 0.01)
                    s = q + (k/(1+np.exp(-m*(axis-a))))
                    #ax.plot(axis,s)
                    #plt.show()

		if(self.actualThrottle<12):
			self.actualThrottle=12

	def setStopCharacteristics(self,x,y):
		self.targetThrottle=x
		self.actualThrottle=0
		#self.targetSteer=y
	
	def setSteer(self):
		if(self.targetSteer>self.actualSteer):
			if(self.actualSteer+15<=self.targetSteer):
				self.actualSteer+=15
			else:
				self.actualSteer=self.targetSteer
		elif(self.targetSteer<self.actualSteer):
			if(self.actualSteer-15>self.targetSteer):
				self.actualSteer-=15
			else:
				self.actualSteer=self.targetSteer
	
	def setThrottle(self,box):
		
		if(self.targetThrottle>self.actualThrottle):
			if(self.actualThrottle+1<=self.targetThrottle):
				self.actualThrottle+=1
			else:
				self.actualThrottle=self.targetThrottle
		elif(self.targetThrottle<self.actualThrottle):
			if(self.actualThrottle-1>self.targetThrottle):
				self.actualThrottle-=1
			else:
				self.actualThrottle=self.targetThrottle
	
	def changeTargetSteer(self,x):
		self.targetSteer+=x
		if(self.targetSteer<-100):	
			self.targerSteer=-100
		elif(self.targetSteer>100):
			self.targetSteer=100

	def changeTargetThrottle(self,x):
		self.targetThrottle+=x
		if(self.targetThrottle<-100):	
			self.targerThrottle=-100
		elif(self.targetThrottle>100):
			self.targetThrottle=100

	def changeThrottle(self,x):
		self.actualThrottle+=x
	
	def changeSteer(self,x):
		self.actualSteer=x

	def stop(self):
		self.targetThrottle=0
		self.actualThrottle=0

	def getDetails(self):
		return str(self.targetSteer)+","+str(self.targetThrottle)+","+str(self.actualSteer)+","+str(self.actualThrottle)
	
	def getActualSteer(self):
		return self.actualSteer

	def getActualThrottle(self):
		return self.actualThrottle
		
		  

