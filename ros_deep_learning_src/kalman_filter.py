
class kalman_filter(object):
	def __init__(self,p,q,r,x):
		self.p=p
		self.q=q
		self.r=r		
		self.x=x
		self.k=0.0

	def getValue(self,x):
		self.p=self.p+self.q
		self.k=self.p/(self.p+self.r)
		self.x=self.x+self.k*(x-self.x)
		self.p=(1-self.k)*self.p
		return self.x
	
	def getX(self):
		return self.x
