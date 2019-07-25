import maestro

class carController:

	def __init__(self):
		self.servo = maestro.Controller()

		self.MAX_STEER = 1900
		self.MIN_STEER = 1100
		self.MID_STEER = 1500

		self.MAX_THROT = 1900
		self.MIN_THROT = 1100
		self.MID_THROT = 1500

		self.STEER_CHANNEL = 0
		self.THROT_CHANNEL = 1

	def usToQs(self, signal):
	    if ( (signal > 1900) or (signal < 1100) ):
	        return (6000)
	    else:    
	        return (signal * 4)

	def check(self,s,t):
		if (s > self.MAX_STEER):
			s = self.MAX_STEER

		if (s < self.MIN_STEER):
			s = self.MIN_STEER

		if (t > self.MAX_THROT):
			t = self.MAX_THROT

		if (t < self.MIN_THROT):
			t = self.MIN_THROT
		return(s,t)

	def parseSteering(self, value):

		pwm_signal = 1500
		pwm_float  = 1500.0

	    # Check out of range
		if ( (value < -100) ):
			return self.MAX_STEER

		if ( (value > 100 ) ):
			return self.MIN_STEER

		if (value == 0):
			return self.MID_STEER
		else:
			if (value < 0):
				pwm_float =  float(self.MID_STEER) + float( (-1) * value * float( float(self.MAX_STEER - self.MID_STEER) / 100.0 ))
				return( int(pwm_float))
			else:
				pwm_signal = self.MID_STEER - ( value * (  self.MID_STEER - self.MIN_STEER) / 100) 
			return pwm_signal

	def parseThrottle(self, value):
		pwm_signal = 1500

	    # Check out of range
		if ( (value < -100) ):
			return self.MIN_THROT

		if ( (value > 100 ) ):
			return self.MAX_THROT

		if (value == 0):
			return self.MID_THROT

		else:
			pwm_signal = 1500 + (value * 4)
			return pwm_signal

	def set(self, steering, throttle):		

		s = self.parseSteering(steering)
		t = self.parseThrottle(throttle)

		self.servo.setTarget(self.STEER_CHANNEL, self.usToQs(s))
		self.servo.setTarget(self.THROT_CHANNEL, self.usToQs(t))

		print "SET: (",steering,", ",throttle,") --> [",s,",",t,"]"
		print " "

	def close(self):
		self.servo.close()
