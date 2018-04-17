import time

"""Pid class"""

class Pid:

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, maxIntegral=1000.0, maxDerivative=1000.0, target=0.0):
        # Constants 
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Max integral and derivative value
        self.maxIntegral = maxIntegral 
        self.maxDerivative = maxDerivative

        # Derivative e integral value
        self.derivative = 0.0
        self.integral = 0.0

        #SetPoint
        self.target = target
        self.error = 0.0

        self.lastTime = 


    def setKp(self, num):
        self.kp = num

    def setKi(self, num):
        self.ki = num

    def setKd(self, num):
        self.kd = num

    def setTarget(self, num):
        self.setTarget = setTarget

    def getConstants(self):
        return self.kp, self.ki, self.kd

    def update(self, value):
        """Update the error value and return it"""
        self.error = self.target - value
        deltaTime = 

        integral = self.integral + (self.error*)
