import sys
import numpy as np
sys.path.append('../')
from control.PID import PID
from auxiliary import angleBetween, distancePoints

class Movement():

    def __init__(self, error):
        self.pid = PID(kp=40.0, ki=0.0, kd=0.0)
        self.lastPos = np.array([0, 0])
        self.errorMargin = error

    def inGoal(self, robotPosition, goalPosition):
        if distancePoints(robotPosition, goalPosition) <= self.errorMargin:
            return True
        return False

    def moveToPoint(self, robotPosition, robotVector, goalPosition, speed):
        if self.inGoal(robotPosition, goalPosition):
            return 0, 0, True
        if any(self.lastPos != goalPosition):
            self.pid.reset()
        directionVector = goalPosition - robotPosition
        return self.followVector(robotVector, directionVector, speed) 

    def followVector(self, robotVector, goalVector, speed):         
        diffAngle = angleBetween(robotVector, goalVector, ccw=False) 
        correction = self.pid.update(diffAngle) 
        if correction > 0: 
            return int(speed), int(speed-correction), False 
        return int(speed+correction), int(speed), False

    def spin(self, speed, ccw=True):
        if ccw:
            return int(-speed), int(speed), False
        return int(speed), int(-speed), False

    def headTo(self, robotVector, goalVector, speed):
        pass


