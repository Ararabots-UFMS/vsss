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

        if any(self.lastPos != goalPosition):#TODO: talvez seja melhor o pid resolver isso
            self.pid.reset()
        directionVector = goalPosition - robotPosition
        diffAngle = angleBetween(robotVector, directionVector)
        correction = self.pid.update(diffAngle)
        if correction > 0: #correcao aplicada na roda X
            return int(speed), int(speed+correction), False
        return int(speed+correction), int(speed), False


