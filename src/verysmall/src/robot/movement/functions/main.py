import numpy as np
from movement import Movement
robotPosition = np.array([0, 0])
robotVector = np.array([1, 1])
goalPosition = np.array([20, 0])
speed = 5
movement = Movement(10)
print movement.moveToPoint(robotPosition, robotVector, goalPosition, speed)