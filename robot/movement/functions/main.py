"""Teste movment class"""

import numpy as np
from movement import Movement
# set robot position
robotPosition = np.array([0, 0])
# set robot vector
robotVector = np.array([1, 1])
# set goal position
goalPosition = np.array([20, 0])
# define speed
speed = 5
# instantiation class
movement = Movement(10)
# print the results
print movement.moveToPoint(robotPosition, robotVector, goalPosition, speed)