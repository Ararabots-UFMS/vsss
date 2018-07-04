import sys
sys.path.append('../')
from simulator import Simulator
from functions.movement import Movement
from univector.un_field import univectorField
import cv2
import numpy as np

# univector
RADIUS = 3.48
KR = 4.15
K0 = 0.12
DMIN = 3.48
# LDELTA = 4.5
LDELTA = 50

robotInitPosition = (200, 200)
ballInitPosition = (500, 500)
advRobotPosition = (350, 350)

if __name__ == "__main__":
    # window size
    img = np.zeros((600,800,3), np.uint8)
    # creating simulator
    sim = Simulator(img)
    # initialize arena
    sim.initArena()
    # initialize robot
    sim.drawRobot(robotInitPosition, [-1,1])
    # initialize ball
    sim.drawBall(ballInitPosition)
    # initialize adversary robot
    sim.drawAdv(np.array(advRobotPosition))

    # Classe de movimentacao
    movement = Movement(10)

    # Obstacles
    obstacle = np.array(advRobotPosition)
    vObstacle = np.array([0, 0])

    # Creates the univector field
    univetField = univectorField()
    univetField.updateConstants(RADIUS, KR, K0, DMIN, LDELTA)
    univetField.updateBall(np.array(sim.ball))
    univetField.updateObstacles(obstacle, vObstacle)


    # show img
    cv2.imshow('Robot Simulation',img)
    cv2.moveWindow('Robot Simulation', 400,0)

    key = cv2.waitKey(0)
    while 1:
        cv2.imshow('Robot Simulation',img)
        cv2.moveWindow('Robot Simulation', 400,0)

        vec = univetField.getVec(np.array(sim.robot), np.array([0,0]), np.array(sim.ball))

        #End simulation
        if key == ord('q'):
            cv2.destroyAllWindows()
            break

        leftSpeed, rightSpeed, done = movement.followVector(np.array(sim.robotVector) ,np.array(vec), 200)
        sim.throwBall([-1, -2], 100)

        if not done:
            # move function
            sim.move(leftSpeed,rightSpeed)
        # 60fps
        key = cv2.waitKey(16)
