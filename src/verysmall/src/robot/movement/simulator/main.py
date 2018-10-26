import sys
sys.path.append('../')
from simulator import Simulator
from functions.movement import Movement
from univector.un_field import univectorField
import cv2
import numpy as np
import random
sys.path.append('../../')
from utils.json_handler import JsonHandler

pathPIDList = '../../../parameters/PID.json'

ERROR = random.randint(0, 20)
jsonHandler = JsonHandler()
PIDLIST = jsonHandler.read(pathPIDList)
ATTACKERPID = PIDLIST['attacker']


# define when use the mouse
mouseMode = False

# univector
RADIUS = 3.48 # Distance from ball
KR = 1
K0 = 0.12
DMIN = 15
LDELTA = 50
RIGHT = 1
LEFT = 0

robotInitPosition = (200, 200)
ballInitPosition = (500, 500)
advRobotPosition = (350, 350)

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
# movement class
movement = Movement(ATTACKERPID, 10)

# obstacles
obstacle = np.array(advRobotPosition)
vObstacle = np.array([0, 0])

# creates the univector field
univetField = univectorField(atack_goal=RIGHT)
univetField.updateConstants(RADIUS, KR, K0, DMIN, LDELTA)
univetField.updateBall(np.array(sim.ball))
univetField.updateObstacles(obstacle, vObstacle)


def updateBallPosition(event,x,y,flags,param):
    if mouseMode:
        sim.drawBall((x, y))


def printInformation():
    print "**********************************************"
    print "         Initializing VSSS simulation         "
    print "**********************************************"
    print "Press any key to start:"
    print "To draw the ball in mouse position use: m"
    print "To exit the simulation press: q"


if __name__ == "__main__":

    printInformation()
    # show img
    cv2.imshow('Robot Simulation',img)
    cv2.moveWindow('Robot Simulation', 400,0)

    # define mouseCallBack
    cv2.setMouseCallback('Robot Simulation', updateBallPosition)

    key = cv2.waitKey(0)
    while 1:
        # avoid to clear the adversary
        sim.drawAdv(np.array(advRobotPosition))

        cv2.imshow('Robot Simulation',img)
        cv2.moveWindow('Robot Simulation', 400,0)

        vec = univetField.getVec(np.array(sim.robot), np.array([0,0]), np.array(sim.ball))

        # use mouse
        if key == ord('m'):
            mouseMode = not mouseMode
        # end simulation
        if key == ord('q'):
            cv2.destroyAllWindows()
            break

        leftSpeed, rightSpeed, done = movement.follow_vector(np.array(sim.robotVector), np.array(vec), 200)

        if not done:
            # move function
            sim.move(leftSpeed+ERROR, rightSpeed)

        # 60fps
        key = cv2.waitKey(16)