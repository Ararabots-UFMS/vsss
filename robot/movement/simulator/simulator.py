import cv2
from auxiliary import *

# minimum speed 44
# max speed 255
# arena size 520X600 

# top-left
TL = (100,40)
# top-right
TR = (700,40)
# bottom-left
BL = (100,560)
# bottom-right 
BR = (700,560)

# robot size in pixels 
robotSize = (32, 32)
# ball radius in pixels
ballRadius = 8

class Simulator():

    def __init__(self, img):
        self.img = img
        # Update the wheels vector
        self.upVec([1,0])

        # robot 32X32
        # robot center
        self.robot = (0,0)
        # ball center
        self.ball = (0,0)


    def drawArena(self):
        """Draw the arena"""
        # horizontal lines
        cv2.line(self.img,TL,TR,(255,255,255),1)
        cv2.line(self.img,BL,BR,(255,255,255),1)
        # vertical lines
        cv2.line(self.img,TL,BL,(255,255,255),1)
        cv2.line(self.img,TR,BR,(255,255,255),1)
        # left side goal
        cv2.line(self.img,(TL[0]-40, TL[1]+180),(TL[0]-40, TL[1]+340),(255,255,255),1)
        cv2.line(self.img,(TL[0]-40, TL[1]+180),(TL[0], TL[1]+180),(255,255,255),1)
        cv2.line(self.img,(TL[0]-40, TL[1]+340),(TL[0], TL[1]+340),(255,255,255),1)
        # right side goal
        cv2.line(self.img,(TR[0]+40, TR[1]+180),(TR[0]+40, TR[1]+340),(255,255,255),1)
        cv2.line(self.img,(TR[0]+40, TR[1]+180),(TR[0], TR[1]+180),(255,255,255),1)
        cv2.line(self.img,(TR[0]+40, TR[1]+340),(TR[0], TR[1]+340),(255,255,255),1)

    def drawMarks(self):
        """Draw the center ball mark"""
        cv2.line(self.img,(400,40),(400,560),(255,255,255),1)
        cv2.circle(self.img,(400,300), 20, (255,255,255), 1)

    def clearArea(self, pos):
        """Clear a square in position pos of the arena"""
        size = (70, 70)
        rect  = (pos, size, 0)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(self.img,[box], 0, (0, 0, 0), -1)

    def drawRobot(self, pos, vec):
        """Draw the robot in position pos with the robotVectortor iqual a vec"""
        # angle between vec and [1,0]
        angle = angleBetween([1,0], vec)*180/(math.pi)
        # get the robot contour
        rect  = (pos, robotSize, angle)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        if self.arenaLimit(pos):
            # update robot position
            self.robot = pos
            # update wheels and robot vector
            vec = unitVector(vec)
            self.upVec(vec)
        # draw contours and vec arrow
        cv2.drawContours(self.img, [box], 0, (25, 25, 25), -1)
        cv2.arrowedLine(self.img, (int(self.robot[0]+12*vec[0]), int(self.robot[1]+12*vec[1])), (int(self.robot[0]+32*vec[0]), int(self.robot[1]+32*vec[1])), (0,0,255), 2)
        cv2.circle(self.img,(int(self.robot[0]-6*vec[0]), int(self.robot[1]-6*vec[1])), 8, (0,255,255), -1)
        cv2.circle(self.img,(int(self.robot[0]+12*vec[0]), int(self.robot[1]+12*vec[1])), 4, (0,255,255), -1)
        # cv2.circle(self.img,(int(self.robot[0]+9*vec[0]), int(self.robot[1]+8*vec[1])), 4, (0,255,255), -1)

    def drawBall(self, pos):
        """Draw ball at position pos"""
        self.ball = pos
        cv2.circle(self.img, self.ball, ballRadius, (31,136,246), -1)

    def initArena(self):
        """Initilize arena and ball"""
        self.drawArena()
        self.drawMarks()
        self.drawBall((500,500))

    def upVec(self, vec):
        """Update robot and wheels vector"""
        self.robotVector = vec
        self.rightVec = self.robotVector
        self.leftVec = self.robotVector

    def move(self, left, right):
        """Recive left wheel speed and right wheel speed and draw the robot in the img"""
        lspeed = int(left*12.0/255.0)
        rspeed = int(right*12.0/255.0)

        backwards = False
        if lspeed < 0 and rspeed < 0:
            backwards = True

        maxv = maxAbs(lspeed, rspeed)

        # commum speed between the wheels
        difv = abs(abs(lspeed-rspeed)-maxv)

        self.robotVector = unitVector(self.robotVector)
        # resultant vector
        diff = abs(lspeed-rspeed)
        # angle in 1 frame
        angle = math.atan2(diff, 32)

        # counterclockwise
        if (rspeed == maxAbs(lspeed, rspeed) and backwards == False) or (lspeed == maxAbs(lspeed, rspeed) and backwards == True):
            auxVec = rotateVector(self.robotVector, angle)
        else:#clockwise
            auxVec = rotateVector(self.robotVector, -angle)

        # clear robot position
        self.clearArea(self.robot)
        self.initArena()

        # draw robot
        if not backwards:
            self.drawRobot((int(self.robot[0]+difv*self.robotVector[0]), int(self.robot[1]+difv*self.robotVector[1])), auxVec)
        else:
            self.drawRobot((int(self.robot[0] - difv * self.robotVector[1]), int(self.robot[1] - difv * self.robotVector[0])), auxVec)
    def arenaLimit(self, pos):
        """Define the arena limits"""
        if pos[0] > 684 or pos[0] < 116:
            return False
        if pos[1] < 56 or pos[1] > 544:
            return False
        return True
