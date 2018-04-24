import cv2
import math
import numpy as np
from auxiliary import *


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
        """Draw the robot in position pos with the robotVector iqual a vec"""
        # angle between vec and [1,0]
        angle = angleBetween([1,0], vec)*180/(math.pi)
        # get the robot contour
        rect  = (pos, robotSize, angle)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        # update robot position
        self.robot = pos
        # update wheels and robot vector
        vec = unitVector(vec)
        self.upVec(vec)
        # draw contours and vec arrow
        cv2.drawContours(self.img,[box], 0, (0, 255, 0), -1)
        cv2.arrowedLine(self.img, self.robot, (int(self.robot[0]+32*vec[0]), int(self.robot[1]+32*vec[1])), (0,0,255), 2)

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
        self.robotVec = vec
        self.rightVec = self.robotVec
        self.leftVec = self.robotVec

    def move(self, lspeed, rspeed):
        """Recive left wheel speed and right wheel speed and draw the robot in the img"""
        maxv = max(lspeed, rspeed)
        # commum speed between the wheels
        difv = abs(abs(lspeed-rspeed)-maxv)
        self.robotVec = unitVector(self.robotVec)

        # resultant vector
        diff = abs(lspeed-rspeed)
        # angle in 1 frame
        angle = math.atan2(diff, 32)
        # clockwise or anti clockwise
        if rspeed == max(lspeed, rspeed):    
            auxVec = rotateVector(self.robotVec, angle)
        else:
            auxVec = rotateVector(self.robotVec, -angle)
        # clear robot position
        self.clearArea(self.robot)
        self.initArena()
        # draw robot
        self.drawRobot((int(self.robot[0]+difv*self.robotVec[0]), int(self.robot[1]+difv*self.robotVec[1])), auxVec)
