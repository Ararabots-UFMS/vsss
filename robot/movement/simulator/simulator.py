import cv2
import math
import numpy as np
from auxiliary import *

# top-left
TL = (100,40)
# top-right
TR = (700,40)
# bottom-left
BL = (100,560)
# bottom-right 
BR = (700,560)

# robot 32X32
# robot center
robot = (0,0)
robotSize = (32, 32)

# ball center
ball = (0,0)
ballRadius = 8

class Simulator():

    def __init__(self, img):
        self.img = img
        self.robotVec = [1,0]
        self.upWheelsVec()

    def drawArena(self):
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
        cv2.line(self.img,(400,40),(400,560),(255,255,255),1)
        cv2.circle(self.img,(400,300), 20, (255,255,255), 1)

    def drawRobot(self, pos, vec):
        angle = angleBetween(self.robotVec, vec)*180/(math.pi)
        robot = pos
        rect  = (robot, robotSize, angle)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(self.img,[box], 0, (0, 255, 0), -1)
        cv2.arrowedLine(self.img, robot, (robot[0]+32*vec[0], robot[1]+32*vec[1]), (0,0,255), 2)

    def drawBall(self, pos):
        ball = pos
        cv2.circle(self.img, ball, ballRadius, (31,136,246), -1)

    def initArena(self):
        self.drawArena()
        self.drawMarks()
        self.drawBall((500,500))

    def upWheelsVec(self):
        self.rightVec = self.robotVec
        self.leftVec = self.robotVec