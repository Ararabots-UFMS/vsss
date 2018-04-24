from auxiliary import *
from simulator import Simulator
import cv2
import math
import numpy as np

if __name__ == "__main__":
    # window size
    # arena size 520X600 
    img = np.zeros((600,800,3), np.uint8)

    sim = Simulator(img)
    sim.initArena()
    sim.drawRobot((200,200), [-1,-1])
    cv2.imshow('Goalkeeper Simulation',img)
    key = cv2.waitKey(0)