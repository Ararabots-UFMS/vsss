from auxiliary import *
from simulator import Simulator
import cv2
import math
import time
import numpy as np

if __name__ == "__main__":
    # window size
    img = np.zeros((600,800,3), np.uint8)

    # creating simulator
    sim = Simulator(img)
    # initialize arena
    sim.initArena()
    # initialize robot
    sim.drawRobot((200,200), [-1,0])

    # show img
    cv2.imshow('Goalkeeper Simulation',img)
    key = cv2.waitKey(1)
    while 1:
        cv2.imshow('Goalkeeper Simulation',img)

        #End simulation
        if key == ord('q'):
            cv2.destroyAllWindows()
            break

        # move function 
        sim.move(80,44)
        # 60fps
        key = cv2.waitKey(16)