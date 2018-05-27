import sys
sys.path.append('../')
from simulator import Simulator
from functions.movement import Movement
import cv2
import numpy as np

if __name__ == "__main__":
    # window size
    img = np.zeros((600,800,3), np.uint8)
    # creating simulator
    sim = Simulator(img)
    # initialize arena
    sim.initArena()
    # initialize robot
    sim.drawRobot((200,200), [-1,1])

    # Classe de movimentacao
    movement = Movement(10)

    # show img
    cv2.imshow('Robot Simulation',img)
    cv2.moveWindow('Robot Simulation', 400,0)

    key = cv2.waitKey(0)
    while 1:
        cv2.imshow('Robot Simulation',img)
        cv2.moveWindow('Robot Simulation', 400,0)

        #End simulation
        if key == ord('q'):
            cv2.destroyAllWindows()
            break

        leftSpeed, rightSpeed, done = movement.moveToPoint(np.array(sim.robot) ,np.array(sim.robotVector), np.array(sim.ball), -200)

        if not done:
            # move function
            sim.move(leftSpeed,rightSpeed)
        # 60fps
        key = cv2.waitKey(16)
