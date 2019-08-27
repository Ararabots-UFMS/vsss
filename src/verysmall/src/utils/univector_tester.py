from random import randint
import numpy as np
import time
from utils.new_univector import univector
from utils.json_handler import JsonHandler
from robot_module.movement.univector.un_field import UnivectorField
import cv2

def read_file():
    try:
        return np.load("univector_rand_points.npy")
    except IOError:
        content = []

        content = np.array([[[randint(0, 150), randint(0, 130)], [randint(0, 150), randint(0, 130)]] for _ in range(1000)])
        np.save("univector_rand_points.npy", content)
        
        return content

def cm2pixel(pos):
    posArray = np.array(pos)
    return 4*posArray

def drawPath(img, start, end, univetField, new_uni = False):
    currentPos = start
    _currentPos = cm2pixel(currentPos)

    newPos = None
    alpha = 2
    beta = 5

    t0 = time.time()

    while(np.linalg.norm(currentPos - end) >= beta):
        
        if new_uni:
            v = univector.generateUnivectorField(currentPos[0], currentPos[1], end, [[1000, 1000]])
        else:
            angle = univetField.get_angle_vec(currentPos,  np.array([0,0]), end, np.array([-1.0,0.0]))
            v = [np.cos(angle), np.sin(angle)]

        newPos = currentPos + (alpha*np.array(v))
        _newPos = cm2pixel(newPos).astype(int)

        if new_uni:
            cv2.line(img, (_currentPos[0], _currentPos[1]), (_newPos[0], _newPos[1]), (255,255,255), 3)
        else:
            cv2.line(img, (_currentPos[0], _currentPos[1]), (_newPos[0], _newPos[1]), (255,255,0), 3)

        cv2.imshow('field', img)
        cv2.waitKey(1)

        if (time.time() - t0 > 5):
            return False, newPos

        currentPos = newPos
        _currentPos = _newPos
    return True, None


if __name__ == "__main__":
    imgField = cv2.imread('../robot_module/movement/univector/img/vss-field.jpg')
     
    content = read_file()
    univector_list = JsonHandler().read("parameters/univector_constants.json")
    
    # Creates the univector field
    univetField = UnivectorField()
    RADIUS = univector_list['RADIUS']
    KR = univector_list['KR']
    K0 = univector_list['K0']
    DMIN = univector_list['DMIN']
    LDELTA = univector_list['LDELTA']
    univetField.update_constants(RADIUS, KR, K0, DMIN, LDELTA)
    univetField.update_obstacles([[1000, 1000]],[[0.0, 0.0]])

    for point in content:

        imgField2 = np.copy(imgField)

        drawPath(imgField2,point[0], point[1], univetField)
        drawPath(imgField2,point[0], point[1], None, True)
        
        #cv2.imshow('field', imgField2)
        cv2.waitKey(0)
