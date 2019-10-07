from random import randint
import numpy as np
import time
from utils.new_univector import univector
from utils.json_handler import JsonHandler
from robot_module.movement.univector.un_field import UnivectorField
from strategy.arena_utils import MAX_W_SIZE, MAX_H_SIZE
from robot_module.movement.univector.debug import enemyColor, teamColor
import cv2
from functools import partial

enemies = [[int(0.25 * MAX_W_SIZE), int(0.75 * MAX_H_SIZE)],
           [int(0.5 * MAX_W_SIZE), int(0.25 * MAX_H_SIZE)],
           [int(0.75 * MAX_W_SIZE), int(0.75 * MAX_H_SIZE)]
           ]
enemies_speed = np.array([[0, 0] for _ in range(len(enemies))])

global point_global, old_vector, juan_vector
juan_vector = old_vector = total_points = 0

def drawRobot(img, robotPos, enemy=False):
    if enemy:
        color = enemyColor
    else:
        color = teamColor

    pos = cm2pixel([robotPos[0], robotPos[1]])
    topLeft = (pos[0] - 15, pos[1] - 15)
    bottomRight = (pos[0] + 15, pos[1] + 15)
    cv2.rectangle(img, topLeft, bottomRight, color, -1)


def read_file():
    try:
        return np.load("univector_rand_points.npy")
    except IOError:
        content = []

        content = np.array(
            [[[randint(0, 150), randint(0, 130)], [randint(0, 150), randint(0, 130)]] for _ in range(10000)])
        np.save("univector_rand_points.npy", content)

        return content


def drawBall(img, ballPos):
    cv2.circle(img, (ballPos[0], ballPos[1]), 9, (0, 69, 255), -1)


def cm2pixel(pos):
    posArray = np.array(pos)
    return 4 * posArray


def drawPath(img, start, end, univetField, new_uni=False, should_draw=False):
    currentPos = start
    _currentPos = cm2pixel(currentPos)

    newPos = None
    alpha = 2
    beta = 5

    t0 = time.time()

    while np.linalg.norm(currentPos - end) >= beta:

        if new_uni:
            v = univector.generateUnivectorField(currentPos[0], currentPos[1], end, enemies)
        else:
            angle = univetField.get_angle_vec(currentPos, np.array([0, 0]), end, np.array([-1.0, 0.0]))
            v = [np.cos(angle), np.sin(angle)]

        newPos = currentPos + (alpha * np.array(v))
        _newPos = cm2pixel(newPos).astype(int)

        if should_draw:
            if new_uni:
                cv2.line(img, (_currentPos[0], _currentPos[1]), (_newPos[0], _newPos[1]), (255, 255, 255), 3)
            else:
                cv2.line(img, (_currentPos[0], _currentPos[1]), (_newPos[0], _newPos[1]), (255, 255, 0), 3)

            #cv2.waitKey(1)

        if (time.time() - t0) > 0.5:
            return False, newPos

        currentPos = newPos
        _currentPos = _newPos
    return True, None

def redraw(imagem_original, univetField, can_count = False, should_draw = True):

    global old_vector
    global juan_vector

    robo, bola = point_global

    imgField2 = np.copy(imagem_original)

    drawBall(imgField2, cm2pixel(bola))
    for enemy_pos in enemies:
        drawRobot(imgField2, enemy_pos, True)

    result, _ = drawPath(imgField2, robo, bola, univetField, False, should_draw)
    
    if can_count:
        old_vector += result

    result, _ = drawPath(imgField2, robo, bola, None, True, should_draw)
    
    if can_count:
        juan_vector += result

    if should_draw:
        cv2.imshow("Path", imgField2)
    #cv2.waitKey(1)

def callback(key, img, uni, scale, value):
    uni.__setattr__(key, value/scale)
    uni.update_constants(uni.RADIUS, uni.KR, uni.K0, uni.DMIN, uni.LDELTA)
    
    



if __name__ == "__main__":
    imgField = cv2.imread('../robot_module/movement/univector/img/vss-field.jpg')

    content = read_file()
    # content = np.array([[[100, 75], [int(0.05 * MAX_W_SIZE), int(0.75 * MAX_H_SIZE)]]]    
    univector_list = JsonHandler().read("parameters/univector_constants.json")

    # Creates the univector field
    univetField = UnivectorField()
    RADIUS = univector_list['RADIUS']
    KR = univector_list['KR']
    K0 = univector_list['K0']
    DMIN = univector_list['DMIN']
    LDELTA = univector_list['LDELTA']
    univetField.update_constants(RADIUS, KR, K0, DMIN, LDELTA)
    univetField.update_obstacles(enemies, enemies_speed)

    juan_vector = old_vector = total_points = 0
    should_draw = False

    if should_draw:
        range = 10000
        scale = 100
        cv2.namedWindow("Path", 1)
        cv2.imshow("Path", imgField)
        cv2.createTrackbar("RADIUS", "Path", int(RADIUS*scale), range, partial(callback,"RADIUS", imgField, univetField, scale))
        cv2.createTrackbar("KR", "Path", int(KR*scale), range, partial(callback,"KR", imgField, univetField, scale))
        cv2.createTrackbar("K0", "Path", int(K0*scale), range, partial(callback,"K0", imgField, univetField, scale))
        cv2.createTrackbar("DMIN", "Path", int(DMIN*scale), range, partial(callback,"DMIN", imgField, univetField, scale))
        cv2.createTrackbar("LDELTA", "Path", int(LDELTA*scale), range, partial(callback,"LDELTA", imgField, univetField, scale))
    
    point_global = None
    quero_mais = True
    old_vector = juan_vector = 0

    for point in content:
        if not quero_mais:
            break

        point_global = point
        redraw(imgField, univetField, True, should_draw)
        total_points += 1

        if total_points % 100 == 0:
            print("Old Vector Success Rate: " + str(old_vector / total_points))
            print("Juan Vector Success Rate: " + str(juan_vector / total_points))

        while quero_mais and should_draw:
            redraw(imgField, univetField, False, should_draw)
            key  = cv2.waitKey(int(1/60*1000)) & 0xFF
    
            if key == ord('q'): 
                cv2.destroyAllWindows()
                quero_mais = False

            if key == ord('s'):
                const = {"RADIUS": univetField.RADIUS, "KR": univetField.KR, "K0": univetField.K0, "DMIN": univetField.DMIN, "LDELTA": univetField.LDELTA}
                JsonHandler().write(const, "parameters/univector_constants.json")

            if key == 13:
                break


    print("Old Vector Success Rate: " + str(old_vector / total_points))
    print("Juan Vector Success Rate: " + str(juan_vector / total_points))
