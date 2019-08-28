from random import randint
import numpy as np
import time
from utils.new_univector import univector
from utils.json_handler import JsonHandler
from robot_module.movement.univector.un_field import UnivectorField
from strategy.arena_utils import MAX_W_SIZE, MAX_H_SIZE
from robot_module.movement.univector.debug import enemyColor, teamColor
import cv2

enemies = [[int(0.25 * MAX_W_SIZE), int(0.75 * MAX_H_SIZE)],
           [int(0.5 * MAX_W_SIZE), int(0.25 * MAX_H_SIZE)],
           [int(0.75 * MAX_W_SIZE), int(0.75 * MAX_H_SIZE)]
           ]
enemies_speed = np.array([[0, 0] for _ in range(len(enemies))])


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

            cv2.imshow('field', img)
            cv2.waitKey(1)

        if (time.time() - t0) > 0.5:
            return False, newPos

        currentPos = newPos
        _currentPos = _newPos
    return True, None


if __name__ == "__main__":
    imgField = cv2.imread('../robot_module/movement/univector/img/vss-field.jpg')

    content = read_file()
    # content = np.array([[[100, 75], [int(0.05 * MAX_W_SIZE), int(0.75 * MAX_H_SIZE)]]])
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

    for point in content:
        imgField2 = np.copy(imgField)

        if should_draw:
            drawBall(imgField2, cm2pixel(point[1]))
            for enemy_pos in enemies:
                drawRobot(imgField2, enemy_pos, True)

        result, _ = drawPath(imgField2, point[0], point[1], univetField, False, should_draw)
        old_vector += result

        result, _ = drawPath(imgField2, point[0], point[1], None, True, should_draw)
        juan_vector += result

        # cv2.imshow('field', imgField2)
        if should_draw:
            cv2.waitKey(0)

        total_points += 1

        if total_points % 100 == 0:
            print("Old Vector Success Rate: " + str(old_vector / total_points))
            print("Juan Vector Success Rate: " + str(juan_vector / total_points))

    print("Old Vector Success Rate: " + str(old_vector / total_points))
    print("Juan Vector Success Rate: " + str(juan_vector / total_points))
