from enum import Enum
from numpy import array
import rospy

X = 0
Y = 1
SQUARE_SIDE = 15.0 * 2 ** 0.5
HALF_SQUARE_SIDE = SQUARE_SIDE / 2

LEFT = 0
RIGHT = 1

HALF_ARENA_HEIGHT = 65
HALF_ARENA_WIDTH = 75
MAX_H_SIZE = 130
MAX_W_SIZE = 150

LEFT_GOAL_LINE = 16
RIGHT_GOAL_LINE = 134

LEFT_AREA_CENTER_X = 8
RIGHT_AREA_CENTER_X = 142

BORDER_THRESHOLD = 15


class ArenaSections(Enum):
    LEFT_GOAL_AREA = 0
    RIGHT_GOAL_AREA = 1

    LEFT_GOAL = 2
    RIGHT_GOAL = 3
    LEFT_UP_CORNER = 4
    LEFT_DOWN_CORNER = 5
    RIGHT_UP_CORNER = 6
    RIGHT_DOWN_CORNER = 7

    UP_BORDER = 8
    DOWN_BORDER = 9
    CENTER = 10

    LEFT_DOWN_BOTTOM_LINE = 11
    LEFT_UP_BOTTOM_LINE = 12
    RIGHT_DOWN_BOTTOM_LINE = 13
    RIGHT_UP_BOTTOM_LINE = 14

    LEFT_CRITICAL_LINE = 15
    RIGHT_CRITICAL_LINE = 16


Axis = [
    array([1.0, 0.0]),  # LEFT_GOAL_AREA
    array([1.0, 0.0]),  # RIGHT_GOAL_AREA

    array([1.0, 0.0]),  # LEFT_GOAL
    array([1.0, 0.0]),  # RIGHT_GOAL
    array([0.0, 1.0]),  # LEFT_UP_CORNER
    array([0.0, -1.0]),  # LEFT_DOWN_CORNER
    array([0.0, -1.0]),  # RIGHT_UP_CORNER
    array([0.0, 1.0]),  # RIGHT_DOWN_CORNER

    array([1.0, 0.0]),  # UP_BORDER
    array([1.0, 0.0]),  # DOWN_BORDER
    array([1.0, 0.0]),  # CENTER

    array([0.0, -1.0]),  # LEFT_DOWN_BOTTOM_LINE
    array([0.0, 1.0]),  # LEFT_UP_BOTTOM_LINE
    array([0.0, 1.0]),  # RIGHT_DOWN_BOTTOM_LINE
    array([0.0, -1.0])  # RIGHT_UP_BOTTOM_LINE
]

Offsets = [
    array([0.0, 0.0]),  # LEFT_GOAL_AREA
    array([0.0, 0.0]),  # RIGHT_GOAL_AREA

    array([0.0, 0.0]),  # LEFT_GOAL
    array([0.0, 0.0]),  # RIGHT_GOALGoToPosition
    array([1.0, -1.0]),  # LEFT_UP_CORNERGoToPosition
    array([1.0, 1.0]),  # LEFT_DOWN_CORNERGoToPosition
    array([-1.0, -1.0]),  # RIGHT_UP_CORNERGoToPosition
    array([-1.0, 1.0]),  # RIGHT_DOWN_CORNER

    array([0.0, -2.0]),  # UP_BORDER
    array([0.0, 2.0]),  # DOWN_BORDER
    array([0.0, 0.0]),  # CENTER

    array([1.0, 0.0]),  # LEFT_DOWN_BOTTOM_LINE
    array([1.0, 0.0]),  # LEFT_UP_BOTTOM_LINE
    array([-1.0, 0.0]),  # RIGHT_DOWN_BOTTOM_LINE
    array([-1.0, 0.0])  # RIGHT_UP_BOTTOM_LINE
]

BORDER_NORMALS = {4: [1.0, -1.0], 5: [1.0, 1.0], 6: [-1.0, -1.0], 7: [-1.0, 1.0], 8: [0.0, 1.0], 9: [0.0, -1.0],
                  11: [1.0, 0.0],
                  12: [1.0, 0.0], 13: [-1.0, 0.0], 14: [-1.0, 0.0]}

BALL_SIZE = 4
ROBOT_SIZE = 7


def on_attack_side(pos, team_side, bias=0):
    """
   Verify if the object is in attack side (True) or in defense side(False)

    :param bias:
   :param pos: np.array([x, y])
   :param team_side: int 0 ou 1
   :return: bool
   """
    return (pos[0] > (75 - bias) and team_side == LEFT) or (pos[0] < (75 + bias) and team_side == RIGHT)


def y_axis_section(pos):
    return pos[1] > HALF_ARENA_HEIGHT


def on_extended_attack_side(pos, team_side):
    """
    Verify if the object is in attack side plus a quarter of field (True) or in defense side(False)

    :param pos: np.array([x, y])
    :param team_side: int 0 ou 1
    :return: bool
    """
    return (pos[0] > 55 and team_side == LEFT) or (pos[0] < 95 and team_side == RIGHT)


def inside_range(a, b, num):
    """
    Verify if num is inside the range

    :param a:
    :param b:
    :param num:
    :return:
    """
    return a <= num <= b or b <= num <= a


def inside_rectangle(a, b, point):
    """
    Verify if point is inside rectangle made by opposite points a and b
    :param point:
    :param a: first point that composes the square
    :param b: second point that composes the square
    :return: return true or false
    """

    return inside_range(a[X], b[X], point[X]) and inside_range(a[Y], b[Y], point[Y])


def section(pos):
    """
    Returns the section of the given object
    :param pos: np.array([x, y])
    :return: int
    """
    # Goal area
    if inside_rectangle((0, 30), (15, 100), pos):
        return ArenaSections.LEFT_GOAL_AREA
    elif inside_rectangle((135, 30), (150, 100), pos):
        return ArenaSections.RIGHT_GOAL_AREA
    elif inside_range(-10, -0.1, pos[X]):
        return ArenaSections.LEFT_GOAL
    elif inside_range(150.1, 160, pos[X]):
        return ArenaSections.RIGHT_GOAL
    # Corners definition
    elif inside_rectangle((0, 0), (SQUARE_SIDE, SQUARE_SIDE), pos):
        return ArenaSections.LEFT_DOWN_CORNER
    elif inside_rectangle((0, 130 - SQUARE_SIDE), (SQUARE_SIDE, 130), pos):
        return ArenaSections.LEFT_UP_CORNER
    elif inside_rectangle((150 - SQUARE_SIDE, 0), (150, SQUARE_SIDE), pos):
        return ArenaSections.RIGHT_DOWN_CORNER
    elif inside_rectangle((150 - SQUARE_SIDE, 130 - SQUARE_SIDE), (150, 130), pos):
        return ArenaSections.RIGHT_UP_CORNER
    elif inside_rectangle((0, HALF_SQUARE_SIDE), (15, 30), pos):
        # Bottom line
        return ArenaSections.LEFT_DOWN_BOTTOM_LINE
    elif inside_rectangle((0, 100), (15, 130 - HALF_SQUARE_SIDE), pos):
        return ArenaSections.LEFT_UP_BOTTOM_LINE
    elif inside_rectangle((150 - HALF_SQUARE_SIDE, HALF_SQUARE_SIDE), (150, 30), pos):
        return ArenaSections.RIGHT_DOWN_BOTTOM_LINE
    elif inside_rectangle((135, 100), (150, 130 - HALF_SQUARE_SIDE), pos):
        return ArenaSections.RIGHT_UP_BOTTOM_LINE
    # Border
    elif inside_rectangle((15, 130), (120, 115), pos):
        return ArenaSections.UP_BORDER
    elif inside_rectangle((15, 0), (120, 15), pos):
        return ArenaSections.DOWN_BORDER
    else:
        return ArenaSections.CENTER


def univector_pos_section(pos):
    """
    Returns the section of the given object
    :param pos: np.array([x, y])
    :return: int
    """

    if pos[0] > MAX_W_SIZE - 10 or pos[0] < 10:
        side = pos[0] > MAX_W_SIZE - 10

        if pos[1] < 10:
            return ArenaSections.LEFT_DOWN_CORNER if side == LEFT else ArenaSections.RIGHT_DOWN_CORNER
        elif pos[1] < 45:
            return ArenaSections.LEFT_DOWN_BOTTOM_LINE if side == LEFT else ArenaSections.RIGHT_DOWN_BOTTOM_LINE
        elif pos[1] < 85:
            return ArenaSections.LEFT_GOAL if side == LEFT else ArenaSections.RIGHT_GOAL
        elif pos[1] < 120:
            return ArenaSections.LEFT_UP_BOTTOM_LINE if side == LEFT else ArenaSections.RIGHT_UP_BOTTOM_LINE
        else:
            return ArenaSections.LEFT_UP_CORNER if side == LEFT else ArenaSections.RIGHT_UP_CORNER

    else:
        if pos[1] > MAX_H_SIZE - BORDER_THRESHOLD:
            return ArenaSections.UP_BORDER
        elif pos[1] < BORDER_THRESHOLD:
            return ArenaSections.DOWN_BORDER
        else:
            return ArenaSections.CENTER


def side_section(pos, team_side):
    """
    Return Attack_side and section of the object
    :param team_side: bool
    :param pos: np.array([x, y])
    :return: (boolen, int)
    """

    return on_attack_side(pos, team_side), section(pos)


def goal_position(team_side):
    """
    Return the position of the goal, given attacking side  and section of the object
    :param team_side: int
    :return: np.array([x,y])
    """

    if team_side == LEFT:
        return array([150, 65])
    return array([0, 65])
