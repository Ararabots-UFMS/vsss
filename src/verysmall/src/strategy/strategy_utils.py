import numpy as np
from utils import math_utils
import math
CW = 0
CCW = 1

X = 0
Y = 1
SQUARE_SIDE = 15.0 * 2 ** 0.5
HALF_SQUARE_SIDE = SQUARE_SIDE/2

LEFT = 0
RIGHT = 1

LEFT_GOAL_AREA    =  0
RIGHT_GOAL_AREA   =  1

LEFT_GOAL         =  2
RIGHT_GOAL        =  3
LEFT_UP_CORNER    =  4
LEFT_DOWN_CORNER  =  5
RIGHT_UP_CORNER   =  6
RIGHT_DOWN_CORNER =  7

UP_BORDER         =  8
DOWN_BORDER       =  9
CENTER            =  10

LEFT_DOWN_BOTTOM_LINE  = 11
LEFT_UP_BOTTOM_LINE    = 12
RIGHT_DOWN_BOTTOM_LINE = 13
RIGHT_UP_BOTTOM_LINE   = 14

BALL_SIZE = 4
ROBOT_SIZE = 7

BORDER_NORMALS = {4:[1.0,-1.0], 5:[1.0,1.0], 6:[-1.0,-1.0], 7:[-1.0,1.0], 8:[0.0,1.0], 9:[0.0,-1.0], 11:[1.0,0.0],
                  12:[1.0,0.0], 13:[-1.0,0.0], 14:[-1.0,0.0]}

############################
# arena sections functions #
############################
def on_attack_side(pos, team_side):
   """
   Verify if the object is in attack side (True) or in defense side(False)

   :param pos: np.array([x, y])
   :param team_side: int 0 ou 1
   :return: bool
   """
   return (pos[0] > 75 and team_side == LEFT) or (pos[0] < 75 and team_side == RIGHT)

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
    '''
    Verify if point is inside rectangle made by opposite points a and b

    :param a: first point that composes the square
    :param b: second point that composes the square
    :param x: point to be verified
    :return: return true or false
    '''

    return inside_range(a[X], b[X], point[X]) and inside_range(a[Y], b[Y], point[Y])

def section(pos):
    """
    Returns the section of the given object

    :param pos: np.array([x, y])
    :return: int
    """
    #Goal area
    if inside_rectangle((0,30), (15,100), pos):
        return LEFT_GOAL_AREA
    elif inside_rectangle((135, 30), (150, 100), pos):
        return RIGHT_GOAL_AREA
    elif inside_range(-10,-0.1,pos[X]):
        return LEFT_GOAL
    elif inside_range(150.1,160,pos[X]):
        return RIGHT_GOAL
    # Corners definition
    elif inside_rectangle((0, 0), (SQUARE_SIDE, SQUARE_SIDE), pos):
        return LEFT_DOWN_CORNER
    elif inside_rectangle((0, 130 - SQUARE_SIDE), (SQUARE_SIDE, 130), pos):
        return LEFT_UP_CORNER
    elif inside_rectangle((150 - SQUARE_SIDE, 0), (150, SQUARE_SIDE), pos):
        return RIGHT_DOWN_CORNER
    elif inside_rectangle((150 - SQUARE_SIDE, 130 - SQUARE_SIDE), (150, 130), pos):
        return RIGHT_UP_CORNER
    elif inside_rectangle((0, HALF_SQUARE_SIDE), (15,30), pos):
    #Bottom line
        return LEFT_DOWN_BOTTOM_LINE
    elif inside_rectangle((0,100), (15,130-HALF_SQUARE_SIDE), pos):
        return LEFT_UP_BOTTOM_LINE
    elif inside_rectangle((150-HALF_SQUARE_SIDE,HALF_SQUARE_SIDE), (150,30), pos):
        return RIGHT_DOWN_BOTTOM_LINE
    elif inside_rectangle((135,100), (150,130-HALF_SQUARE_SIDE), pos):
        return RIGHT_UP_BOTTOM_LINE
    #Border
    elif inside_range(130 - 12, 130, pos[Y]):
        return UP_BORDER
    elif inside_range(0, 12, pos[Y]):
        return DOWN_BORDER
    else:
        return CENTER

def extended_area(pos, team_side):
    if inside_rectangle((0,25), (20,105), pos):
        return LEFT_GOAL_AREA
    elif inside_rectangle((130, 25), (150, 105), pos):
        return RIGHT_GOAL_AREA
    elif inside_range(-10,-0.1,pos[X]):
        return LEFT_GOAL
    elif inside_range(150.1,160,pos[X]):
        return RIGHT_GOAL
    return CENTER

def side_section(pos,team_side):
    """
    Return Attack_side and section of the object
    :param pos: np.array([x, y])
    :return: (boolen, int)
    """

    return (on_attack_side(pos,team_side), section(pos))

def goal_position(team_side):
    """
    Return the position of the goal, given attacking side  and section of the object
    :param team_side: int
    :return: np.array([x,y])
    """

    if team_side == LEFT:
        return np.array([150, 65])
    return np.array([0, 65])

######################
# distance functions #
######################

def distance_point(position_one, position_two):
    return np.linalg.norm(position_one-position_two)

def near_ball(ball_position, robot_position, _distance = 9.5):
    """
    Returns if the robot is near to the ball
    :params ball_position: np.array([x,y])
    :params robot_position: np.array([x,y])
    :return: boolean
    """
    distance = distance_point(ball_position, robot_position)
    return (distance <= _distance)

def behind_ball(ball_position, robot_position, team_side):
    """
    Returns if the robot is behind to the ball
    :params ball_position: np.array([x,y])
    :params robot_position: np.array([x,y])
    :params team_side: int
    :return: boolean
    """
    if (team_side == LEFT):
        if (near_ball(ball_position, robot_position)):
            return robot_position[0] < ball_position[0]
    else:
        if (near_ball(ball_position, robot_position)):
            return robot_position[0] > ball_position[0]
    return False

def spin_direction(ball_position, robot_position, team_side):
    """
    Returns the direction of the spin
    :params ball_position: np.array([x,y])
    :params robot_position: np.array([x,y])
    :params team_side: int

    :return: int
    """
    if (team_side == LEFT):
        if (robot_position[1] >=65 ):
            return  CCW
        return CW
    else:
        if (robot_position[1] < 65 ):
            return CW
        return CCW


def border_stuck(position_buffer, orientation):
    """

    :param position_buffer:
    :param orientation
    :return: true or false
    """
    flag = 0
    error = 2
    mean = sum(position_buffer[-5::])/5.0
    #if section(mean) == CENTER:
    #    return False
    #else:
    #orientation verify
    sec = section(position_buffer[-1])

    if sec not in BORDER_NORMALS.keys():
        return False

    orientation = np.array([math.cos(orientation), math.sin(orientation)])
    front_angle = math_utils.angleBetween(orientation, BORDER_NORMALS[sec])
    back_angle = math_utils.angleBetween(-orientation, BORDER_NORMALS[sec])

    angle = min(front_angle,back_angle)*180/math.pi

    #if angle < 15:

    mean = sum(position_buffer)/len(position_buffer)

    for x in position_buffer[-10::]:
        if not (mean[0]-error< x[0] < mean[0]+error or mean[1]-error < x[1] < mean[1]+error):
            flag = 1

    if flag == 0:
        return True
    else:
        return False
    #else:
        #    return False