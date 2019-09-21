import math
from enum import Enum

import numpy as np
import rospy

from robot_module.movement.definitions import OpCodes
from strategy.arena_utils import on_attack_side
from strategy.arena_utils import section, LEFT, BORDER_NORMALS
from utils import math_utils

CW = 0
CCW = 1
DEGREE = float


class GameStates(Enum):
    STOPPED = 0
    NORMAL = 1
    FREE_BALL = 2
    PENALTY = 3
    META = 4


######################
# distance functions #
######################

def distance_point(position_one, position_two):
    return np.linalg.norm(position_one - position_two)


def near_ball(ball_position, robot_position, _distance=9.5):
    """
    Returns if the robot is near to the ball
    :params ball_position: np.array([x,y])
    :params robot_position: np.array([x,y])
    :return: boolean
    """
    distance = distance_point(ball_position, robot_position)
    return distance <= _distance


def behind_ball(ball_position, robot_position, team_side, _distance=9.5):
    """
    Returns if the robot is behind to the ball
    :params ball_position: np.array([x,y])
    :params robot_position: np.array([x,y])
    :params team_side: int
    :return: boolean
    """

    if team_side == LEFT:
        if near_ball(ball_position, robot_position, _distance):
            return robot_position[0] < ball_position[0]
    else:
        if near_ball(ball_position, robot_position, _distance):
            return robot_position[0] >= ball_position[0]
    return False

def is_behind_ball(ball_position: np.ndarray,
                    robot,
                    team_side: int,
                    max_distance: float = 10.0,
                    max_angle: DEGREE = 15) -> bool:

    distance = np.linalg.norm(ball_position - robot.position)

    if distance > max_distance:
        rospy.logfatal("distance")
        return False

    theta = robot.orientation
    robot_vector = np.array([math.cos(theta), math.sin(theta)])
    rb_vector = ball_position - robot.position
    angle1 = math_utils.angle_between(robot_vector, rb_vector, abs=False)
    angle2 = math_utils.angle_between(-robot_vector, rb_vector, abs=False)

    max_angle = max_angle * math_utils.DEG2RAD

    if not (abs(angle1) < max_angle or abs(angle2) < max_angle):
        rospy.logfatal("angle")
        return False

    if team_side == LEFT:
        if ball_position[0] > robot.position[0]:
            rospy.logfatal("lado")
            return False
    else:
        if ball_position[0] > robot.position[0]:
            rospy.logfatal("lado_else")
            return False
    return True

def spin_direction(ball_position, robot_position, team_side, invert=False):
    """
    Returns the direction of the spin
    :params ball_position: np.array([x,y])
    :params robot_position: np.array([x,y])
    :params team_side: int

    :return: int
    """

    if team_side == LEFT:
        if robot_position[1] >= 65:
            return OpCodes.SPIN_CCW if not invert else OpCodes.SPIN_CW
        return OpCodes.SPIN_CW if not invert else OpCodes.SPIN_CCW
    else:
        if robot_position[1] < 65:
            return OpCodes.SPIN_CW if not invert else OpCodes.SPIN_CCW
        return OpCodes.SPIN_CCW if not invert else OpCodes.SPIN_CW


def border_stuck(position_buffer, orientation):
    """

    :param position_buffer:
    :param orientation
    :return: true or false
    """
    flag = 0
    error = 2
    mean = sum(position_buffer[-5::]) / 5.0
    # if section(mean) == CENTER:
    #    return False
    # else:
    # orientation verify
    sec = section(position_buffer[-1])

    if sec not in BORDER_NORMALS.keys():
        return False

    orientation = np.array([math.cos(orientation), math.sin(orientation)])
    front_angle = math_utils.angle_between(orientation, BORDER_NORMALS[sec])
    back_angle = math_utils.angle_between(-orientation, BORDER_NORMALS[sec])

    angle = min(front_angle, back_angle) * 180 / math.pi

    # if angle < 15:

    mean = sum(position_buffer) / len(position_buffer)

    for x in position_buffer[-10::]:
        if not (mean[0] - error < x[0] < mean[0] + error or mean[1] - error < x[1] < mean[1] + error):
            flag = 1

    if flag == 0:
        return True
    else:
        return False
    # else:
    #    return False


def ball_on_attack_side(ball_position, team_side) -> bool:
    return on_attack_side(ball_position, team_side)


def robot_behind_ball(robot_position, ball_position, team_side) -> bool:
    return behind_ball(robot_position, ball_position, team_side)


def ball_on_critical_position(ball_position) -> bool:
    return ball_position[0] < 30 or ball_position[0] > 120


def ball_on_border(ball_position, team_side) -> bool:
    if not ball_on_attack_side(ball_position, team_side) and not ball_on_critical_position(ball_position):
        sec = section(ball_position)

    return sec.value in BORDER_NORMALS.keys()
