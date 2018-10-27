import numpy as np
from arena_sections import *
CW = 0
CCW = 1

def distance_point(position_one, position_two):
    return np.linalg.norm(position_one-position_two)

def near_ball(ball_position, robot_position):
    """
    Returns if the robot is near to the ball
    :params ball_position: np.array([x,y])
    :params robot_position: np.array([x,y])
    :return: boolean
    """
    distance = np.linalg.norm(ball_position-robot_position)
    return (distance <= 6.5)

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
