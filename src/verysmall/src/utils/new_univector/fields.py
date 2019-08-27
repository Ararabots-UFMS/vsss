from copy import copy
import cv2
import draw
from get_vectors import getVectors
import math_utils
import measures
import numpy as np
import univector

ball = measures.ball
obstacles = list([(10, 20), (50, 10)])
def hyperbolic(r_x, r_y, ball_pos, __):
    
    ball_x, ball_y = ball_pos
    delta_x, delta_y = math_utils.delta_axis(ball_x, ball_y, r_x, r_y)
    theta = univector.phiR(delta_x, delta_y)
    rho = math_utils.norm(delta_x, delta_y)
    phi_h = univector.phiH(rho, theta)

    return univector.Nh(phi_h)

def repulsive(r_x, r_y, __, obs_pos):
    obs_x, obs_y = obs_pos[0]
    delta_x, delta_y = math_utils.delta_axis(obs_x, obs_y, r_x, r_y)
    phi_r = univector.phiR(delta_x, delta_y)

    return univector.Nh(phi_r)

def moveToGoal(r_x, r_y, ball_pos, __):
    ball_x, ball_y = ball_pos
    delta_x, delta_y = math_utils.delta_axis(ball_x, ball_y, r_x, r_y)
    theta = univector.phiR(delta_x, delta_y)

    phi_tuf = univector.phiTuf(theta, delta_x, delta_y)
    return univector.Nh(phi_tuf)

def avoidObstacle(r_x, r_y, __, obs_pos):
    obs_x, obs_y = obs_pos[0]
    obs_robot_x, obs_robot_y = math_utils.delta_axis(obs_x, obs_y, r_x, r_y)
    obs_robot_dist = math_utils.norm(obs_robot_x, obs_robot_y)
    phi_auf = univector.phiAuf(obs_x, obs_y, r_x, r_y, obs_robot_dist)

    return univector.Nh(phi_auf)

def composition(r_x, r_y, ball_pos, obs_pos):
    ball_x, ball_y = ball_pos
    ball_robot_x, ball_robot_y = math_utils.delta_axis(ball_x, ball_y, r_x, r_y)
    theta = univector.phiR(ball_robot_x, ball_robot_y)
    phi_tuf = univector.phiTuf(theta, ball_robot_x, ball_robot_y)

    obstacle = math_utils.closestObstacle(r_x, r_y, obs_pos)
    obs_x, obs_y = obstacle

    obs_robot_x, obs_robot_y = math_utils.delta_axis(obs_x, obs_y, r_x, r_y)
    obs_robot_dist = math_utils.norm(obs_robot_x, obs_robot_y)
    phi_auf = univector.phiAuf(obs_x, obs_y, r_x, r_y, obs_robot_dist)

    phi_composed = univector.phiComposed(phi_tuf, phi_auf, obs_robot_dist, obstacle)

    return univector.Nh(phi_composed)





w, h = measures.getArenaSize()
img_w, img_h = measures.getImageSize()
step = measures.step

field = np.zeros((img_h, img_w, 3))
vectors = getVectors(w, h, step, composition, ball, obstacles)
vectorField = draw.drawVectorField(copy(field), vectors, w, h, step, ball, obstacles)

cv2.imshow('field', vectorField)
cv2.waitKey(0)

cv2.destroyAllWindows()