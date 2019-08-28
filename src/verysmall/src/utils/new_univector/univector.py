from math import pi, sqrt, cos, sin, atan2
import utils.new_univector.math_utils as math_utils
import numpy as np

de =  12#5.37
kr = 6
ko = 40
d_min = 8.5
delta = 4.5


def v_obs():
    return np.array([0, 0])


def v_robot():
    return np.array([0, 0])


def phiH(rho, theta, ccw=True, de=de, kr=kr):
    if rho > de:
        angle = (pi / 2) * (2 - ((de + kr) / (rho + kr)))
    elif 0 <= rho <= de:
        angle = (pi / 2) * sqrt(rho / de)

    if ccw:
        return math_utils.wrapToPi(theta + angle)
    else:
        return math_utils.wrapToPi(theta - angle)


def Nh(phi):
    return np.array([cos(phi), sin(phi)])


def phiR(d_x, d_y):  # Theta
    return math_utils.wrapToPi(atan2(d_y, d_x))


def phiTuf(theta, d_x, d_y, radius=de):
    y_l = d_y + radius
    y_r = d_y - radius

    rho_l = math_utils.norm(d_x, d_y - radius)
    rho_r = math_utils.norm(d_x, d_y + radius)

    phi_ccw = phiH(rho_l, theta, ccw=False)
    phi_cw = phiH(rho_r, theta, ccw=True)

    nh_ccw = Nh(phi_ccw)
    nh_cw = Nh(phi_cw)

    spiral_merge = (abs(y_l) * nh_ccw + abs(y_r) * nh_cw) / (2 * radius)

    if -radius <= d_y < radius:
        phi_tuf = atan2(spiral_merge[1], spiral_merge[0])
    elif d_y < -radius:
        phi_tuf = phiH(rho_l, theta, ccw=True)
    else:
        phi_tuf = phiH(rho_r, theta, ccw=False)

    return math_utils.wrapToPi(phi_tuf)


def phiAuf(obs_x, obs_y, r_x, r_y, obs_robot_dist, v_obs=v_obs(), v_robot=v_robot(), ko=ko):
    obstacle_position = np.array([obs_x, obs_y])
    s_vec = ko * (v_obs - v_robot)
    s_norm = math_utils.norm(s_vec[0], s_vec[1])
    d = obs_robot_dist

    if d >= s_norm:
        p_line_obs = obstacle_position + s_vec
    else:
        p_line_obs = obstacle_position + (d / s_norm) * s_vec

    delta_x, delta_y = math_utils.delta_axis(p_line_obs[0], p_line_obs[1], r_x, r_y)
    phi_auf = phiR(delta_x, delta_y)

    return math_utils.wrapToPi(phi_auf)


def phiComposed(phi_tuf, phi_auf, R, obstacle, delta=delta, d_min=d_min):
    if obstacle is None:
        phi_composed = math_utils.wrapToPi(phi_tuf)

    else:
        gauss = math_utils.gaussian(R - d_min, delta)

        if R <= d_min:
            phi_composed = phi_auf
        else:
            diff = math_utils.wrapToPi(phi_auf - phi_tuf)
            phi_composed = math_utils.wrapToPi(gauss * diff + phi_tuf)

    return math_utils.wrapToPi(phi_composed)


def generateUnivectorField(r_x, r_y, ball_pos, obs_pos, de=de, v_obs=
np.array([0, 0]), v_rob: list = np.array([0, 0]), ko: float = ko, delta:
float = delta, d_min: float = d_min) -> float:
    ball_x, ball_y = ball_pos
    d_ball_x, d_ball_y = math_utils.delta_axis(ball_x, ball_y, r_x, r_y)
    theta = phiR(d_ball_x, d_ball_y)
    phi_tuf = phiTuf(theta, d_ball_x, d_ball_y, de)

    obstacle = math_utils.closestObstacle(r_x, r_y, obs_pos)
    obs_x, obs_y = obstacle

    robot_obs_x, robot_obs_y = math_utils.delta_axis(obs_x, obs_y, r_x, r_y)
    R = math_utils.norm(robot_obs_x, robot_obs_y)
    robot_obs_dist = math_utils.norm(robot_obs_x, robot_obs_y)

    phi_auf = phiAuf(obs_x, obs_y, r_x, r_y, robot_obs_dist, v_obs, v_rob, ko)
    phi_composed = phiComposed(phi_tuf, phi_auf, R, obstacle, delta, d_min)

    return Nh(phi_composed)
