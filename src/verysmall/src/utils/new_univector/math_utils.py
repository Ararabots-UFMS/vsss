from math import pi, sqrt, exp

def wrapToPi(angle):
    if angle > pi:
        return angle - 2 * pi
    elif angle < -pi:
        return 2 * pi + angle
    else:
        return angle

def delta_axis(x_1, y_1, x_2, y_2):
    return x_2 - x_1, y_2 - y_1

def norm(d_x, d_y):
    return sqrt(d_x ** 2 + d_y ** 2)

def gaussian(r, delta_const):
    return exp(-((r **2) / (2 * delta_const ** 2)))

def closestObstacle(r_x, r_y, obstacles):
    
    prev_dist = 0
    init = True
    for obstacle in obstacles:
        obs_x, obs_y = obstacle
        delta_x, delta_y = delta_axis(r_x, r_y, obs_x, obs_y)

        if init:
            prev_dist = norm(delta_x, delta_y)
        if norm(delta_x, delta_y) <= prev_dist:
            closest_obs = obstacle
            prev_dist = norm(delta_x, delta_y)

        init = False
    return closest_obs






