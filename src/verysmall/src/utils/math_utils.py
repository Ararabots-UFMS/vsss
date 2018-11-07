import numpy as np
import numpy.linalg as la
import math
import scipy.stats as stats
import random
import rospy

MINCHANGE = 0.5

FORWARD = True
BACKWARDS = False


def unitVector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def angleBetween(v1, v2, abs=True):
    """ Returns the angle in radians between vectors 'v1' and 'v2' """
    cosang = np.dot(v1, v2)
    sinang = np.cross(v1, v2)
    if abs:
        sinang = la.norm(np.cross(v1, v2))
    return np.arctan2(sinang, cosang)  # atan2(y, x) or atan2(sin, cos)


def rotateVector(x, angle):
    """Rotate vector x anticlockwise around the origin by angle degrees, return angle in format [x, y]"""
    y1 = math.cos(angle)*x[0] - math.sin(angle)*x[1]
    y2 = math.sin(angle)*x[0] + math.cos(angle)*x[1]
    return [y1, y2]


def rotatePoint(origin, point, angle):
    """Rotate a point counterclockwise by a given angle around a given origin.
    The angle should be given in radians."""
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return (int(qx),int(qy))


def distancePoints(a, b):
    """Distance between two points"""
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5


def maxAbs(x, y):
    """Return the maximum absolute value"""
    return max(abs(x), abs(y))


def gaussian(m, v):
    return math.exp(-(m**2) / (2 * (v**2)))


def opposite_vector(vec):
    """
    Return the opposite vector
    :param vec: [float, float]
    :return: [float, float]
    """
    return [-vec[0], -vec[1]]


def min_diff_vec_and_opposite(num, orientation, vec, goal):
    """
    Return true if the angle difference between vec and goal is less then opposite vec and goal
    :param num: int
    :param orientation: boolean
    :param vec: [float, float]
    :param goal: [float, float]
    :return: boolean, int
    """
    rand = random.random()
    gamma_value = stats.gamma.cdf(num, a=10, scale=0.8)
    if rand < gamma_value:
        if angleBetween(vec, goal) <= angleBetween(opposite_vector(vec), goal):
            return True, 0
        return False, 0
    else:
        return orientation, num+1


def forward_min_diff(num, orientation, vec, goal, only_forward=False):
    """
     Return True if forward and the min difference angle
    :param num: int
    :param orientation: boolean
    :param vec: [float, float]
    :param goal: [float, float]
    :param only_forward: boolean
    :return: boolean, float,
    """
    tmp, new_gamma_count = min_diff_vec_and_opposite(num, orientation, vec, goal)
    if tmp or only_forward:
        return True, angleBetween(vec, goal, abs=False), new_gamma_count
    return False, angleBetween(opposite_vector(vec), goal, abs=False), new_gamma_count

def raio_vetores(p1, v1, p2, v2, speed_max=255, upper_bound=800, angle = 3,k = 0.01):
    p1 = np.array(p1)
    p2 = np.array(p2)
    v1 = np.array(v1)
    v2 = np.array(v2)
    ret = upper_bound

    cos = abs(np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))
    r1 = 2*(1-cos)
    d = np.linalg.norm(p1-p2)
    if (cos < 1 - angle):
        ret = 10/(np.sqrt(float(r1*k)))
    #rospy.logfatal("%4.3f %4.3f"%(cos,ret))
    return (ret/upper_bound) * speed_max

def get_orientation_and_angle(orientation, vec, goal_vector):
    # vec is the true robot orientation vector
    th_80 = 70. * (math.pi / 180.)
    th_110 = 120. * (math.pi/180.)

    theta = angleBetween(vec, goal_vector, abs=False)

    if th_80 < abs(theta) < th_110:
        return orientation, theta# maintains the same
    else:
        if abs(theta) > th_110:
            return False, theta # backward
        else:
            return True, theta # forward
