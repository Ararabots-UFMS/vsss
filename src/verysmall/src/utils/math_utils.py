import numpy as np
import numpy.linalg as la
import math

MINCHANGE = 0.174533

FORWARD = 1
BACKWARDS = 0

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

def min_diff_vec_and_opposite(orientation, vec, goal):
    """
    Return true if the angle difference between vec and goal is less then opposite vec and goa
    :param vec: [float, float]
    :param goal: [float, float]
    :return: boolean
    """
    if (angleBetween(vec, goal) - angleBetween(opposite_vector(vec), goal)) <= MINCHANGE:
        return bool(orientation)

    if angleBetween(vec, goal) <= angleBetween(opposite_vector(vec), goal):
        return True
    return False

def forward_min_diff(orientation, vec, goal):
    """
    Return True if forward and the min difference angle
    :param vec: [float, float]
    :param goal: [float, float]
    :return: boolean, float
    """
    tmp = min_diff_vec_and_opposite(orientation, vec, goal)
    if tmp:
        return True, angleBetween(vec, goal)
    return False, angleBetween(opposite_vector(vec), goal)