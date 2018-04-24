import numpy as np
import math  

def unitVector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angleBetween(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'"""
    v1_u = unitVector(v1)
    v2_u = unitVector(v2)
    dot = v1_u[0]*v2_u[0] + v1_u[1]*v2_u[1]      # dot product
    det = v1_u[0]*v2_u[1] - v1_u[1]*v2_u[0]      # determinant
    aux = math.atan2(det, dot)
    if aux < 0:
        aux += 2*math.pi
    return aux  # atan2(y, x) or atan2(sin, cos)

def rotate(origin, point, angle):
    """Rotate a point counterclockwise by a given angle around a given origin.
    The angle should be given in radians."""
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return (int(qx),int(qy))