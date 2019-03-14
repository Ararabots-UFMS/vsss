import numpy as np
from time import time

class Vec2:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def to_list(self):
        return [self.x, self.y]

    def to_np(self):
        return np.array(self.to_list())

    def __add__(self, vec):
        return Vec2(self.x + vec.x, self.y + vec.y)

    def __radd__(self, vec):
        _v = Vec2(self.x + vec.x, self.y + vec.y)
        return _v

    def __mul__(self, alpha):
        """ alpha is a scalar number """
        return Vec2(alpha*self.x, alpha*self.y)

    def __str__(self):
        return "({0}, {1})".format(self.x, self.y)

class BoundingBox:
    def __init__(self):
        self.top_left = Vec2()
        self.bottom_right = Vec2()

    def to_list(self):
        """ converts bounding box to list like this [[tl_x tl_y], [br_x, br_y]]"""
        return [self.top_left.to_list(), self.bottom_right.to_list()]

    def size(self):
        """ returns the width and height of the bounding box """
        return self.bottom_right.x - self.top_left.x, self.bottom_right.y - self.top_left.y

    def intersect(self, bbox):
        """ returns if the bounding box passed as argument intersects
            this bounding box """
        tl_x = max(self.top_left.x, bbox.top_left.x)
        tl_y = max(self.top_left.y, bbox.top_left.y)
        br_x = min(self.bottom_right.x, bbox.bottom_right.x)
        br_y = min(self.bottom_right.y, bbox.bottom_right.y)

        intersect_area = max(0, br_x - tl_x + 1) * max(0, br_y - tl_y + 1)
        return (intersect_area > 0)

class Tracker():
    def __init__(self):
        self.position = Vec2()
        self.speed = Vec2()
        self.t = 0.0
        self.bbox = BoundingBox()

    def update(self, position):
        old_p = self.position
        self.position = position
        t0 = self.t
        self.t = time()
        self.speed = (old_p - self.position) / (self.t - t0)

    def predict(self):
        dt = time() - self.t
        return self.position + self.speed*dt


    def predict_window(self):
        pass

class NewSeeker:

    def __init__(self, num_objects, obj_detector):
        pass
