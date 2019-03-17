import numpy as np
import math
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
    def __init__(self, seeker):
        self.position = Vec2()
        self.speed = Vec2()
        self.t = 0.0
        self.bbox = BoundingBox()
        self.my_seeker = seeker

    def update(self, position):
        old_p = self.position
        self.position = position
        t0 = self.t
        self.t = time()
        self.speed = (old_p - self.position) / (self.t - t0)

    def predict(self, dt = -1):
        """
            This function predicts the next object position given a time step dt
            :param dt : float
            :return position : vec2
        """
        if dt < 0:
            dt = time() - self.t

        return self.position + self.speed*dt


    def predict_window(self):

        return self.bbox
        pass

class NewSeeker:

    def __init__(self, num_objects, obj_detector):
        self.num_objects = num_objects
        self.obj_detector = obj_detector
        self.trackers = [Tracker(self) for i in range(self.num_objects)]
        self.segments = []
        self.parent_bboxes = []


    def predict_all_windows(self):
        bboxes = []
        for i in range(self.num_objects):
            bboxes.append(self.trackers[i].predict_window())
        self.fuser(bboxes)
        return self.parent_bboxes


    def get_intersections(self, intersection_matrix, vertex_index, visited):
        intersected = []
        if vertex_index not in visited:
            intersected.append(vertex_index)
            visited.append(vertex_index)
            intersections_indexes = np.where(intersection_matrix[vertex_index, :].reshape(-1) == 1)
            #print(list(intersections_indexes[0]))
            for v in list(intersections_indexes[0]):
                intersected += self.get_intersections(intersection_matrix, v, visited)
                print(intersected)
        return intersected

    def get_unions(self, intersection_matrix):
        m = intersection_matrix
        n = self.num_objects
        intersections = []
        visited = []
        for v in range(n):
            if v not in visited:
                intersections.append(self.get_intersections(m, v, visited))
        return intersections


    def get_parent_bbox(self, bboxes, bboxes_indexes):
        """
            This function creates a bouding box that includes all bbox in bboxes
            :param bboxes: list(BoundingBox)
            :param bboxes_indexes: list(int)
        """
        top_left = vec2(math.inf, math.inf)
        bottom_right = vec2(-math.inf, -math.inf)
        for i in bboxes_indexes:
            top_left.x = min(top_left.x, bboxes[i].top_left.x)
            top_left.y = min(top_left.y, bboxes[i].top_left.y)

            bottom_right.x = max(bottom_right.x, bboxes[i].bottom_right.x)
            bottom_right.y = max(bottom_right.y, bboxes[i].bottom_right.y)

        return (top_left,bottom_right)

    def fuser(self, bboxes):

        n = self.num_objects
        intersections = np.zeros((n, n))

        for i in range(n):
            for j in range(i+1,n):
                intersections[i,j] = int(bboxes[i].intersect(bboxes[j]))

        self.parent_bboxes = []
        self.segments = self.get_unions(intersections)
        for segment in self.segments:            
            self.parent_bboxes.append(self.get_parent_bbox(bboxes, segment))
