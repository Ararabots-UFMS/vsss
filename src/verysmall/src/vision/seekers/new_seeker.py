# -*- coding: utf-8 -*-

import numpy as np
import math
import cv2
from seeker_data_structures import *
from time import time
from aruco_object_detector import ArucoObjectDetector
from simple_object_detector import SimpleObjectDetector


class BoundingBox:
    def __init__(self, tl=Vec2(), br=Vec2()):
        self.top_left = tl
        self.bottom_right = br

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

    def bound(self, tl_x, tl_y, br_x, br_y):
        self.top_left.x = max(tl_x, self.top_left.x)
        self.top_left.y = max(tl_y, self.top_left.y)
        self.bottom_right.x = min(br_x, self.bottom_right.x)
        self.bottom_right.y = min(br_y, self.bottom_right.y)

    def __add__(self, b):
        tl, br = Vec2(), Vec2()
        tl.x = min(self.top_left.x, b.top_left.x)
        tl.y = min(self.top_left.y, b.top_left.y)
        br.x = max(self.bottom_right.x, b.bottom_right.x)
        br.y = max(self.bottom_right.y, b.bottom_right.y)
        return BoundingBox(tl, br)

    def __radd__(self, b):
        return self.__add__(b)

    # def __iadd__(self, b):
    #     self.top_left.x = min(self.top_left.x, b.top_left.x)
    #     self.top_left.y = min(self.top_left.y, b.top_left.y)
    #     self.bottom_right.x = max(self.bottom_right.x, b.bottom_right.x)
    #     self.bottom_right.y = max(self.bottom_right.y, b.bottom_right.y)

    def __str__(self):
        tl = self.top_left
        br = self.bottom_right
        return "[[%r, %r], [%r, %r]]" % (tl.x, tl.y, br.x, br.y)

    def __repr__(self):
        tl = self.top_left
        br = self.bottom_right
        return "BoundingBox(Vec2(%r, %r), Vec2(%r, %r))" % (tl.x, tl.y, br.x, br.y)

    def __mul__(self, alpha):
        """ alpha is a scalar number """
        center = (self.top_left + self.bottom_right) / 2.0
        diff = self.bottom_right - self.top_left
        w = alpha * Vec2(abs(diff.x)/2.0, abs(diff.y)/2.0)
        tl = center - w
        tl.x, tl.y = int(tl.x), int(tl.y)
        br = center + w
        br.x, br.y = int(br.x), int(br.y)
        return BoundingBox(tl, br)

    def __rmul__(self, alpha):
        return self.__mul__(alpha)

class Tracker():
    def __init__(self, seeker, obj_id=-1, alpha=2.5):
        self.obj = ObjState(obj_id)
        self.t = 0.0
        self.bbox = BoundingBox()
        self.my_seeker = seeker
        self.alpha = alpha

    def set_id(self, id):
        self.obj.id = id

    def set_pos(self, x, y):
        self.obj.pos.x = x
        self.obj.pos.y = y

    def set_obj_size(self):
        # TODO:
        pass

    def update(self, position, orientation=0.0):
        old_p = self.obj.pos
        self.obj.pos = position
        #print("pos", position)
        t0 = self.t
        self.t = time()
        self.obj.speed = (1/(self.t - t0)) * (self.obj.pos - old_p)
        #print("vel", self.obj.speed)
        self.obj.orientation = orientation

    def predict(self, dt = -1):
        """
            This function predicts the next object position given a time step dt
            :param dt : float
            :return position : vec2
        """
        if dt < 0:
            dt = time() - self.t
        return self.obj.position + self.obj.speed*dt

    def predict_window(self):
        l = self.my_seeker.obj_detector.obj_size / 2.0

        tl = self.obj.pos + Vec2(-l, -l)
        br = self.obj.pos + Vec2(l,l)
        bbox = 2 * BoundingBox(tl, br)
        a = self.alpha
        dt = time() - self.t

        if self.obj.speed.y < 0:
            bbox.top_left.y += a * self.obj.speed.y * dt
        else:
            bbox.bottom_right.y += a * self.obj.speed.y * dt

        if self.obj.speed.x < 0:
            bbox.top_left.x += a * self.obj.speed.x * dt
        else:
            bbox.bottom_right.x += a * self.obj.speed.x * dt

        w,h = self.my_seeker.img_shape
        bbox.bound(0, 0, w, h)
        print(bbox)
        return bbox

class NewSeeker:
    def __init__(self, num_objects, obj_detector):
        self.num_objects = num_objects
        self.obj_detector = obj_detector
        self.trackers = [Tracker(self, i) for i in range(self.num_objects)]
        self.segments = []
        self.parent_bboxes = []
        self.img_shape = None  # (w, h) -> cols, lines

        if isinstance(obj_detector, ArucoObjectDetector):
            self.update = self.aruco_update
            self.initialize = self.aruco_initialize
            self.aruco_table = []

    def aruco_initialize(self, frames):
        objects_per_segment = [self.num_objects]
        segs = self.obj_detector.seek([frames[0]], objects_per_segment)
        for k,obj in enumerate(segs[0]):
            self.aruco_table.append(obj.id)
            self.trackers[k].set_id(obj.id)
            self.trackers[k].set_pos(obj.pos.x, obj.pos.y)

        segs = self.obj_detector.seek([frames[1]], objects_per_segment)
        self.update(segs)

    def initialize(self, frames):
        """
            This function should be capable of initialing the state variables
            from the trackers

            frames: two frames of the full image already segementeds
        """
        objects_per_segment = [self.num_objects]

        segs = self.obj_detector.seek([frames[0]], objects_per_segment)
        h,w = frames[0].shape[:2]
        cv2.imwrite('frame0.png', frames[0])
        cv2.imwrite('frame1.png', frames[1])
        self.img_shape = (w,h)
        for i,object in enumerate(segs[0]):
            self.trackers[i].set_pos(object.pos.x, object.pos.y)

        # second frame
        segs = self.obj_detector.seek([frames[1]], objects_per_segment)
        self.segments = [[i for i in range(self.num_objects)]]
        self.parent_bboxes = [(Vec2(0,0),Vec2(w,h))]
        self.update(segs)

    def feed(self, img_segments):
        # TODO: TEM QUE OLHAR O NOME DESSA FUNCAO, TALKEI?
        obj_in_segs = self.obj_detector.seek(img_segments, [len(seg) for seg in self.segments])
        self.update(obj_in_segs)

    def aruco_update(self, objs_by_segment):
        # TODO: achar um nome melhor para o parâmetro objs_by_segment
        segments = objs_by_segment
        for segment in segments:
            for obj in segment:
                k = self.aruco_table.index(obj.id)
                self.trackers[k].update(obj.pos, obj.orientation)

    def update(self, objs_in_segs):
        # Remap position values before update
        self.mapper(objs_in_segs)
        # For each segment
        for index in range(len(objs_in_segs)):
            # Build a distance matrix between postion given and segments
            n = len(objs_in_segs[index])
            if n > 1:
                # Sort the positions with current tracker positions
                t = self.segments[index]
                o = objs_in_segs[index]
                sorted_objects = self.cluster_objects_and_trackers(t, o)

                # Update each tracker
                for k in range(n):
                    self.trackers[t[k]].update(sorted_objects[k].pos)

            elif len(objs_in_segs[index]) == 0:
                pass # Do nothing in case of empty array
            else:
                # Trivial case
                # Update only one tracker
                obj = objs_in_segs[index][0]
                self.trackers[self.segments[index][0]].update(obj.pos)

    def cluster_objects_and_trackers(self, trackers_index_list, objects_list):
        n = len(trackers_index_list)
        if n != len(objects_list):
            print("Incorrect size of arrays!")
            return []

        trackers = [i for i in range(len(trackers_index_list))]
        objects = [i for i in range(len(objects_list))]

        distances = np.full((n,n), np.inf)
        for i in range(n):
            for j in range(n):
                tracker_pos = self.trackers[trackers[i]].obj.pos
                obj_pos = objects_list[objects[j]].pos
                distances[i][j] = abs(tracker_pos - obj_pos)

        sorted_objects = [-1 for i in range(n)]
        while n > 1:
            flat_index = np.argmin((distances[trackers,:])[:, objects])
            t_index = int(flat_index / n)
            obj_index = flat_index % n

            sorted_objects[trackers[t_index]] = objects_list[objects[obj_index]]
            del(trackers[t_index])
            del(objects[obj_index])
            n -= 1

        sorted_objects[trackers[0]] = objects_list[objects[0]]
        return sorted_objects

    def predict_all_windows(self):
        bboxes = []
        for i in range(self.num_objects):
            bboxes.append(self.trackers[i].predict_window())
        #the fuser function append the self.parent_bboxes and dont return anything
        self.fuser(bboxes)
        return self.parent_bboxes

    def get_intersections(self, intersection_matrix, vertex_index, visited):
        intersected = []
        if vertex_index not in visited:
            intersected.append(vertex_index)
            visited.append(vertex_index)
            line = intersection_matrix[vertex_index, :].reshape(-1)
            intersections_indexes = np.where(line == 1)
            for v in list(intersections_indexes[0]):
                intersected += self.get_intersections(intersection_matrix, v, visited)
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
        tl = Vec2(np.inf, np.inf)
        br = Vec2(-np.inf, -np.inf)
        b = BoundingBox(tl, br)
        for i in bboxes_indexes:
            b = b + bboxes[i]

        return (b.top_left, b.bottom_right)

    def fuser(self, bboxes):
        n = self.num_objects
        intersections = np.zeros((n, n))

        for i in range(n):
            for j in range(i,n):
                intersections[i,j] = int(bboxes[i].intersect(bboxes[j]))

        self.parent_bboxes = []
        self.segments = self.get_unions(intersections)
        for segment in self.segments:
            self.parent_bboxes.append(self.get_parent_bbox(bboxes, segment))

    def mapper(self, objs_in_segs):
        """
            maps the position of the objects in segment to global coordinates
            objs_in_segs:       list of objects in each segment defined by the parent boxes

        """

        k = len(objs_in_segs)
        if k != len(self.parent_bboxes):
            return
        for i in range(k):
            objs = objs_in_segs[i]
            for j in range(len(objs)):
                #print(objs[j].pos, "+", self.parent_bboxes[i][0])
                objs[j].pos = objs[j].pos + self.parent_bboxes[i][0]


    def get_serialized_objects(self):
        return [self.trackers[i].obj.flat() for i in range(self.num_objects)]
