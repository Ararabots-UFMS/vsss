# -*- coding: utf-8 -*-
import math
import cv2
from time import time
import numpy as np
from typing import List, Tuple

from vision_module.seekers.seeker_data_structures import *
from vision_module.seekers.aruco_object_detector import ArucoObjectDetector
from vision_module.seekers.simple_object_detector import SimpleObjectDetector
from vision_module.seekers.obj_detector import ObjDetector

class BoundingBox:
    def __init__(self, tl:Vec2 = Vec2(), br:Vec2 = Vec2()):
        self.top_left = tl
        self.bottom_right = br

    def to_list(self):
        """ converts bounding box to list like this [[tl_x tl_y], [br_x, br_y]]"""
        return [self.top_left.to_list(), self.bottom_right.to_list()]

    def size(self) -> Tuple[float, float]:
        """ returns the width and height of the bounding box """
        return self.bottom_right.x - self.top_left.x, self.bottom_right.y - self.top_left.y

    def intersect(self, bbox) -> bool:
        """ returns if the bounding box passed as argument intersects
            this bounding box """
        tl_x = max(self.top_left.x, bbox.top_left.x)
        tl_y = max(self.top_left.y, bbox.top_left.y)
        br_x = min(self.bottom_right.x, bbox.bottom_right.x)
        br_y = min(self.bottom_right.y, bbox.bottom_right.y)

        intersect_area = max(0, br_x - tl_x + 1) * max(0, br_y - tl_y + 1)
        return (intersect_area > 0)

    def bound(self, tl_x, tl_y, br_x, br_y) -> None:
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

    def __str__(self) -> str:
        tl = self.top_left
        br = self.bottom_right
        return "[[%r, %r], [%r, %r]]" % (tl.x, tl.y, br.x, br.y)

    def __repr__(self) -> str:
        tl = self.top_left
        br = self.bottom_right
        return "BoundingBox(Vec2(%r, %r), Vec2(%r, %r))" % (tl.x, tl.y, br.x, br.y)

    def __mul__(self, alpha:float):
        """ alpha is a scalar number """
        center = (self.top_left + self.bottom_right) / 2.0
        diff = self.bottom_right - self.top_left
        w = alpha * Vec2(abs(diff.x)/2.0, abs(diff.y)/2.0)
        tl = center - w
        tl.x, tl.y = int(tl.x), int(tl.y)
        br = center + w
        br.x, br.y = int(br.x), int(br.y)
        return BoundingBox(tl, br)

    def __rmul__(self, alpha:float):
        return self.__mul__(alpha)
        
class NewSeeker:
    def __init__(self, num_objects:int, obj_detector:ObjDetector):
        self.num_objects = num_objects
        self.obj_detector = obj_detector
        self.trackers = [Tracker(self, i) for i in range(self.num_objects)]
        self.segments = []
        self.parent_bboxes = []
        self.img_shape = None  # (w, h) -> cols, lines

        if isinstance(obj_detector, ArucoObjectDetector):
            self.update = self.__aruco_update
            self.initialize = self.__aruco_initialize
            self.aruco_table = []
        else:
            self.update = self.__common_update
            self.initialize = self.__common_initialize

    def __aruco_initialize(self, frames:List[np.ndarray]) -> None:
        h, w = frames[0].shape[:2]
        self.img_shape = (w, h)

        cv2.imwrite('frame0.png', frames[0])
        cv2.imwrite('frame1.png', frames[1])

        objects_per_segment = [self.num_objects]
        segs = self.obj_detector.seek([frames[0]], objects_per_segment)

        for k,obj in enumerate(segs[0]):
            self.aruco_table.append(obj.id)
            self.trackers[k].set_id(obj.id)
            self.trackers[k].set_pos(obj.pos.x, obj.pos.y)

        #second frame
        segs = self.obj_detector.seek([frames[1]], objects_per_segment)
        self.segments = [[i for i in range(self.num_objects)]]
        self.parent_bboxes = [(Vec2(0, 0), Vec2(w, h))]

        self.update(segs)

    def __common_initialize(self, frames:List[np.ndarray]) -> None:
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
        self.img_shape = (w, h)
        for i,object in enumerate(segs[0]):
            self.trackers[i].set_pos(object.pos.x, object.pos.y)

        # second frame
        segs = self.obj_detector.seek([frames[1]], objects_per_segment)
        self.segments = [[i for i in range(self.num_objects)]]
        self.parent_bboxes = [(Vec2(0,0),Vec2(w,h))]
        self.update(segs)

    def feed(self, img_segments:List[np.ndarray]) -> None:
        # TODO: TEM QUE OLHAR O NOME DESSA FUNCAO, TALKEI?
        obj_in_segs = self.obj_detector.seek(img_segments, [len(seg) for seg in self.segments])
        self.update(obj_in_segs)

    def __aruco_update(self, objs_by_segment:List[ObjState]) -> None:
        # TODO: achar um nome melhor para o parâmetro objs_by_segment
        self.mapper(objs_by_segment)

        for index in range(len(objs_by_segment)):
            n = len(objs_by_segment[index])
            for i in range(n):
                try:
                    obj = objs_by_segment[index][i]
                    k = self.aruco_table.index(obj.id)
                    
                    self.trackers[k].update(obj.pos, obj.orientation)
                except ValueError:
                    print('tag id ', obj.id, 'doesnt exist')


    def __common_update(self, objs_in_segs:List[List[ObjState]]) -> None:
        # Remap position values before update
        self.mapper(objs_in_segs)

        # For each segment
        for index in range(len(self.segments)):
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

    def cluster_objects_and_trackers(self, trackers_index_list:List[int], objects_list:List[ObjState]) -> List[int]:
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

    def predict_all_windows(self) -> [(tuple)]:
        bboxes = []
        for i in range(self.num_objects):
            bboxes.append(self.trackers[i].predict_window(speed_gain=1))

        #print("predict_all... ", bboxes)
        self.fuser(bboxes)
        return self.parent_bboxes

    def get_intersections(self, intersection_matrix:np.ndarray, vertex_index:List[int], visited:List[int]) -> List[int]:
        intersected = []
        if vertex_index not in visited:
            intersected.append(vertex_index)
            visited.append(vertex_index)
            line = intersection_matrix[vertex_index, :].reshape(-1)
            intersections_indexes = np.where(line == 1)
            for v in list(intersections_indexes[0]):
                intersected += self.get_intersections(intersection_matrix, v, visited)
        return intersected

    def get_unions(self, intersection_matrix:np.ndarray) -> list:
        m = intersection_matrix
        n = self.num_objects
        intersections = []
        visited = []
        for v in range(n):
            if v not in visited:
                intersections.append(self.get_intersections(m, v, visited))
        return intersections
    
    def get_parent_bbox(self, bboxes:List[BoundingBox], bboxes_indexes:List[int]) -> tuple:
        """
            This function creates a bouding box that includes all bbox in bboxes
        """

        degenerated  = BoundingBox(Vec2(np.inf, np.inf),Vec2(-np.inf, -np.inf))
        b = sum([bboxes[i] for i in bboxes_indexes], degenerated)
        return (b.top_left, b.bottom_right)

    def fuser(self, bboxes:List[BoundingBox]) -> None:
        # self.get_parent_bbox()
        n = self.num_objects
        intersections = np.zeros((n, n))

        for i in range(n):
            for j in range(i,n):
                intersections[i,j] = int(bboxes[i].intersect(bboxes[j]))

        self.parent_bboxes = []
        self.segments = self.get_unions(intersections)

        for i, segment in enumerate(self.segments):
            self.parent_bboxes.append(self.get_parent_bbox(bboxes, segment))
            #print("fuser: %r -> %r" % (i, self.parent_bboxes[-1]))

    def mapper(self, objs_in_segs:List[ObjState]) -> None:
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
                objs[j].pos = objs[j].pos + self.parent_bboxes[i][0]
                #print("mapper: %r -> %r" % (i, self.parent_bboxes[i]))


    def get_serialized_objects(self):
        return [self.trackers[i].obj.flat() for i in range(self.num_objects)]


class Tracker():
    def __init__(self, seeker:NewSeeker, obj_id:int = -1):
        self.updated = False
        self.obj = ObjState(obj_id)
        self.t = 0.0
        self.bbox = BoundingBox()
        self.my_seeker = seeker

        self.lost_counter = 0

        # Saves the time of the last update
        self.last_update = None

        # This variable is used as the kalman filter object
        self.kalman = None

        self.angular_kalman = None

        self.init_kalman()



    def init_angular_kalman(self):
        dt = 1.0
        self.angular_kalman = cv2.KalmanFilter(3, 1, 0)
        self.angular_kalman.transitionMatrix = np.array([[1., dt, .5*dt**2],
                                                         [0., 1., dt],
                                                         [0., 0., 1.]]).reshape(3,3)
        self.angular_kalman.processNoiseCov = 1e-5 * np.eye(3)
        self.angular_kalman.measurementNoiseCov = 1e-1 * np.ones((1, 1))
        self.angular_kalman.measurementMatrix = 0. * np.zeros((1, 3))
        self.angular_kalman.measurementMatrix[0,0] = 1.
        self.angular_kalman.errorCovPost = 1. * np.ones((3, 3))
        self.angular_kalman.statePost = np.array([[0., 0., 0.]]).reshape(3,1)

    def init_kalman(self):
        #estimated frame rate
        dt = 1.0
        self.kalman = cv2.KalmanFilter(6, 2, 0)
        self.kalman.transitionMatrix = np.array([[1., 0., dt, 0, .5*dt**2, 0.],
                                                 [0., 1., 0., dt, 0., .5*dt**2],
                                                 [0., 0., 1., 0., dt, 0.],
                                                 [0., 0., 0., 1., 0., dt],
                                                 [0., 0., 0., 0., 1., 0.],
                                                 [0., 0., 0., 0., 0., 1.]]).reshape(6,6)

        self.kalman.processNoiseCov = 1e-5 * np.eye(6)
        self.kalman.measurementNoiseCov = 1e-1 * np.ones((2, 2))

        self.kalman.measurementMatrix = 0. * np.zeros((2, 6))
        self.kalman.measurementMatrix[0,0] = 1.
        self.kalman.measurementMatrix[1,1] = 1.

        self.kalman.errorCovPost = 1. * np.ones((6, 6))
        self.kalman.statePost = np.array([[0., 0., 0., 0., 0., 0.]]).reshape(6,1)

        self.init_angular_kalman()

    def set_dt(self, time_now):
        dt = time_now - self.last_update
        self.kalman.transitionMatrix[0,2] = dt
        self.kalman.transitionMatrix[1,3] = dt

    def __repr__(self):
        return "Tracker:\n---"+str(self.obj.id)+"\n---"+str(self.bbox)

    def set_id(self, id:int) -> None:
        self.obj.id = id

    def set_pos(self, x, y) -> None:
        self.obj.pos.x = x
        self.obj.pos.y = y

    def update(self, position:Vec2, orientation:float = 0.):
        # self.updated = True
        #
        # old_p = self.obj.pos
        # self.obj.pos = position
        #
        # t0 = time()
        # self.obj.speed = (1.0/(-self.t + t0)) * (self.obj.pos - old_p)
        # self.t = t0
        #
        # self.obj.orientation = orientation

        now = time()
        if self.last_update is None and np.all(position is not None):  # first run
            self.init_kalman()
            # A initialization state must be provided to the kalman filter
            self.kalman.statePost = np.array([[position[0], position[1], 0., 0., 0., 0.]]).reshape(6, 1)

            if orientation is not None:
                self.angular_kalman.statePost = np.array([[orientation, 0., 0.]]).reshape(3, 1)
            else:
                self.angular_kalman.statePost = np.array([[0, 0., 0.]]).reshape(3, 1)

            self.lost_counter = 0
            self.obj.velocity = np.array([0, 0])
        else:
            self.kalman.predict()
            self.angular_kalman.predict()

            # updates the kalman filter
            if np.all(position is not None) and self.lost_counter < 60:
                self.kalman.correct(position.to_np().reshape(2, 1))
                if orientation is not None:
                    self.angular_kalman.correct(np.array([orientation]).reshape(1, 1))
                self.lost_counter = 0
            else:  # updates the lost counter
                self.lost_counter += 1

            # uses the kalman info
            state = self.kalman.predict()
            pos = np.array([state[0, 0], state[1, 0]])
            self.obj.velocity = np.array([state[2, 0], state[3, 0]]) * 60.0

            if orientation is not None and abs(abs(orientation) - math.pi) < 0.15:
                # self.init_angular_kalman()
                self.angular_kalman.statePost = np.array([[orientation, 0., 0.]]).reshape(3, 1)
            elif orientation is None:
                orientation = self.angular_kalman.predict()[0, 0]

        if self.lost_counter >= 60:  # if the thing was lost in all previous 10 frames
            self.reset()
        else:
            # Updates the robot's state variables
            self.updated = True
            self.last_update = now

            old_p = self.obj.pos
            self.obj.pos = position

            t0 = time()
            self.obj.velocity = (1.0 / (-self.t + t0)) * (self.obj.pos - old_p)
            self.t = t0

            self.obj.orientation = orientation

    def predict(self, dt:int = -1) -> Vec2:
        """
            This function predicts the next object position given a time step dt
            :param dt : float
            :return position : vec2
        """
        if dt < 0:
            dt = time() - self.t
        return self.obj.pos + self.obj.velocity * dt

    def predict_window(self, speed_gain:float = 2.5, scale_factor:float = 2.0, ) -> BoundingBox:

        if not self.updated:
            return self.bbox

        l = self.my_seeker.obj_detector.obj_size / 2.0
        tl = self.obj.pos + Vec2(-l, -l)
        br = self.obj.pos + Vec2(l, l)
        bbox = scale_factor * BoundingBox(tl, br)
        dt = time() - self.t

        if self.obj.velocity.y < 0:
            bbox.top_left.y += speed_gain * self.obj.velocity.y * dt
        else:
            bbox.bottom_right.y += speed_gain * self.obj.velocity.y * dt

        if self.obj.velocity.x < 0:
            bbox.top_left.x += speed_gain * self.obj.velocity.x * dt
        else:
            bbox.bottom_right.x += speed_gain * self.obj.velocity.x * dt

        bbox.bound(0, 0, *self.my_seeker.img_shape)

        self.bbox = bbox

        self.updated = False

        return bbox