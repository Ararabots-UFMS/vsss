import numpy as np
import math
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

    def __add__(self, b):
        tl, br = Vec2(), Vec2()
        tl.x = min(self.top_left.x, b.top_left.x)
        tl.y = min(self.top_left.y, b.top_left.y)
        br.x = max(self.bottom_right.x, b.bottom_right.x)
        br.y = max(self.bottom_right.y, b.bottom_right.y)
        return BoundingBox(tl, br)

    def __radd__(self, b):
        return self.__add__(b)

    def __iadd__(self, b):
        self.top_left.x = min(self.top_left.x, b.top_left.x)
        self.top_left.y = min(self.top_left.y, b.top_left.y)
        self.bottom_right.x = max(self.bottom_right.x, b.bottom_right.x)
        self.bottom_right.y = max(self.bottom_right.y, b.bottom_right.y)

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
    def __init__(self, seeker, obj_id=-1, alpha=1.5):
        self.obj = ObjState(obj_id)
        self.t = 0.0
        self.bbox = BoundingBox()
        self.my_seeker = seeker
        self.alpha = alpha

    def set_id(self, id):
        self.obj.id = id

    def set_pos(self, x, y):
        self.obj.position.x = x
        self.obj.position.y = y

    def set_obj_size(self):
        # TODO:
        pass

    def update(self, position, orientation=0.0):
        old_p = self.obj.position
        self.obj.position = position
        t0 = self.t
        self.t = time()
        self.obj.speed = (1/(self.t - t0)) * (old_p - self.obj.position)
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
        # l = self.my_seeker.obj_detector.obj_size
        # tl = self.obj.pos + Vec2(-l, -l)
        # br = self.obj.pos + Vec2(l,l)
        # a = self.alpha
        # # Calculates search region
        # if self.obj.speed.y < 0:
        #     tl.y += self.obj.speed.y * alpha
        #     end_line = self.last_pos[1] + s
        # else:
        #     start_line = self.last_pos[1] - s
        #     end_line = self.last_pos[1] + s + vy * delta_t
        #
        # if vx < 0:
        #     start_col = self.last_pos[0] - s + vx * delta_t
        #     end_col = self.last_pos[0] + s
        # else:
        #     start_col = self.last_pos[0] - s
        #     end_col = self.last_pos[0] + s + vx * delta_t
        pass



class NewSeeker:
    def __init__(self, num_objects, obj_detector, img_shape):
        self.num_objects = num_objects
        self.obj_detector = obj_detector
        self.trackers = [Tracker(self, i) for i in range(self.num_objects)]
        self.segments = []
        self.parent_bboxes = []
        self.img_shape = img_shape # (w, h) -> cols, lines

        if isinstance(obj_detector, ArucoObjectDetector):
            self.update = self.aruco_update
            self.initialize = self.aruco_initialize

    def aruco_initialize(self, frames):
        objects_per_segment = [self.num_objects]
        segs = self.obj_detector.seek([frames[0]], objects_per_segment)
        for k,obj in enumerate(segs[0]):
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

        # first frame
        segs = self.obj_detector.seek([frames[0]], objects_per_segment)
        for i,object in enumerate(segs[0]):
            # just assign a object to a tracker
            self.trackers[i].set_pos(object.x, object.y)

        # TODO: set the size of each object into the trackers

        # second frame
        segs = self.obj_detector.seek([frames[1]], objects_per_segment)
        self.update(segs)

    def feed(self, img_segments):
        # TODO: TEM QUE OLHAR O NOME DESSA FUNCAO, TALKEI?
        obj_states_in_segs = self.obj_detector.seek(img_segments, [len(seg) for seg in self.segments])
        self.update(obj_states_in_segs)

    def sort_by_distance_matrix(self, trackers_in_segment, objects_in_segment):
        """
        :param trackers_in_segment:
        :param objects_in_segment:
        :return: Array of positions sorted by trackers in segment
        """
        # Verify if the two arrays are the same size
        if len(trackers_in_segment) == len(objects_in_segment):

            size_of_matrix = len(trackers_in_segment) # Get one of the sizes since is a n x n matrix
            matrix = np.zeros(shape=(size_of_matrix,size_of_matrix)) # Create the matrix

            tracker_position_array = np.empty(size_of_matrix)
            sorted_array = np.array([Vec2()]*size_of_matrix)

            for i in range(size_of_matrix): # rows for trackers in segment
                tracker_position_array[i] = np.inf
                tracker_index = trackers_in_segment[i]

                for j in range(size_of_matrix): # col for objects founds

                    new_distance = matrix[i][j] = abs(self.trackers[tracker_index].position - objects_in_segment[j])
                    if new_distance < tracker_position_array[i]:
                        tracker_position_array[i] = new_distance
                        sorted_array[i] = objects_in_segment[j]

            return sorted_array
        else:
            print("Incorrect size of arrays!")
            return []

    def aruco_update(self, objs_by_segment):
        # TODO: achar um nome melhor para o parâmetro objs_by_segment
        k = 0
        segments = objs_by_segment
        for segment in segments:
            for obj in segment:
                self.trackers[k].update(obj.pos, obj.orientation)
                k += 1

    def update(self, positions_by_segment):
        # For each segment
        for index in range(len(positions_by_segment)):
            # Build a distance matrix between postion given and segments
            if len(positions_by_segment[index]) > 1:
                # Sort the positions with current tracker positions
                sorted_array = self.sort_by_distance_matrix(self.segments[index] ,positions_by_segment[index])

                # Remap position values before update
                corrected_values = self.mapper(self.segments[index])

                # Update each tracker
                for tracker_index in corrected_values:
                    self.trackers[tracker_index].update(sorted_array[tracker_index])

            elif len(positions_by_segment[index]) == 0:
                pass # Do nothing in case of empty array

            else:
                # Trivial case
                # Update only one tracker
                self.trackers[self.segments[index][0]].update(positions_by_segment[index][0])

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
        tl = vec2(math.inf, math.inf)
        br = vec2(-math.inf, -math.inf)
        b = BoundingBox(tl, br)
        for i in bboxes_indexes:
            b += bboxes[i]
        return (b.top_left, b.bottom_right)

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

    def mapper(self, local_pos):
        """
            local_positions:    list of positions localized by the objet detector
                                in each image segment defined by the parent boxes
            out_put:            list of global_positions of the objects inside
                                de segments
        """
        k = len(local_pos)
        if k != len(self.parent_bboxes):
            return
        global_pos = []
        for i in range(k):
            objs = []
            seg = local_pos[i]
            for pos in seg:
                objs.append(self.parent_bboxes[i].top_left + pos)
            global_pos.append(objs)
        return global_pos

    def get_serialized_objects(self):
        serialized = [self.tracker[i].position.to_list() for i in range(self.num_objects)]
        # for i in range(self.num_objects):
        #     serialized.append(self.tracker[i].position.to_list())
        return serialized
