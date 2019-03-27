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

    def __sub__(self, vec):
        _v = Vec2(self.x - vec.x, self.y - vec.y)
        return _v

    def __rsub__(self, vec):
        _v = Vec2(vec.x - self.x, vec.y - self.y)
        return _v

    def __mul__(self, alpha):
        """ alpha is a scalar number """
        return Vec2(alpha*self.x, alpha*self.y)

    def __rmul__(self, alpha):
        """ alpha is a scalar number """
        return Vec2(alpha*self.x, alpha*self.y)

    def __div__(self, alpha):
        return Vec2(self.x/alpha, self.y/alpha)

    def __rdiv__(self, alpha):
        _v = Vec2(self.x/alpha, self.y/alpha)
        return _v

    def __repr__(self):
        return "Vec2(%r, %r)" % (self.x, self.y)

    def __abs__(self):
        return math.sqrt(self.x*self.x + self.y*self.y)

    def __neg__(self):
        return Vec2(-self.x, -self.y)


class ObjState():
    def __init__(self, id):
        self.id = id
        self.pos = Vec2()
        self.speed = Vec2()
        self.orientation = 0
        # Size should be the largest axis of the object
        self.size = math.inf

    def set_pos(self, x, y):
        self.pos.x = x
        self.pos.y = y

    def set_speed(self, vx, vy):
        self.speed.x = vx
        self.speed.y = vy

    def set_orientation(self, theta):
        # For convention the orientation is always taken in relation
        # with the x axis
        self.orientation = theta


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

    def __repr__(self):
        tl = self.top_left
        br = self.bottom_right
        return "[[%r, %r], [%r, %r]]" % (tl.x, tl.y, br.x, br.y)


class Tracker():
    def __init__(self, seeker, obj_id=-1):
        self.obj = ObjState(obj_id)
        self.t = 0.0
        self.bbox = BoundingBox()
        self.my_seeker = seeker

    def set_pos(self, x, y):
        self.obj.position.x = x
        self.obj.position.y = y

    def set_obj_size(self):
        # TODO:
        pass

    def update(self, position):
        old_p = self.obj.position
        self.obj.position = position
        t0 = self.t
        self.t = time()
        self.obj.speed = (1/(self.t - t0))*(old_p - self.obj.position)

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
        return self.bbox
        pass


class NewSeeker:
    def __init__(self, num_objects, obj_detector):
        self.num_objects = num_objects
        self.obj_detector = obj_detector
        self.trackers = [Tracker(self, i) for i in range(self.num_objects)]
        self.segments = []
        self.parent_bboxes = []

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
        # TODO: TEM QUE OLHAR O NOME DESSA FUNCAO
        objs_in_segs = self.obj_detector.seek(img_segments, [len(seg) for seg in self.segments])
        self.update(objs_in_segs)

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
        serialized = []
        for i in range(self.num_objects):
            serialized.append(self.tracker[i].position.to_list())
        return serialized
