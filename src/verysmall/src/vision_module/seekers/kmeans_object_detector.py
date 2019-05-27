import numpy as np
import cv2
from sklearn.cluster import KMeans

from vision_module.seekers.seeker_data_structures import *
from vision_module.seekers.obj_detector import ObjDetector

class KmeansObjectDetector(ObjDetector):
    """A simple detector using find contours
        :param : None"""
    def __init__(self):
        super().__init__()
        self.obj_size = -1
        self.max_iter = 30
        self.number_of_jobs = 1
        self.distance_threshold = 2.5

    def update_obj_size(self, cnt):
        _, _, w, h = cv2.boundingRect(cnt)
        self.obj_size = max(max(w,h), self.obj_size)

    def seek(self, segments, objects_per_segment):
        """
            This function receives a list of binary images and list of number of objects per segment
            and return its centers positions per segment using kmeans implementation
            :param segments: np.array([uint8]).shape([m,n])
            :param objects_per_segment: Bool
            :return: np.array([float, float]).shape([k, 2]) object has the position of the center of each object in img
        """

        # Our return value
        centroids_per_segment = []

        number_of_segments = len(segments)

        # Iterate over segments
        for index in range(number_of_segments):
            # Insantiate KMeans for this segment
            kmeans = KMeans(n_clusters=objects_per_segment[index], n_init=1, max_iter=self.max_iter,
            precompute_distances=True, n_jobs=self.number_of_jobs)

            # Find contours in segment
            cnts = cv2.findContours(segments[index], cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]
            num_cnts = len(cnts)

            # Variable to store object positions
            center_of_objects = None
            #print num_cnts
            if num_cnts > 0:

                cnts_array = np.array(cnts[0]).reshape(-1, 2)
                if self.should_calculate_size:
                    self.update_obj_size((cnts[0]))

                for i in range(1, num_cnts):
                    cnts_array = np.vstack([cnts_array, np.array(cnts[i]).reshape(-1, 2)])
                    if self.should_calculate_size:
                        self.update_obj_size(cnts[i])

                if cnts_array.shape[0] > objects_per_segment[index]:
                    kmeans.fit(cnts_array)
                    center_of_objects = kmeans.cluster_centers_

            obj_states_in_segment = []
            for object_center in center_of_objects:
                obj_state = ObjState()
                x,y = object_center
                obj_state.set_pos(x, y)
                obj_states_in_segment.append(obj_state)

            # Store found objects in return value
            centroids_per_segment.append(obj_states_in_segment)

        return centroids_per_segment
