import numpy as np 
import cv2

class KmeansObjectDetector():
    """A simple detector using find contours
        :param : None"""
    def __init__(self):
        self.max_iter = 30
        self.number_of_jobs = 1
        self.distance_threshold = 2.5



    def seek(self, segments, objects_per_segment):
        """
            This function receives a list of binary images and list of number of objects per segment
            and return its centers positions per segment using kmeans implementation
            :param segments: np.array([uint8]).shape([m,n])
            :param objects_per_segment: Bool
            :return: np.array([float, float]).shape([k, 2]) object has the position of the center of each object in img
        """

        # Our return value
        centroids_per_segment = np.array([])
        
        number_of_segments = len(segments)

        # Iterate over segments
        for index in xrange(number_of_segments):

            # Insantiate KMeans for this segment
            kmeans = KMeans(n_clusters=objects_per_segment[index], n_init=1, max_iter=self.max_iter,
            precompute_distances=True, n_jobs=self.number_of_jobs)

            # Find contours in segment
            cnts = cv2.findContours(segments[index], cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]
            num_cnts = len(cnts)

            # Variable to store object positions
            center_of_objects = None

            if num_cnts > 0:

                cnts_array = np.array(cnts[0]).reshape(-1, 2)

                for i in xrange(1, num_cnts):
                    cnts_array = np.vstack([cnts_array, np.array(cnts[i]).reshape(-1, 2)])

                if cnts_array.shape[0] > objects_per_segment[index]:
                    first_iteration = 1
                    if np.all(center_of_objects != None):
                        first_iteration = 0
                        kmeans.init = center_of_objects

                    kmeans.fit(cnts_array)
                    newObjects = kmeans.cluster_centers_

                    if not first_iteration and np.all(center_of_objects != None):
                        diff = newObjects - center_of_objects
                        distances = np.linalg.norm(diff, axis=1)
                        changes = np.where(distances > self.distance_threshold)[0]
                        center_of_objects[changes,:] = kmeans.cluster_centers_[changes,:]
                    else:
                        center_of_objects = newObjects

            # Store found objects in return value
            centroids_per_segment = np.append( centroids_per_segment, center_of_objects)

        return centroids_per_segment