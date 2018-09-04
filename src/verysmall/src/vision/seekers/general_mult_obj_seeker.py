import numpy as np
import cv2
from sklearn.cluster import KMeans
import rospy
import time
# @author Wellington Castro <wvmcastro>

class GeneralMultObjSeeker:

    def __init__(self, num_objects):
        self.num_objects = num_objects
        self.kmeans = KMeans(n_clusters=self.num_objects, n_init=1, max_iter=30,
        precompute_distances=True, n_jobs=1)
        self.objects = None

    def seek(self, img):
        cnts = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]
        centroids_list = []

        if len(cnts):
            for cnt in cnts:
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    c_x = int(M['m10'] / M['m00'])
                    c_y = int(M['m01'] / M['m00'])
                    centroids_list.append(np.asarray([c_x, c_y]))

            if len(centroids_list) >= self.num_objects:
                centroids = np.asarray(centroids_list).reshape(-1, 2)

                if np.all(self.objects != None):
                    self.kmeans.init = self.objects
                self.kmeans.fit(centroids)
                self.objects = self.kmeans.cluster_centers_

        return self.objects

    def reset(self):
        self.kmeans.init = 'k-means++'

        self.objects = None
