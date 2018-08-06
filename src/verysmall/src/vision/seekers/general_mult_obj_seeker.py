import numpy as np
import cv2
from sklearn.cluster import KMeans

# @author Wellington Castro <wvmcastro>

class GeneralMultObjSeeker:

    def __init__(self, num_objects):
        self.num_objects = num_objects
        self.kmeans = KMeans(n_clusters=self.num_objects, n_init=1, max_iter=50,
        precompute_distances=True, n_jobs=1)

        self.objects = None

    def seek(self, img):
        cnts = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]
        num_cnts = len(cnts)

        if num_cnts > 0:
            cnts_array = np.array(cnts[0]).reshape(-1, 2)

            for i in xrange(1, num_cnts):
                cnts_array = np.vstack([cnts_array, np.array(cnts[i]).reshape(-1, 2)])

            if cnts_array.shape[0] > self.num_objects:
                if np.all(self.objects != None):
                    self.kmeans.init = self.objects
                    self.kmeans.fit(cnts_array)
                else:
                    self.kmeans.fit(cnts_array)

                self.objects = self.kmeans.cluster_centers_

        return self.objects
