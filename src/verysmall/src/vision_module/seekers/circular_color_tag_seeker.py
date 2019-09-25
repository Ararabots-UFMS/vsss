from typing import List, Tuple
import cv2
import numpy as np
import math

IMAGE = np.ndarray
ROBOT_STATE = Tuple[int, np.array, float]

class CircularColorTagSeeker:
    def __init__(self, color_thresholds: List[Tuple[np.ndarray, np.ndarray]]):
        self._colors = color_thresholds
        
        self._radius_thresh = -1
        self._l_thresh = 0
        self._r_thresh = 0
        self._h = 0
        self._w = 0

        self._theta = math.pi / 4
    
    def seek(self, binary_img: IMAGE, color_img: IMAGE):
        if self._h == 0:
            self._h, self._w, *_ = binary_img.shape

        first_centroids = self.get_main_color_centroids(binary_img)
        slices = self.get_crop_areas(first_centroids)

        patches = []
        for s in slices:
            patches.append(color_img[s, ...])
        
        ids, second_centroids = self.segment_and_get_second_centroids(patches)

        robots = self.compute_robot_states(ids, first_centroids, second_centroids)
    
    def get_main_color_centroids(self, img) -> List[np.ndarray]:
        cnts = self.get_contours(img)
        
        centroids = []
        # compute all areas and centroids
        for cnt in cnts:
            m = cv2.moments(cnt)
            a = m["m00"]
            cx = m["m10"] / a
            cy = m["m01"] / a
            centroids.append((centroids, np.array([cx, cy])))
        
        sorted(centroids, key = lambda x: x[0], reverse=True)

        if len(centroids) < 2:
            n = len(centroids)
        else:
            self.l_thresh = 0.9 * centroids[0][0]
            self.r_thresh = 1.1 * centroids[0][0]

            n = 1
            for c in centroids[1:]:
                if self.l_thresh <= c[0] <= self.r_thresh:
                    n += 1
        
        if self._radius_thresh < 0 and n != 0:
            *_, self._radius_thresh = int(2.5 * cv2.minEnclosingCircle(cnts[0]))
        
        return [centroids[i][1] for i in range (n)]
    
    def get_contours(self, img: IMAGE):
        if cv2.__version__[0] == '4':
            cnts, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        else:
            _, cnts, *_ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        return cnts

    def get_crop_areas(self, centroids) -> List[Tuple[slice, slice]]:
        
        slices = []
        for centroid in centroids:
            x_min = max(0, centroid[0] - self._radius_thresh)
            x_max = min(self._w, centroid[0] + self._radius_thresh)
            y_min = max(0, centroid[1] - self._radius_thresh)
            y_max = min(self._h, centroid[1] + self._radius_thresh)

            slices.append((slice(y_min, y_max), slice(x_min, x_max)))
        
        return slices

    def segment_and_get_second_centroids(self, patches: List[IMAGE]) -> \
                                    Tuple[List[int], List[np.ndarray]]:
        
        centroids = []
        ids = []
        for i, color in enumerate(self._colors):
            for patch in patches:
                thresholded = cv2.inRange(patch, color[0], color[1])
                found = False
                if np.any(thresholded):
                    found  = True
                    break
                
                cnt = self.get_contours(thresholded)[0]
                if found == True:
                    m = cv2.moments(cnt)
                    a = m["m00"]
                    cx = m["m10"] / a
                    cy = m["m01"] / a
                    centroids.append(np.array([cx, cy]))
                    ids.append(i)
        
        return ids, centroids
    
    def compute_robot_states(self, ids: List[int], 
                             first_centroids: List[np.ndarray], 
                             second_centroids: List[np.ndarray]) -> \
                             List[ROBOT_STATE]:
        robots = []
        i = 0
        for c1, c2 in zip(first_centroids, second_centroids):
            c = (c1 + c2) / 2
            vec = c2 - c1
            angle = math.atan2(-vec[1], vec[0])  - self._theta
            robots.append((ids[i], c, angle))

            i += 1
        
        return robots


            
                    

