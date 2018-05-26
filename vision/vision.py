#!/usr/bin/python
import sys
sys.path.append('../')
import COLORS
import cv2
import numpy as np
import time
from sklearn.cluster import MiniBatchKMeans
from camera.camera import Camera
from json_handler import JsonHandler
from params_setter import ParamsSetter

# @author Wellington Castro <wvmcastro>

# MACROS
NUM_CLUSTERS = 5

class Vision:

    def __init__(self, camera, params_file_name="", cluster_cfg=(5, 100, 500)):
        self.arena_vertices = []
        self.arena_size = ()
        self.arena_image = None
        self.arena_mask = None
        self.camera = camera
        self.params_file_name = params_file_name
        self.raw_image = None
        self.warp_matrix = None
        self.json_handler = JsonHandler()
        self.mbc_kmeans = MiniBatchKMeans(n_clusters = cluster_cfg[0], max_iter = cluster_cfg[1],
                                         batch_size=cluster_cfg[2])

        self.params_setter = ParamsSetter(camera, params_file_name)
        self.i = 0 # gambito
        self.LAB_MAX = ([255, 255, 255])
        self.lab_min = ([0, 0, 0])

        if self.params_file_name != "":
            self.load_params()

    def load_params(self):
        """ Loads the warp matrix and the arena vertices from the arena parameters file"""
        params = self.json_handler.read(self.params_file_name)

        self.arena_vertices = params['arena_vertices']
        self.warp_matrix = np.asarray(params['warp_matrix']).astype("float32")
        self.arena_size = (params['arena_size'][0], params['arena_size'][1])

        if 'value_min' in params:
            self.lab_min = params['value_min']

        self.get_mask()

    def warp_perspective(self):
        """ Takes the real world arena returned by camera and transforms it
            to a perfect retangle """
        self.arena_image = cv2.warpPerspective(self.raw_image, self.warp_matrix, self.arena_size)

    def get_mask(self):
        """ Creates the image where the mask will be stored """
        _arena_mask = np.zeros((self.arena_size[1], self.arena_size[0], 3), np.uint8)

        """ Here is drawn the arena shape with all pixles set to white color """
        cv2.fillConvexPoly(_arena_mask, np.asarray(self.arena_vertices), COLORS.WHITE)

        """ Gets the binary mask used in the bitwise operation of the
            set_dark_border function """
        self.arena_mask = cv2.inRange(_arena_mask, COLORS.WHITE, COLORS.WHITE)

    def set_dark_border(self):
        """ Applies a bitwise operation between the arena image and the arena mask
            to get rid of the pixels behind the goal lines"""
        self.arena_image = cv2.bitwise_and(self.arena_image, self.arena_image, mask=self.arena_mask)

    def get_lab_mask(self, lab_min, lab_max):
        """ Sets the dark and out of the arena pixels in the image to completely black pixels """
        temp_value_mask = cv2.inRange(self.arena_image, np.array(lab_min), np.array(lab_max))
        return temp_value_mask

    def cluster_pipeline(self):
        self.arena_image = cv2.cvtColor(self.arena_image, cv2.COLOR_BGR2LAB)

        """ Gets rid of bad pixles, ie: pixels out the field and most dark pixels """
        mask1 = self.get_lab_mask(self.lab_min, self.LAB_MAX)
        self.arena_image = cv2.bitwise_and(self.arena_image, self.arena_image, mask=np.bitwise_and(mask1, self.arena_mask))

        """ Select the interest pixels ie the colored ones that passed the threshold """
        indexes = np.argwhere(np.all(self.arena_image != (0,0,0), axis=2))
        samples = self.arena_image[indexes[:, 0], indexes[:, 1]]
        
        """ Feeds the minibatch kmeans once every two seconds"""
        if self.i % 120 == 0:
            self.mbc_kmeans.fit(samples)

        self.i = self.i + 1

        """ Predicts the label of the cluster that each sample belongs """
        labels = self.mbc_kmeans.predict(samples)

        """ Substitute every sample by the centroid of the clustes that it belongs 
            now the image has only n_clusters colors """
        self.arena_image[indexes[:,0], indexes[:,1]] = self.mbc_kmeans.cluster_centers_.astype("uint8")[labels]

        # return separated imgs 

    def get_frame(self):
        """ Takes the raw imagem from the camera and applies the warp perspective transform
            and the mask """
        self.raw_image = camera.read()
        
        if(self.params_file_name != ""):
            self.warp_perspective()
            self.cluster_pipeline()

        return self.arena_image



if __name__ == "__main__":

    camera = Camera(1, "../parameters/CAMERA_ELP-USBFHD01M-SFV.json")
    v = Vision(camera, "../parameters/ARENA.json")

    i = 0
    t0 = time.time()
    cv2.namedWindow('vision')
    while True: 
        i += 1
        arena = v.get_frame()
        cv2.imshow('vision', cv2.cvtColor(arena, cv2.COLOR_LAB2BGR))
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'): # exit
            break
        elif key == ord('c'): # open cropper
            tc0 = time.time()
            v.params_setter.run()
            v.load_params()
            t0 += time.time() - tc0
    
    print "framerate:", i / (time.time() - t0)
    cv2.destroyAllWindows()

    
