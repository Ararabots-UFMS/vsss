#!/usr/bin/python
import sys
sys.path.append('../')
import COLORS
import cv2
import numpy as np
import time
from sklearn.cluster import MiniBatchKMeans
from camera.camera import Camera
from utils.json_handler import JsonHandler
from vision_utils.params_setter import ParamsSetter

# @author Wellington Castro <wvmcastro>


class Vision:
    def __init__(self, camera, params_file_name="", colors_params = "", method="", cluster_cfg=(5, 100, 500)):
        self.arena_vertices = []
        self.arena_size = ()
        self.arena_image = None
        self.arena_mask = None
        self.camera = camera
        self.params_file_name = params_file_name
        self.raw_image = None
        self.warp_matrix = None
        self.json_handler = JsonHandler()

        if method == "clustering":
            self.mbc_kmeans = MiniBatchKMeans(n_clusters = cluster_cfg[0], max_iter = cluster_cfg[1],
                                         batch_size=cluster_cfg[2])
            self.LAB_MAX = ([255, 255, 255])
        elif method == "color_segmentation":
            self.colors_params_file = colors_params
            self.load_colors_params()
        else:
            print "Method not recognized!"

        self.params_setter = ParamsSetter(camera, params_file_name)
        self.i = 0 # gambito
        self.method = method

        if self.params_file_name != "":
            self.load_params()

    def load_params(self):
        """ Loads the warp matrix and the arena vertices from the arena parameters file"""
        params = self.json_handler.read(self.params_file_name)

        self.arena_vertices = params['arena_vertices']
        self.warp_matrix = np.asarray(params['warp_matrix']).astype("float32")
        self.arena_size = (params['arena_size'][0], params['arena_size'][1])

        if 'value_min' in params and self.method == "clustering":
            self.lab_min = params['value_min']

        self.create_mask()

    def load_colors_params(self, colors_params=None):
        """ Loads the colors thresholds from the json file or from a given
            params dictionary """
        if colors_params is None:
            colors_params = self.json_handler.read(self.colors_params_file)

        self.yellow_min = np.asarray(colors_params['hsv_yellow_min']).astype("uint8")
        self.yellow_max = np.asarray(colors_params['hsv_yellow_max']).astype("uint8")
        self.blue_min = np.asarray(colors_params['hsv_blue_min']).astype("uint8")
        self.blue_max = np.asarray(colors_params['hsv_blue_max']).astype("uint8")
        self.ball_min = np.asarray(colors_params['hsv_ball_min']).astype("uint8")
        self.ball_max = np.asarray(colors_params['hsv_ball_max']).astype("uint8")

    def warp_perspective(self):
        """ Takes the real world arena returned by camera and transforms it
            to a perfect retangle """
        self.arena_image = cv2.warpPerspective(self.raw_image, self.warp_matrix, self.arena_size)

    def create_mask(self):
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

    def get_filter(self, img, lower, upper):
        """ Returns a binary images where the white pixels corresponds to the pixels
        of the img that are between lower and upper threshold """
        temp_value_mask = cv2.inRange(img, np.array(lower), np.array(upper))
        return temp_value_mask

    def cluster_pipeline(self):
        """ Implements all the stepes required to segment color bases in a clustering approach
            with the pixels in the LAB color space """
        self.arena_image = cv2.cvtColor(self.arena_image, cv2.COLOR_BGR2LAB)

        """ Gets rid of bad pixles, ie: pixels out the field and most dark pixels """
        mask1 = self.get_filter(self.arena_image, self.lab_min, self.LAB_MAX)
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

    def color_seg_pipeline(self):
        """ Wait until the color parameters are used """
        self.arena_image = cv2.cvtColor(self.arena_image, cv2.COLOR_BGR2HSV)
        self.blue_seg = self.get_filter(self.arena_image, self.blue_min, self.blue_max)
        self.yellow_seg = self.get_filter(self.arena_image, self.yellow_min, self.yellow_max)
        self.ball_seg = self.get_filter(self.arena_image, self.ball_min, self.ball_max)

    def get_frame(self):
        """ Takes the raw imagem from the camera and applies the warp perspective transform
            and the mask """
        self.raw_image = camera.read()
        self.warp_perspective()
        
        if self.method == "clustering":
            self.cluster_pipeline()
        elif self.method  == "color_segmentation":
            self.color_seg_pipeline()

        return self.arena_image



if __name__ == "__main__":

    arena_params = "../parameters/ARENA.json"
    colors_params = "../parameters/COLORS.json"
    camera = Camera(1, "../parameters/CAMERA_ELP-USBFHD01M-SFV.json", threading=False)
    v = Vision(camera, arena_params, colors_params, method="color_segmentation")

    i = 0
    t0 = time.time()
    cv2.namedWindow('control')
    show = False
    while True: 
        i += 1
        arena = v.get_frame()
        key = cv2.waitKey(1) & 0xFF
        if show:
            cv2.imshow('vision', cv2.cvtColor(arena, cv2.COLOR_HSV2BGR))
            cv2.imshow('segs', np.hstack([v.blue_seg, v.yellow_seg, v.ball_seg]))
        if key == ord('q'): # exit
            camera.stop()
            break
        elif key == ord('s'): #show/hide
            show = not show
            if show == False:
                cv2.destroyWindow("vision")
                cv2.destroyWindow("segs")
        elif key == ord('c'): # open cropper
            tc0 = time.time()
            v.params_setter.run()
            v.load_params()
            t0 += time.time() - tc0            
    
    print "framerate:", i / (time.time() - t0)
    cv2.destroyAllWindows()

    
