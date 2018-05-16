#!/usr/bin/python
import cv2
import json
import numpy as np
import time
from sklearn.cluster import MiniBatchKMeans
from camera.camera import Camera

# MACROS
NUM_CLUSTERS = 5
HSV_MIN = np.array([])
HSV_MAX = np.array([255,255,255])

# TODO
    # reload online parametros
    # selecionar range hsv
    # adicionar range hsv arena.json

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
        self.mbc_kmeans = MiniBatchKMeans(n_clusters = cluster_cfg[0], max_iter = cluster_cfg[1],
                                         batch_size=cluster_cfg[2])

        self.HSV_MAX = HSV_MAX
        self.HSV_MIN = HSV_MIN

        self.COLORS = dict()
        self.colors_dict()

        if self.params_file_name != "":
            self.load_params()
            self.get_mask()


    def colors_dict(self):
        self.COLORS['WHITE'] = (255,255,255)
        self.COLORS['BLUE'] = (255, 0, 0)
        self.COLORS['YELLOW'] = (0, 255, 255)


    def load_params(self):
        """ Loads the warp matrix and the arena vertices from the arena parameters file"""
        params_file = open(self.params_file_name, "r")
        params = json.loads(params_file.read())
        params_file.close()

        self.arena_vertices = params['arena_vertices']
        self.warp_matrix = np.asarray(params['warp_matrix']).astype("float32")
        self.arena_size = (params['arena_size'][0], params['arena_size'][1])

    def warp_perspective(self):
        """ Takes the real world arena returned by camera and transforms it
            to a perfect retangle """
        self.arena_image = cv2.warpPerspective(self.raw_image, self.warp_matrix, self.arena_size)

    def get_mask(self):
        """ Creates the image where the mask will be stored """
        _arena_mask = np.zeros((self.arena_size[1], self.arena_size[0], 3), np.uint8)

        """ Here is drawn the arena shape with all pixles set to white color """
        cv2.fillConvexPoly(_arena_mask, np.asarray(self.arena_vertices), self.COLORS['WHITE'])

        """ Gets the binary mask used in the bitwise operation of the
            set_dark_border function """
        self.arena_mask = cv2.inRange(_arena_mask, self.COLORS['WHITE'], self.COLORS['WHITE'])

    def set_dark_border(self):
        """ Applies a bitwise operation between the arena image and the arena mask
            to get rid of the pixels behind the goal lines"""
        self.arena_image = cv2.bitwise_and(self.arena_image, self.arena_image, mask=self.arena_mask)

    def get_frame(self):
        """ Takes the raw imagem from the camera and applies the warp perspective transform
            and the mask """
        self.raw_image = camera.read()
        self.warp_perspective()
        self.set_dark_border()
        return self.arena_image


    def dark2black(self):
        """ Sets the dark pixels in the image to completely black pixels """
        temp_img = cv2.cvtColor(self.arena_image, cv2.COLOR_BGR2HSV)
        temp_value_mask = cv2.inRange(_image, self.HSV_MIN, self.HSV_MAX)
        temp_img = cv2.bitwise_and(temp_img, temp_img, mask=temp_value_mask)
        self.arena_image = cv2.cvtColor(self.arena_image, cv2.COLOR_HSV2BGR)

    def select_colored_pixels(self):
        """ Takes only the colored pixels in the image to feed the MiniBatchKmeans """
        img = self.arena_image

        """ Reshape the 2D image to something like 1D image """
        img = img.reshape(img.shape[0] * img.shape[1], 3)

        # TODO: CONFERIR AXIS=1
        indexes = np.where(np.all(img != (0,0,0), axis=1))

        colored = img[indexes]
        return indexes, colored






if __name__ == "__main__":

    camera = Camera(1, "../parameters/CAMERA_ELP-USBFHD01M-SFV.json")
    v = Vision(camera, "../parameters/ARENA.json")

    i = 0
    t0 = time.time()
    cv2.namedWindow('vision')
    while True: 
        i += 1
        arena = v.get_frame()
        cv2.imshow('vision', arena)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    print "framerate:", i / (time.time() - t0)
    cv2.destroyAllWindows()

    
