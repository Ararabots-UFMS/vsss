#!/usr/bin/python
import sys
sys.path.append('../../')
import cv2
import os
import sys
import numpy as np
from threading import Thread
from json_handler import JsonHandler

# @author Wellington Castro <wvmcastro>

class Camera:
    def __init__(self, device_id, params_file_name="", lens_correction=True):
        self.id = device_id
        self.lens_correction = lens_correction
        self.params_file_name = params_file_name
        self.capture = cv2.VideoCapture(self.id)
        self.json_handler = JsonHandler()
        self.frame = None

        if self.params_file_name != "":
            self.load_params()
            self.set_frame_size(self.frame_width, self.frame_height)

    def read(self):
        try:
            ret, self.frame = self.capture.read()
        except:
            e = sys.exc_info()[0]
            print "Erro: %s", e

        if self.lens_correction and self.params_file_name != "":
            self.frame = cv2.remap(self.frame, self.mapx, self.mapy, cv2.INTER_LINEAR)
        
        return self.frame
        
    def load_params(self):
    	""" Loads the parameters of the camera from a json """
        params = self.json_handler.read(self.params_file_name)

        """ mapx and mapy are the matrix with the lens correction map """
        self.mapx = np.asarray(params['matrix_x']).astype("float32")
        self.mapy = np.asarray(params['matrix_y']).astype("float32")
        self.frame_width = int(params['default_frame_width'])
        self.frame_height = int(params['default_frame_height'])

    def set_frame_size(self, width, height):
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)