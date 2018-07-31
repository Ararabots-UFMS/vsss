#!/usr/bin/python
import sys
sys.path.append('../../')

import cv2
import os
import sys
import numpy as np
from threading import Thread
from time import sleep

# Top level imports
old_path = sys.path[0]
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT']+"src/"
from utils.json_handler import JsonHandler
sys.path[0] = old_path

# @author Wellington Castro <wvmcastro>
# The threading code is highly inspired by Adrian Rosebrock approach
# https://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/

class Camera:
    def __init__(self, device_id, params_file_name="", lens_correction=True, threading=False):
        self.id = device_id
        self.lens_correction = lens_correction
        self.params_file_name = params_file_name
        self.capture = cv2.VideoCapture(self.id)
        self.json_handler = JsonHandler()
        self.frame = None

        # This flag ensures that in threading mode the frame returned now is different
        # of the frame returned by the previous read
        self.last_state = 0

        """ Controls if the thread should run """
        self.thread_stopped = True

        if self.params_file_name != "":
            self.load_params()
            self.set_frame_size(self.frame_width, self.frame_height)

        if threading == True:
            self.start()

        """ Give some time for the camera auto calibrate its sensors """
        sleep(1)

    def start(self):
        """ starts a separated thread to execute read() from OpenCV """
        self.thread_stopped = False
        self.semaphore = 0
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def stop(self):
        """ Stops the thread created by start() """
        if not self.thread_stopped:
            self.thread_stopped = True

    """ Simple functions to implement some kind of semaphore to deal with the read and
        write frame process """
    def semaphore_up(self):
        self.semaphore = 1

    def semaphore_down(self):
        self.semaphore = 0

    def semaphore_isUp(self):
        return self.semaphore

    def update(self):
        """ This is the target function for the thread, this will read from OpenCV forever
        until stop() is called """
        while not self.thread_stopped:
            """ Reads the next frame and apply correction it is on """
            ret, frame = self.capture.read()
            if ret == True:
                if self.lens_correction == True:
                    frame = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)
                self.semaphore_up()
                self.frame = frame
                self.last_state = 1
                self.semaphore_down()

    def threaded_read(self):
        """ If the frame is being updated at the moment of reading it waits until
            the update is done """
        while self.semaphore_isUp() or self.last_state == 0:
            pass
        self.last_state = 0
        return self.frame

    def non_threaded_read(self):
        """ Read function for the non threaded version """
        try:
            ret, self.frame = self.capture.read()
        except:
            e = sys.exc_info()[0]
            print "Erro: %s", e

        if self.lens_correction:
            self.frame = cv2.remap(self.frame, self.mapx, self.mapy, cv2.INTER_LINEAR)

        return self.frame

    def read(self):
        if self.thread_stopped == True:
            return self.non_threaded_read()
        else:
            return self.threaded_read()

    def load_params(self):
    	""" Loads the parameters of the camera from a json """
        params = self.json_handler.read(self.params_file_name)

        """ mapx and mapy are the matrix with the lens correction map """
        self.mapx = np.asarray(params['matrix_x']).astype("float32")
        self.mapy = np.asarray(params['matrix_y']).astype("float32")

        """ The frame width and height """
        self.frame_width = int(params['default_frame_width'])
        self.frame_height = int(params['default_frame_height'])

        """ The matrix of the intrinsic parameters and the vector of distortion coefficients """
        self.camera_matrix = np.asarray(params['cam_matrix'])
        self.dist_vector = np.asarray(params['dist_vector'])

    def set_frame_size(self, width, height):
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
