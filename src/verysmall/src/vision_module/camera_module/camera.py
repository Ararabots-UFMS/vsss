from typing import Union
import cv2
import numpy as np
from multiprocessing import Queue, Process
from utils.json_handler import JsonHandler
import time

# @author Wellington Castro <wvmcastro>

class Camera:
    def __init__(self, device_id: Union[int, str] = 0,
                 params_file_name: str = "",
                 lens_correction: bool = True,
                 threading: bool = False):

        self.id = device_id
        self.lens_correction = lens_correction
        self.params_file_name = params_file_name
        self.threading = threading
        self.capture = cv2.VideoCapture(self.id)
        self.thread_stopped = True

        self.json_handler = JsonHandler()
        self.frame = None

        if self.params_file_name != "":
            self._load_params()
            self.set_frame_size(self.frame_width, self.frame_height)
            self.capture_frame = self._capture_and_correct_frame
        else:
            self.capture_frame = self._capture_frame

        self._calibrate_sensors()

        if self.threading == True:
            self.buffer = Queue(maxsize=1)
            self._start_reading_process()
            self._read = self._threaded_read
        else:
            self._read = self._sequential_read

    def _calibrate_sensors(self):
        for i in range(10):
            self.capture_frame()

    def _start_reading_process(self) -> None:
        self.thread_stopped = False
        p = Process(target=self._update, args=())
        p.daemon = True
        p.start()

    def stop(self) -> None:
        if not self.thread_stopped:
            self.thread_stopped = True

    def _update(self) -> None:
        while not self.thread_stopped:
            frame = self.capture_frame()
            self.buffer.put(frame)

    def _capture_frame(self) -> np.ndarray:
        _, frame = self.capture.read()
        return frame

    def _capture_and_correct_frame(self) -> np.ndarray:
        _, frame = self.capture.read()
        frame = cv2.remap(frame, self.converted_mapx, self.converted_mapy, cv2.INTER_LINEAR)
        return frame

    def read(self) -> np.ndarray:
        try:
            return self._read()
        except IOError as e:
            print("ERROR ", end='')
            print("Camera: ", e)
        except Exception as e:
            print("ERROR ", end='')
            print("Camera: ", e)

    def _threaded_read(self) -> np.ndarray:
        return self.buffer.get(block=True)

    def _sequential_read(self) -> np.ndarray:
        return self.capture_frame()

    def _load_params(self) -> None:
        """ Loads the parameters of the camera from a json """
        params = self.json_handler.read(self.params_file_name)

        """ mapx and mapy are the matrix with the lens correction map """
        self.mapx = np.asarray(params['matrix_x']).astype("float32")
        self.mapy = np.asarray(params['matrix_y']).astype("float32")

        self.converted_mapx, self.converted_mapy = cv2.convertMaps(self.mapx, self.mapy, cv2.CV_16SC2)
        """ The frame width and height """
        self.frame_width = int(params['default_frame_width'])
        self.frame_height = int(params['default_frame_height'])

        """ The matrix of the intrinsic parameters and the vector of distortion coefficients """
        self.camera_matrix = np.asarray(params['cam_matrix'])
        self.dist_vector = np.asarray(params['dist_vector'])

    def set_frame_size(self, width: int, height: int) -> None:
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def __repr__(self) -> str:
        return "Camera(device_id=%r, params_file_name=%r, lens_correction=%r, " \
               "threading=%r)" % (self.id, self.params_file_name,
                                  self.lens_correction, self.threading)
