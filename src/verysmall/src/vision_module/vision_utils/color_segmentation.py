from utils.json_handler import JsonHandler
from vision_module.camera_module.camera import Camera
import cv2
import numpy as np
import copy
import COLORS
import os

# @author Wellington Castro <wvmcastro>

class ColorSegmentation:

    def __init__(self, cam, color_params_file=""):

        self.json_handler = JsonHandler()
        self.params_file = color_params_file
        self.camera = cam
        self.last_key = None
        self.reset_all()
        self.load_params()

    def reset_all(self):
        components = ["blue", "yellow", "ball", "temp"]
        for c in components:
            self.reset(c)

    def reset(self, component):
        if component == "blue":
            self.blue_min = np.uint8([255, 255, 255])
            self.blue_max = np.uint8([0, 0, 0])
        elif component == "yellow":
            self.yellow_min = np.uint8([255, 255, 255])
            self.yellow_max = np.uint8([0, 0, 0])
        elif component == "ball":
            self.ball_min = np.uint8([255, 255, 255])
            self.ball_max = np.uint8([0, 0, 0])
        elif component == "temp":
            self.temp_min = np.uint8([])
            self.temp_max = np.uint8([])

    def load_params(self):
        if os.path.isfile(self.params_file):
            params = self.json_handler.read(self.params_file)
            if "hsv_ball_min" in params and "hsv_ball_max" in params:
                self.ball_min = np.asarray(params['hsv_ball_min']).astype("uint8")
                self.ball_max = np.asarray(params['hsv_ball_max']).astype("uint8")
            if "hsv_blue_min" in params and "hsv_blue_max" in params:
                self.blue_min = np.asarray(params['hsv_blue_min']).astype("uint8")
                self.blue_max = np.asarray(params['hsv_blue_max']).astype("uint8")
            if "hsv_yellow_min" in params and "hsv_yellow_max" in params:
                self.yellow_min = np.asarray(params['hsv_yellow_min']).astype("uint8")
                self.yellow_max = np.asarray(params['hsv_yellow_max']).astype("uint8")
        else:
            print("The is no previous params to load")

    def write_params(self):
        params = dict()
        params["hsv_ball_min"] = self.ball_min.tolist()
        params["hsv_ball_max"] = self.ball_max.tolist()
        params["hsv_blue_min"] = self.blue_min.tolist()
        params["hsv_blue_max"] = self.blue_max.tolist()
        params["hsv_yellow_min"] = self.yellow_min.tolist()
        params["hsv_yellow_max"] = self.yellow_max.tolist()

        self.json_handler.write(params, self.params_file)

    def onMouse_get_color(self, event, x, y, flags, arg):

        if self.temp_min.size > 0 and self.temp_max.size > 0:

            if event == cv2.EVENT_LBUTTONUP and (flags & cv2.EVENT_FLAG_SHIFTKEY):
                self.temp_min = np.minimum(self.temp_min, self.hsv_frame[y,x,:])
                self.temp_max = np.maximum(self.temp_max, self.hsv_frame[y,x,:])
                self.visited.append([y,x])
            elif event == cv2.EVENT_LBUTTONUP:
                for i in range(y-1, y+2):
                    for j in range(x-1, x+2):
                        self.temp_min = np.minimum(self.temp_min, self.hsv_frame[i,j,:])
                        self.temp_max = np.maximum(self.temp_max, self.hsv_frame[i,j,:])
                        self.visited.append([i,j])

            if self.last_key == 'b':
                self.ball_min = self.temp_min
                self.ball_max = self.temp_max
            elif self.last_key == 'a':
                self.blue_min = self.temp_min
                self.blue_max = self.temp_max
            elif self.last_key == 'y':
                self.yellow_min = self.temp_min
                self.yellow_max = self.temp_max

    def onMouse_no_mode(self, event, x, y, flags, arg):
        pass

    def draw_selected(self, mask):
        pos = np.argwhere(mask == 255)
        self.frame[pos[:, 0], pos[:, 1], :] = COLORS.RED

    def draw_mask(self):
        mask = cv2.inRange(self.hsv_frame, self.temp_min, self.temp_max)
        return mask

    def erase_previous(self):
        if self.last_key == 'a':
            self.reset("blue")
        elif self.last_key == 'y':
            self.reset("yellow")
        else:
            self.reset("ball")

        self.reset("temp")

    def run(self):
        window_name = "color segmentation"
        cv2.namedWindow(window_name)

        exit = False
        self.visited = []
        frame = self.camera.read()
        aux_mask = np.empty(frame.shape).astype("uint8")

        while(not exit):
            self.frame = self.camera.read()
            self.hsv_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

            key  = cv2.waitKey(1) & 0xFF

            if self.temp_min.size > 0 and self.temp_max.size > 0:
                self.mask = self.draw_mask()
                aux_mask[:,:,0],aux_mask[:,:,1], aux_mask[:,:,2] = self.mask, self.mask, self.mask
                self.draw_selected(self.mask)
                cv2.imshow("Segment", aux_mask)
            else:
                cv2.destroyWindow("Segment")

            cv2.imshow(window_name, self.frame)

            if key in set([ord('b'), ord('a'), ord('y')]):
                self.visited = []
                cv2.setMouseCallback(window_name, self.onMouse_get_color)

            if key == ord('q'):
                exit = True
            elif key == ord('b'): #ball
                self.temp_max = self.ball_max
                self.temp_min = self.ball_min
                self.last_key = 'b'
            elif key == ord('a'): # blue
                self.temp_max = self.blue_max
                self.temp_min = self.blue_min
                self.last_key = 'a'
            elif key == ord('y'): # yellow
                self.temp_max = self.yellow_max
                self.temp_min = self.yellow_min
                self.last_key = 'y'
            elif key == ord(' '):
                print("LAST KEY", self.last_key)
                self.erase_previous()
                self.visited = []
            elif key == ord('s'):
                self.write_params()
                print("saving")
            elif key == 27: # no mode
                self.reset("temp")
                cv2.setMouseCallback(window_name, self.onMouse_no_mode)
                self.visited = []
                self.last_key = None

        cv2.destroyAllWindows()

CAMERA_ID = 0
CAMERA_PARAMS_PATH = "../../parameters/CAMERA_ELP-USBFHD01M-SFV.json"

if __name__ == "__main__":

    params_file = "../../parameters/COLORS.json"
    camera = Camera(CAMERA_ID, CAMERA_PARAMS_PATH)
    c = ColorSegmentation(camera, params_file)
    c.run()
