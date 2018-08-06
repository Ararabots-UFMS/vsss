#!/usr/bin/python
import rospy
import sys
import COLORS
import cv2
import numpy as np
import time
from camera.camera import Camera
from threading import Thread
from vision_utils.params_setter import ParamsSetter

from seekers.things_seeker import HawkEye
from seekers.things_seeker import Things

# Top level imports
import os
old_path = sys.path[0]
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT']+"src/"
from ROS.ros_vision_publisher import RosVisionPublisher
from utils.json_handler import JsonHandler
sys.path[0] = old_path

# @author Wellington Castro <wvmcastro>

HEIGHT = 1


class Vision:

    def __init__(self, camera, adv_robots, home_color, home_robots,
    home_tag="aruco", params_file_name="", colors_params = "", method=""):

        # This object will be responsible for publish the game state info
        # at the bus. Mercury is the gods messenger
        self.mercury = RosVisionPublisher(True)

        self.json_handler = JsonHandler()
        self.camera = camera
        self.params_file_name = params_file_name
        self.home_color = home_color
        self.home_robots = home_robots
        self.adv_robots = adv_robots
        self.home_tag = home_tag

        self.arena_vertices = []
        self.arena_size = ()
        self.arena_image = None
        self.arena_mask = None
        self.raw_image = None
        self.warp_matrix = None
        self.pipeline = None
        self.fps = None
        self.t0 = None

        self.computed_frames = 0

        self.origin = None
        self.conversion_factor = None
        self.game_on = False

        self.finish = False

        # Creates the lists to the home team and the adversary
        self.home_team = [Things() for _ in xrange(home_robots)]
        self.adv_team =[Things() for _ in xrange(adv_robots)]

        # Object to store ball info
        self.ball = Things()

        # Initialize the vitamins according to the chosen method
        if method == "color_segmentation":
            self.colors_params_file = colors_params
            self.load_colors_params()
            self.pipeline = self.color_seg_pipeline
        else:
            print "Method not recognized!"

        self.params_setter = ParamsSetter(camera, params_file_name)

        if self.params_file_name != "":
            self.load_params()

        self.set_origin_and_factor()

        # Gets a initialization frame
        self.raw_image = camera.read()
        self.warp_perspective()

        # The hawk eye object will be responsible to locate and identify all
        # objects at the field
        hawk_eye_extra_params = []
        if self.home_tag == "aruco":
            hawk_eye_extra_params = [camera.camera_matrix, camera.dist_vector]

        self.hawk_eye = HawkEye(self.origin, self.conversion_factor, self.home_tag,
        self.home_robots, self.adv_robots, self.arena_image.shape, hawk_eye_extra_params)


    def start(self):
        self.game_on = True

    def pause(self):
        self.game_on = False

    def stop(self):
        self.pause()
        self.finish = True

    def update_fps(self):
        self.fps = self.computed_frames / (time.time() - t0)

    def set_origin_and_factor(self):
        """ This function calculates de conversion factor between pixel to centimeters
            and finds the (0,0) pos of the field in the image """

        # for x
        x = np.sort(self.arena_vertices[:,0])

        # xo is the third smaller vertice element because the first two ones are
        # the goal vertices
        xo = x[2]

        # the yo origin is the most bottom vertice
        yo = np.sort(self.arena_vertices[:,1])[-1]

        self.origin = np.array([xo, yo])

        # now just calculate the pixel to cm factor
        self.conversion_factor = 130.0 / self.arena_size[HEIGHT]

    def load_params(self):
        """ Loads the warp matrix and the arena vertices from the arena parameters file"""
        params = self.json_handler.read(self.params_file_name)

        self.arena_vertices = np.array(params['arena_vertices'])
        self.warp_matrix = np.asarray(params['warp_matrix']).astype("float32")
        self.arena_size = (params['arena_size'][0], params['arena_size'][1])

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

    def color_seg_pipeline(self):
        """ Wait until the color parameters are used """
        self.arena_image = cv2.cvtColor(self.arena_image, cv2.COLOR_BGR2HSV)

        self.blue_seg = self.get_filter(self.arena_image, self.blue_min, self.blue_max)
        self.yellow_seg = self.get_filter(self.arena_image, self.yellow_min, self.yellow_max)

        self.ball_seg = self.get_filter(self.arena_image, self.ball_min, self.ball_max)

    def attribute_teams(self):
        if self.home_color == "blue":
            self.home_seg = self.blue_seg
            self.adv_seg = self.yellow_seg
        else:
            self.home_seg = self.yellow_seg
            self.adv_seg = self.blue_seg

    def run(self):

        while not self.finish:

            self.t0 = time.time()
            while self.game_on:
                self.raw_image = camera.read()

                """ Takes the raw imagem from the camera and applies the warp perspective transform """
                self.warp_perspective()

                """ After the self.pipeline() and self.attribute_teams are executed, is expected that will be three images:
                    self.home_seg, self.adv_seg and self.ball_seg """
                self.pipeline()

                self.attribute_teams()
                #cv2.imshow('vision', 255-self.home_seg)
                self.hawk_eye.seek_home_team(255-self.home_seg, self.home_team)

                self.hawk_eye.seek_adv_team(self.adv_seg, self.adv_team)

                self.hawk_eye.seek_ball(self.ball_seg, self.ball)
                
                self.send_message(ball=True, home_team=True, adv_team=True)

                self.computed_frames += 1

                #self.update_fps()

        self.camera.stop()
        self.camera.capture.release()

    def send_message(self, ball=False, home_team=False, adv_team=False):
        """ This function will return the message in the right format to be
            published in the ROS vision bus """

        # Ball info
        ball_pos = [0,0]
        ball_speed = [0,0]

        # Home team info
        home_team_pos = [ [0,0] for _ in xrange(6)]
        home_team_orientation = [ 0 for _ in xrange(6)]
        home_team_speed = [[0,0] for _ in xrange(6)]

        # Adv team info
        adv_team_pos = [ [0,0] for _ in xrange(6)]
        adv_team_speed = [ [0,0] for _ in xrange(6)]

        if ball:
            ball_pos = self.ball.pos
            ball_speed = self.ball.speed

        if home_team:
            for robot in self.home_team:
                i = robot.id
                home_team_pos[i] = robot.pos
                
                home_team_orientation[i] = robot.orientation
                home_team_speed[i] = robot.speed

        if adv_team:
            for robot in self.adv_team:
                i = robot.id
                adv_team_pos[i] = robot.pos
                adv_team_speed[i] = robot.speed


        self.mercury.publish(ball_pos, ball_speed, home_team_pos, home_team_orientation,
                             home_team_speed, adv_team_pos, adv_team_speed)


if __name__ == "__main__":

    home_color = "yellow"  # blue or yellow
    home_robots = 5
    adv_robots = 3
    home_tag = "aruco"

    frame_hater = int(1/60*1000)

    arena_params = root_path+"parameters/ARENA.json"
    colors_params = root_path+"parameters/COLORS.json"

    #cv2.namedWindow('vision')

    try:
        device = int(sys.argv[1])
    except ValueError:
        device = sys.argv[1]

    camera = Camera(device, root_path+"parameters/CAMERA_ELP-USBFHD01M-SFV.json", threading=True)

    v = Vision(camera, adv_robots, home_color, home_robots, home_tag,
               arena_params, colors_params, method="color_segmentation")
    v.game_on = True

    t = Thread(target=v.run, args=())
    t.daemon = True
    t.start()

    rospy.on_shutdown(v.stop)

    rospy.spin()
    #cv2.destroyAllWindows()

