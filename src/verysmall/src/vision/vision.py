#!/usr/bin/python
import sys
sys.path.append('../')
import COLORS
import cv2
import numpy as np
import rospy
import time
from camera.camera import Camera
from vision_utils.params_setter import ParamsSetter
from vision_utils.color_segmentation import ColorSegmentation

from seekers.things_seeker import HawkEye
from seekers.things_seeker import Things

from verysmall.msg import game_topic

# Top level imports
import os
old_path = sys.path[0]
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT']+"src/"
from utils.json_handler import JsonHandler
from ROS.ros_vision_publisher import RosVisionPublisher
sys.path[0] = old_path

# @author Wellington Castro <wvmcastro>

HEIGHT = 1


class Vision:

    def __init__(self, camera, adv_robots, home_color, home_robots,
    home_tag="aruco", params_file_name="", colors_params = "", method=""):

        # This object will be responsible for publish the game state info
        # at the bus. Mercury is the gods messenger
        self.mercury = RosVisionPublisher(True)

        # Lists used to unpack the info from Things objects and publish those
        # infos in the vision topic

        # Ball info
        self.ball_pos = np.array([[.0, .0]])
        self.ball_speed = np.array([[.0, .0]])

        # Home team info
        self.home_team_pos = np.array([[0.0, 0.0]] * 5)
        self.home_team_orientation = np.array([0.0] * 5)
        self.home_team_speed = np.array([[0.0,0.0]] * 5)

        # Adv team info
        self.adv_team_pos = np.array([[0.0,0.0]] * 5)
        self.adv_team_orientation = np.array([0.0] * 5)
        self.adv_team_speed = np.array([[0.0,0.0]] * 5)

        # Subscribes to the game topic
        rospy.Subscriber('game_topic_1', game_topic, self.on_game_state_change)

        self.game_state = None

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
        self.last_time = None
        self.new_time = None
        ##self. computed_frames = 0

        # Super necessary to compute the robots positions
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
            self.color_calibrator = ColorSegmentation(camera, self.colors_params_file)
        else:
            print "Method not recognized!"

        if self.params_file_name != "":
            self.load_params()

        self.params_setter = ParamsSetter(camera, params_file_name)

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

    def on_game_state_change(self, data):
        self.game_state = data.game_state
        if self.game_state:
            self.hawk_eye.reset()
            self.reset_all_things()

    def reset_all_things(self):
        # Used when the game state changes to playing
        self.ball.reset()

        for i in xrange(len(self.home_team)):
            self.home_team[i].reset()

        for i in xrange(len(self.adv_team)):
            self.adv_team[i].reset()

    def start(self):
        self.game_on = True

    def pause(self):
        self.game_on = False

    def stop(self):
        self.pause()
        self.finish = True

    def update_fps(self):
        self.new_time = time.time()
        self.fps =  1/(self.new_time - self.last_time)##self.computed_frames / (time.time() - self.t0)
        self.last_time = self.new_time

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

            self.last_time = time.time()
            while self.game_on:
                self.raw_image = self.camera.read()

                """ Takes the raw imagem from the camera and applies the warp perspective transform """
                self.warp_perspective()

                self.set_dark_border()
                
                """ After the self.pipeline() and self.attribute_teams are executed, is expected that will be three images:
                    self.home_seg, self.adv_seg and self.ball_seg """
                self.pipeline()

                self.attribute_teams()

                self.hawk_eye.seek_home_team(255-self.home_seg, self.home_team)

                self.hawk_eye.seek_adv_team(self.adv_seg, self.adv_team)

                self.hawk_eye.seek_ball(self.ball_seg, self.ball)

                #self.computed_frames += 1

                self.update_fps()

                self.send_message(ball=True, home_team=True, adv_team=True)

        self.camera.stop()
        self.camera.capture.release()

    def unpack_things_to_lists(self, things, positions_list, orientations_list, speeds_list):
        """ Auxiliary  function created to not duplify code in the send_message
            function"""
        tags = [9, 14, 18, 23, 28]

        for thing in things:
            id = thing.id
            if id > 0 and id in tags: # sorry
                id = tags.index(thing.id)

            if id >= 0:
                positions_list[id] = thing.pos
                orientations_list[id] = thing.orientation
                speeds_list[id] = thing.speed

    def send_message(self, ball=False, home_team=False, adv_team=False):
        """ This function will return the message in the right format to be
            published in the ROS vision bus """

        if ball:
            self.unpack_things_to_lists([self.ball], self.ball_pos, [[]], self.ball_speed)

        if home_team:
            self.unpack_things_to_lists(self.home_team, self.home_team_pos,
                                        self.home_team_orientation, self.home_team_speed)

        if adv_team:
            self.unpack_things_to_lists(self.adv_team, self.adv_team_pos,
                                        self.adv_team_orientation, self.adv_team_speed)

        self.mercury.publish(self.ball_pos[0], self.ball_speed[0], self.home_team_pos,
                             self.home_team_orientation, self.home_team_speed, self.adv_team_pos,
                             self.adv_team_orientation, self.adv_team_speed, self.fps)


if __name__ == "__main__":
    from threading import Thread

    home_color = "yellow" # blue or yellow
    home_robots = 1
    adv_robots = 2
    home_tag = "aruco"

    arena_params = "../parameters/ARENA.json"
    colors_params = "../parameters/COLORS.json"
    camera = Camera(sys.argv[1], "../parameters/CAMERA_ELP-USBFHD01M-SFV.json", threading=True)

    v = Vision(camera, adv_robots, home_color, home_robots, home_tag,
    arena_params, colors_params, method="color_segmentation")
    v.game_on = True

    t = Thread(target=v.run, args=())
    t.daemon = True
    t.start()

    i = 0
    last_time = time.time()
    cv2.namedWindow('control')
    show = False

    while True:
        arena = v.arena_image
        key = cv2.waitKey(1) & 0xFF
        if show:
            cv2.imshow('vision', cv2.cvtColor(arena, cv2.COLOR_HSV2BGR))
            cv2.imshow('segs', np.hstack([v.blue_seg, v.yellow_seg, v.ball_seg]))
        if key == ord('q'): # exit
            v.pause()
            v.finish = True
            break
        elif key == ord('s'): #show/hide
            show = not show
            if show == False:
                cv2.destroyWindow("vision")
                cv2.destroyWindow("segs")
        elif key == ord('c'): # open cropper
            v.params_setter.run()
            v.load_params()
        elif key == ord('g'): # "get color"
            v.color_calibrator.run()
            v.load_colors_params()
        elif key == ord('f'):
            print v.fps, "frames/second"

    cv2.destroyAllWindows()
