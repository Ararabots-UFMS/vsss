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
from ROS.ros_vision_publisher import RosVisionPublisher
from vision_utils.params_setter import ParamsSetter


from robot_seeker import RobotSeeker
from robot_seeker import Things
from ball_seeker import BallSeeker

# @author Wellington Castro <wvmcastro>

HEIGHT = 1

class Vision:

    def __init__(self, camera, home_color, home_robots, adv_robots,
        params_file_name="", colors_params = "", method="", cluster_cfg=(5, 100, 500)):

        # This object will be responsible for publish the game state info
        # at the bus
        self.mercury = RosVisionPublisher()

        self.json_handler = JsonHandler()
        self.camera = camera
        self.params_file_name = params_file_name
        self.home_color = home_color
        self.home_robots = home_robots
        self.adv_robots = adv_robots

        self.arena_vertices = []
        self.arena_size = ()
        self.arena_image = None
        self.arena_mask = None
        self.raw_image = None
        self.warp_matrix = None
        self.pipeline = None

        self.origin = None
        self.conversion_factor = None

        # Creates the lists to the home team and the adversary
        self.home_team = [Things() for i in range(home_robots)]
        self.adv_team = [Things() for i in range(adv_robots)]

        # Object to store ball info
        self.ball = Things()

        # Initialize the vitamins according to the chosen method
        if method == "clustering":
            self.mbc_kmeans = MiniBatchKMeans(n_clusters = cluster_cfg[0], max_iter = cluster_cfg[1],
                                         batch_size=cluster_cfg[2])
            self.LAB_MAX = ([255, 255, 255])
            self.pipeline = self.cluster_pipeline
        elif method == "color_segmentation":
            self.colors_params_file = colors_params
            self.load_colors_params()
            self.pipeline = self.color_seg_pipeline
        else:
            print "Method not recognized!"

        self.params_setter = ParamsSetter(camera, params_file_name)
        self.i = 0 # gambito
        self.method = method

        if self.params_file_name != "":
            self.load_params()

        self.virtual_to_real()

        # Instantiates the RobotSeeker object
        self.hawk_eye = RobotSeeker(field_origin=self.origin, conversion_factor=self.conversion_factor)

        # Ball Seeker object
        self.ball_seeker = BallSeeker(field_origin=self.origin, conversion_factor=self.conversion_factor)

    def virtual_to_real(self):
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

    def attribute_teams(self):
        if self.home_color == "blue":
            self.home_seg = self.blue_seg
            self.adv_seg = self.yellow_seg
        else:
            self.home_seg = self.yellow_seg
            self.adv_seg = self.blue_seg

    def get_frame(self):
        """ Takes the raw imagem from the camera and applies the warp perspective transform
            and the mask """
        self.raw_image = camera.read()
        self.warp_perspective()

        self.pipeline()
        self.attribute_teams()


        """ After the self.pipeline() and self.attribute_teams are executed, is expected that will be three images:
            self.home_seg, self.adv_seg and self.ball_seg """
        self.hawk_eye.seek_aruco(255-self.home_seg, self.home_team, self.camera.camera_matrix, self.camera.dist_vector)
        self.ball_seeker.seek_ball(self.ball_seg, self.ball)
        self.mercury.publish(self.get_message(ball=True, home_team=True))
        # self.hawk_eye.seek(self.home_seg, self.home_team, direction=True, home_team=False)
        # self.hawk_eye.seek(self.adv_seg, self.adv_team, direction=False, home_team=True)

        # self.hawk_eye.debug(self.home_seg, self.home_team)
        # self.hawk_eye.debug(self.adv_seg, self.adv_team)

        return self.arena_image

    def get_message(self, ball=False, home_team=False, adv_team=False):
        """ This function will return the message in the right format to be
            published in the ROS vision bus """

        # Ball info
        ball_pos = [0,0]
        ball_speed = [0,0]

        # Home team info
        home_team_pos = 6*[[0,0]]
        home_team_orientation = 6*[0]
        home_team_speed = 6*[[0,0]]

        # Adv team info
        adv_team_pos = 6*[[0,0]]
        adv_team_speed = 6*[[0,0]]

        if ball == True:
            ball_pos = self.ball.pos
            ball_speed = self.ball.speed

        if home_team == True:
            for robot in self.home_team:
                i = robot.id
                home_team_pos[i] = robot.pos
                home_team_orientation[i] = robot.orientation
                home_team_speed[i] = robot.speed

        if adv_team == True:
            for robot in self.adv_team:
                i = robot.id
                adv_team_pos[i] = robot.pos
                adv_team_speed[i] = robot.speed

        return ball_pos, ball_speed, home_team_pos, home_team_orientation,\
                home_team_speed, adv_team_pos, adv_team_speed

if __name__ == "__main__":

    home_color = "yellow" # blue or yellow
    home_robots = 3
    adv_robots = 3

    arena_params = "../parameters/ARENA.json"
    colors_params = "../parameters/COLORS.json"
    camera = Camera("record.avi", "../parameters/CAMERA_ELP-USBFHD01M-SFV.json", threading=True)

    v = Vision(camera, home_color, home_robots, adv_robots,
                arena_params, colors_params, method="color_segmentation")

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
