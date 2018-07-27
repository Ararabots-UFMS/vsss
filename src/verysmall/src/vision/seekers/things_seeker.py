import numpy as np
import cv2
import time
import math

from aruco_seeker import ArucoSeeker
from general_object_seeker import GeneralObjSeeker
from general_mult_obj_seeker import GeneralMultObjSeeker

# @author Wellington Castro <wvmcastro>

# Sorry for these globals, but is good for code reading
# Go Ararabots!
POS = 0
ANGLE = 1

class Things:
    # This is an auxiliary class to hold the variables from the things identified
    # by this hawk eye system
    def __init__(self):
        self.id = -1

        # Stores the (x,y) pos from the rebot
        self.pos = np.array([None, None])

        # The orientation is stored in radians in relation with the x axis
        self.orientation = None

        # Stores the (dx/dt, dy/dt) components
        self.speed = np.array([None, None])

        # Saves the time of the last update
        self.last_update = None

    def update(self, id, pos, orientation=None):
        last_up = None
        now = time.time()

        # Checkes if this is not the first update
        if self.last_update != None and np.all(self.pos != None) and np.all(pos != None):
            # If it is not the first update, calculate the robot speed
            self.speed = (pos - self.pos) / (now - self.last_update)

        # Updates the robot's state variables
        self.id = id
        self.last_update = now
        self.pos = pos
        self.orientation = orientation

    def reset(self):
        self.pos = np.array([None, None])
        self.last_update = None

class HawkEye:
    """ This class will be responsible of locate and identify all objects present
        in the field """
    # https://docs.opencv.org/master/d9/d8b/tutorial_py_contours_hierarchy.html#gsc.tab=0

    def __init__(self, field_origin, conversion_factor, home_tag, num_robots_home_team,
    num_robots_adv_team, img_shape, aux_params):

        self.field_origin = field_origin
        self.conversion_factor = conversion_factor
        self.rad_to_degree_factor = 180.0/math.pi
        self.num_robots_home_team = num_robots_home_team
        self.num_robots_adv_team = num_robots_adv_team

        if home_tag == "aruco":
            camera_matrix = aux_params[0]
            distortion_vector = aux_params[1]
            self.home_team_seeker = ArucoSeeker(camera_matrix, distortion_vector, self.num_robots_home_team)
        else:
            print "falta implementar"

        self.adv_team_seeker = GeneralMultObjSeeker(num_robots_adv_team)

        self.ball_seeker = GeneralObjSeeker(img_shape)

    def pixel_to_real_world(self, pos):
        # This function expects that pos is a 1D numpy array
        pos = pos - self.field_origin
        pos[1] *= -1

        return pos * self.conversion_factor

    def seek_home_team(self, img, robots_list):
        """ This function expects a binary image with the team robots and a list
            of Things objects to store the info """

        robots = self.home_team_seeker.seek(img, degree=False)

        for i in xrange(len(robots)):
            pos = self.pixel_to_real_world(robots[i][POS])
            robots_list[i].update(i, pos, orientation=robots[i][ANGLE])

    def seek_ball(self, img, ball):
        """ Expects a binary image with just the ball and a Thing object to
            store the info """

        px_pos = self.ball_seeker.seek(img)
        if np.all(px_pos != None):
            pos = self.pixel_to_real_world(px_pos)
            ball.update(-1, pos)
        else:
            ball.reset()

    def seek_adv_team(self, img, robots_list):

        adv_centers = self.adv_team_seeker.seek(img)

        if adv_centers != None and adv_centers.size:
            for i in xrange(self.num_robots_adv_team):
                pos = self.pixel_to_real_world(adv_centers[i, :])
                robots_list[i].update(i, pos)


if __name__ == '__main__':
    pass
