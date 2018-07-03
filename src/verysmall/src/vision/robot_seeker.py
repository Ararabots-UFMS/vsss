import numpy as np
import cv2
import cv2.aruco as aruco
import time

import sys
sys.path.append('../robot/movement')

from auxiliary.auxiliary import angleBetween

# @author Wellington Castro <wvmcastro>

# Sorry for these globals, but is good for code reading
# Go Ararabots!

AREA        = 0
CENTER      = 1
VERTICES    = 2

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

    def update(self, id, pos, orientation):
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


class RobotSeeker:
    # https://docs.opencv.org/master/d9/d8b/tutorial_py_contours_hierarchy.html#gsc.tab=0

    def __init__(self):
        self.is_aruco_started = False
        self.aruco_dict = None
        seld.aruco_params = None

    def init_aruco(self):
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        seld.aruco_params = aruco.DetectorParameters_create()
        pass

    def get_contours(self, img):
        # Takes a binary image as input and return its contours
        im2, cont, h = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        return cont

    def get_rectangles(self, contours, area_threshold=None):
        # Gets the contours of the image and approximate then by a rectangle
        # outputs all the rectangles finded that satisfies the threshold if passed

        rectangles = []
        for contour in contours:

            area = cv2.contourArea(contour)

            # Checks if the contour passes the threshold  if any is set
            if area_threshold == None or area > area_threshold:
                # Approaches the contour to the smallest rectangle that includes it
                # Rect is a list that has the [(x,y) center, (width, height), angle of rotation]
                rect = cv2.minAreaRect(contour)

                # Takes the info from rect and calculates its four vertices
                rect_vertices = cv2.boxPoints(rect)

                # Saves to the rectangles list
                rectangles.append( (area, np.asarray(rect[0]), np.array(rect_vertices)) )

        # Returns all the interest rectangles
        return rectangles

    def big_small_threshold(self, rectangles):
        # This function returns the threshold that separates the big from the small
        # rectangles finded by self.get_rectangles. The threshold is the value of the
        # biggest rect from the set of small rectangles (I hope that it is clear, sorry)

        # Extracts the area of each tuple in rectangles
        areas = [x[AREA] for x in rectangles]

        # Sort all the rectangles in relation with the area
        sorted_areas = sorted(areas)

        # Calculates the diff between the areas of consecutive rects
        diff = np.array( sorted_areas[1:] ) - np.array( sorted_areas[0:-1] )

        n = np.argmax(diff)

        return sorted_areas[n]

    def find_robots(self, robots_list, rectangles, direction=False, home_team=False):
        # This function fills the robots_list with the info of each robot
        # If the home_team flag is True than it will indentify the home team robots too

        # Aux flag
        k = 0

        if home_team == True and len(rectangles) > 1:
            # Find the threshold that separates the big from small rectangles
            thresh = self.big_small_threshold(rectangles)

            big_ones = []
            small_ones = []
            robots_rects = []

            # Separates the found rectangles in two sets (big ones and small ones)
            for rect in rectangles:
                # Checks if the rectangle area is bigger than the threshold
                if(rect[AREA] > thresh):
                    big_ones.append(rect)
                else:
                    small_ones.append(rect)

            while(len(big_ones) > 0 and len(small_ones) > 0):
                # Finds the closest small rectangle to the big one
                dst = np.linalg.norm(np.array([ x[CENTER] for x in small_ones ]) - np.array(big_ones[0][CENTER]))
                small_index = np.argmin(dst)

                # Insert the pair in this Auxialiary list
                robots_rects.append(( big_ones[0], small_ones[small_index]))

                # Delete the exampples from the samples
                del big_ones[0]
                del small_ones[small_index]
            k = 1
        else:
            robots_rects = rectangles


        # Now we assume that each element in robots_rects represents a robot
        for i,r in enumerate(robots_rects):
            _direction = None
            pos_xy = np.array([None, None])

            # Finds the position of the robots
            if home_team == True and k:
                # Calculates the centroind of the two rectangles
                pos_xy = (r[0][AREA]*r[0][CENTER] + r[1][AREA]*r[1][CENTER]) / (r[0][AREA]+r[1][AREA])
                id = i
            else:
                pos_xy = r[CENTER]
                id = -1

            if direction == True and home_team == True and k:
                # Detects the direction that the robot is pointed

                # Finds the index of the vertice of the large rectangle closest
                # to the center of the small rectangle
                origin_index = np.argmin( np.linalg.norm(r[0][VERTICES] - r[1][CENTER], axis=1) )

                # Now with the origin of the direction vector we calculate the
                # two possible direction, ie, the two edges of the big rectangle
                # and choose the bigger
                direct1 = r[0][VERTICES][(origin_index+1)%4] - r[0][VERTICES][origin_index]
                direct2 = r[0][VERTICES][(origin_index+3)%4] - r[0][VERTICES][origin_index]

                # Choose the _direction with the biggest norm
                if np.linalg.norm(direct1) > np.linalg.norm(direct2):
                    _direction = direct1
                else:
                    _direction = direct2

                _direction = angleBetween(np.array([1,0]), _direction)

            if i < len(robots_list):
                robots_list[i].update(id, pos_xy, _direction)

    def find aruco_robots(self, img, things):
        pass

    def seek(self, img, things_list, min_area_thresh=20, direction=False, home_team=False):
        # Implements the pipeline to find the robots in the field
        # receives a binary image with the elements and a list of Things
        # to store the data
        cnt = self.get_contours(img)
        rects = self.get_rectangles(cnt, area_threshold=min_area_thresh)
        self.find_robots(things_list, rects, direction, home_team)

    def debug(self, img, things_list):
        # This function prints the center and vectors of
        # each thing in the things list

        radius = 3

        # Creates a copy of img with 3 channels
        im_shape = img.shape
        rgb_shape = (im_shape[0], im_shape[1], 3)
        debug_img = np.zeros(rgb_shape)
        debug_img[:,:,0], debug_img[:,:,1], debug_img[:,:,2] = 255*img, 255*img, 255*img

        for thing in things_list[0:2]:
            # Draws the center of the thing
            if np.all(thing.pos != None):
                cv2.circle(debug_img, tuple(np.int_(thing.pos)), radius, (0,0,255), -1)

        cv2.imshow("Debug seek robots", debug_img)

if __name__ == '__main__':
    pass
