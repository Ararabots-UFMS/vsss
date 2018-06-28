import numpy as np
import cv2

class Robot:
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

class RobotSeeker:
    # https://docs.opencv.org/master/d9/d8b/tutorial_py_contours_hierarchy.html#gsc.tab=0

    def __init__(self):
        pass

    def get_contours(self, img):
        # Takes a binary image as input and return its contours
        im2, cont, h = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        return cont


    def getKey(self, item):
        # Auxialiary function to sort some list of tuples
        return item[0]

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
                rectangles.append( (area, rect[0], rect_vertices) )

        # Returns all the interest rectangles
        return rectangles

    def big_small_threshold(self, rectangles):
        # This function returns the threshold that separates the big from the small
        # rectangles finded by self.get_rectangles. The threshold is the value of the
        # biggest rect from the set of small rectangles (I hope that it is clear, sorry)

        # Sort all the rectangles in relation with the area
        sorted_rectangles = sorted(rectangles, key=getKey)

        # Calculates the diff between the areas of consecutive rects
        diff = np.array( sorted_rectangles[1:][0] ) - np.array( sorted_rectangles[0:-1][0] )

        n = np.argmax(diff)

        return sorted_rectangles[n][0]

    def find_robots(self, robots_list, rectangles, pos=True, direction=False, speed=False, homeTeam=False):
        # This function fills the robots_list with the info of each robot
        # If the homeTeam flag is True than it will indentify the home team robots too

        if homeTeam == True:
            # Find the threshold that separates the big from small rectangles
            thresh = self.big_small_threshold(rectangles)

            big_ones = []
            small_ones = []
            robots_rects = []

            # Separates the found rectangles in two sets (big ones and small ones)
            for rect in rectangles:
                # Checks if the rectangle area is bigger than the threshold
                if(rect[0] > thresh):
                    big_ones.append(rect)
                else:
                    small_ones.append(rect)

            while(len(big_ones) > 0 and len(small_ones) > 0):

                # Finds the closest small rectangle to the big one
                dst = np.linalg.norm(np.array(small_ones[:][1]) - np.array(big_ones[0][1]))
                small_index = np.argmin(dst)

                # Insert the pair in this Auxialiary list
                robots_rects.append(big_ones[0], small_ones[small_index])

                # Delete the exampples from the samples
                del big_ones[0]
                del small_ones[small_index]
        else:
            robots_rects = rectangles

        # Now we assume that each element in robots_rects represents a robot
        for i,r in enumerate(robots_rects):

            if pos == True:
                if homeTeam == True:
                    # Calculates the centroind of the two rectangles
                    # Area [0] and center [1]
                    x = (r[0][0] * r[0][1][0] + r[1][0] * r[1][1][0]) / (r[0][0] + r[1][0])
                    y = (r[0][0] * r[0][1][1] + r[1][0] * r[1][1][1]) / (r[0][0] + r[1][0])
                else:
                    x,y = r[1]

                robots_list[i].pos = (x,y)

            if direction == True and homeTeam == True:





if __name__ == '__main__':
    pass
