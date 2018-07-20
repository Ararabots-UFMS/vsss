import cv2 as cv
import math
import numpy as np
import time
from auxiliary import *


class Virtual_Field():
    """ class constructor """

    def __init__(self, width=760, height=680, is_rgb = False):
        # 1 cm for 4 pixels
        self.width = width
        self.height = height
        self.field = np.zeros((self.height, self.width, 3), np.uint8)
        self.raw_field = np.zeros((self.height, self.width, 3), np.uint8)

        self.top_left = (self.proportion_width(10), self.proportion_height(11))
        self.top_right = (self.proportion_width(89), self.proportion_height(11))
        self.bottom_left = (self.proportion_width(10), self.proportion_height(88))
        self.bottom_right = (self.proportion_width(89), self.proportion_height(88))
        self.field_origin = (self.proportion_width(5), self.proportion_height(88))

        self.ball_radius = self.proportion_average(1)  # pixels
        self.robot_side_size = self.proportion_average(4)  # pixels

        # middle line and circle
        self.middle_line = [(self.proportion_width(50), self.proportion_height(11)),
                            (self.proportion_width(50), self.proportion_height(88))]
        self.center_circle = [(self.proportion_width(50), self.proportion_height(50)),
                              self.proportion_average(11)]

        # left goal and left area
        self.left_goal_inside = [(self.proportion_width(5), self.proportion_height(38)),
                                 (self.proportion_width(10), self.proportion_height(61))]
        self.left_area = [(self.proportion_width(10), self.proportion_height(29)),
                          (self.proportion_width(18), self.proportion_height(70))]

        # rigth goal and right area
        self.right_goal_inside = [(self.proportion_width(89), self.proportion_height(38)),
                                  (self.proportion_width(94), self.proportion_height(61))]
        self.right_area = [(self.proportion_width(81), self.proportion_height(29)),
                           (self.proportion_width(89), self.proportion_height(70))]
        # still areas...
        self.round_areas = [
            (self.proportion_width(3), self.proportion_height(6)),  # Axis
            (self.proportion_width(18), self.proportion_height(50)),  # Left circle
            (self.proportion_width(81), self.proportion_height(50))  # Right Circle
        ]

        # corners points
        self.corner_points_x = [self.proportion_width(10),
                                self.proportion_width(14),
                                self.proportion_width(85),
                                self.proportion_width(89)]

        self.corner_points_y = [self.proportion_height(11),
                                self.proportion_height(15),
                                self.proportion_height(84),
                                self.proportion_height(88)]

        if is_rgb:
            self.colors = {"blue": [0, 0, 255],
                           "orange": [255, 100, 0],
                           "white": [255, 255, 255],
                           "yellow": [255, 255, 0],
                           "red": [255, 0, 0],
                           "green": [116, 253, 0],
                           "gray": [111, 111, 111]
                           }
        else:
            self.colors = {"blue": [255, 0, 0],
                           "orange": [0, 100, 255],
                           "white": [255, 255, 255],
                           "yellow": [0, 255, 255],
                           "red": [0, 0, 255],
                           "green": [0, 253, 116],
                           "gray": [111, 111, 111]
                           }
    """system pause for n FPS"""

    def pause(self, n):
        time.sleep(1.0 / n)

    """plots all arena contours and inner lines"""

    def plot_arena(self):
        self.field = np.zeros((self.height, self.width, 3), np.uint8)

        # main field
        cv.line(self.field, self.top_left, self.top_right, self.colors["white"])
        cv.line(self.field, self.top_right, self.bottom_right, self.colors["white"])
        cv.line(self.field, self.bottom_right, self.bottom_left, self.colors["white"])
        cv.line(self.field, self.bottom_left, self.top_left, self.colors["white"])
        cv.line(self.field, self.middle_line[0], self.middle_line[1], self.colors["white"])
        cv.circle(self.field, self.center_circle[0], self.center_circle[1], self.colors["white"], 0)

        # goal 1
        cv.rectangle(self.field, self.left_goal_inside[0], self.left_goal_inside[1], self.colors["white"])

        # goal 2
        cv.rectangle(self.field, self.right_goal_inside[0], self.right_goal_inside[1], self.colors["white"])

        # corners
        cv.line(self.field, (self.corner_points_x[1], self.corner_points_y[0]),
                (self.corner_points_x[0], self.corner_points_y[1]), self.colors["white"])
        cv.line(self.field, (self.corner_points_x[3], self.corner_points_y[1]),
                (self.corner_points_x[2], self.corner_points_y[0]), self.colors["white"])
        cv.line(self.field, (self.corner_points_x[0], self.corner_points_y[2]),
                (self.corner_points_x[1], self.corner_points_y[3]), self.colors["white"])
        cv.line(self.field, (self.corner_points_x[3], self.corner_points_y[2]),
                (self.corner_points_x[2], self.corner_points_y[3]), self.colors["white"])

        # goal areas
        cv.rectangle(self.field, self.left_area[0], self.left_area[1], self.colors["white"])
        cv.ellipse(self.field, self.round_areas[1], self.round_areas[0], 180.0, 270.0, 90.0, self.colors["white"])

        cv.rectangle(self.field, self.right_area[0], self.right_area[1], self.colors["white"])
        cv.ellipse(self.field, self.round_areas[2], self.round_areas[0], 0.0, 90.0, 270.0, self.colors["white"])

    """plot an orange 7 pixels radius circle as the ball"""

    def plot_ball(self, ball_center):

        ball_center = position_from_origin(unit_convert(ball_center))

        #r1 = range(40, 80)
        #r2 = range(260, 420)
        #r3 = range(680, 720)
        #r4 = range(260, 420)
        #r5 = range(81, 140)
        #r6 = range(200, 480)
        #r7 = range(620, 679)

        #if (ball_center[0] in r1 and ball_center[1] in r2):
        #    cv.rectangle(self.field, (41, 261), (79, 419), self.colors["green"], -1)
        #elif (ball_center[0] in r3 and ball_center[1] in r4):
        #    cv.rectangle(self.field, (681, 261), (719, 419), self.colors["green"], -1)
        #elif (ball_center[0] in r5 and ball_center[1] in r6):
        #    cv.rectangle(self.field, (81, 201), (139, 479), self.colors["green"], -1)
        #    cv.ellipse(self.field, (140, 340), (19, 39), 180.0, 270.0, 90.0, self.colors["green"], -1)
        #elif (ball_center[0] in r7 and ball_center[1] in r6):
        #    cv.rectangle(self.field, (621, 201), (679, 479), self.colors["green"], -1)
        #    cv.ellipse(self.field, (620, 340), (19, 39), 0.0, 90.0, 270.0, self.colors["green"], -1)
        #elif ((ball_center[0] - 620) ** 2 / 400 + (ball_center[1] - 340) ** 2 / 1600 < 1):
        #    cv.rectangle(self.field, (621, 201), (679, 479), self.colors["green"], -1)
        #    cv.ellipse(self.field, (620, 340), (19, 39), 0.0, 90.0, 270.0, self.colors["green"], -1)
        #elif ((ball_center[0] - 140) ** 2 / 400 + (ball_center[1] - 340) ** 2 / 1600 < 1):
        #    cv.rectangle(self.field, (81, 201), (139, 479), self.colors["green"], -1)
        #    cv.ellipse(self.field, (140, 340), (19, 39), 180.0, 270.0, 90.0, self.colors["green"], -1)

        cv.circle(self.field, ball_center, self.ball_radius, self.colors["orange"], -1)

    """plots all contours from all robots of a designed color given as parameter"""

    def plot_robots(self, robot_list, color, origin_vector=False):

        for robot in robot_list:

            vector = robot[1]
            center = position_from_origin(unit_convert(robot[0]))

            angle = angle_between([1, 0], vector) * 180 / (math.pi)

            contour = (center, (self.robot_side_size, self.robot_side_size), angle)

            n_contour = cv.boxPoints(contour)
            n_contour = np.int0(n_contour)

            cv.drawContours(self.field, [n_contour], -1, color, -1)

            # direction vectors
            cv.arrowedLine(self.field, center, (int(center[0] + 3 * vector[0]), int(center[1] + 3 * vector[1])),
                           self.colors["red"], 2)

            # vector from field origin
            if (origin_vector):
                cv.arrowedLine(self.field, self.field_origin, (int(center[0]), int(center[1])), self.colors["gray"], 1)

    def proportion_height(self, proportion):
        """Returns the Y value for the designed vertical screen proportion"""
        return int(self.height * proportion / 100)

    def proportion_width(self, proportion):
        """Returns the X value for the designed horizontal screen proportion"""
        return int(self.width * proportion / 100)

    def proportion_average(self, size):
        return int(((self.width + self.height) * 0.5) * size / 100)
