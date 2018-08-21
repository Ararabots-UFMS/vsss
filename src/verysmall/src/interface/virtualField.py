import cv2 as cv
import math
import numpy as np
import rospy
import time
from auxiliary import *
import copy


class virtualField():
    """ class constructor """

    def __init__(self, width=850, height=650, is_rgb=False):
        # 1 cm for 4 pixels
        self.width = width
        self.height = height
        self.field = None
        self.raw_field = np.zeros((self.height, self.width, 3), np.uint8)

        self.width_conv = 0.88136 * self.width / 150
        self.height_conv = 0.998 * self.height / 130
        self.angle_conversion_factor = 180 / math.pi

        self.field_origin = (self.proportion_width(5.882), self.proportion_height(99.9))
        self.ball_radius = self.proportion_average(1.5)
        self.mark_radius = self.proportion_average(0.3)
        self.robot_side_size = self.proportion_average(4.5)
        self.away_team_radius = self.proportion_average(2.5)

        self.text_font = cv.FONT_HERSHEY_SIMPLEX

        if is_rgb:
            self.colors = {"blue": [0, 0, 255],
                           "orange": [255, 100, 0],
                           "white": [255, 255, 255],
                           "yellow": [255, 255, 0],
                           "red": [255, 0, 0],
                           "green": [116, 253, 0],
                           "dgreen": [0, 104, 0],
                           "black": [0, 0, 0],
                           "mark": [30, 30, 30],
                           "gray": [150, 150, 150]
                           }
        else:
            self.colors = {"blue": [255, 0, 0],
                           "orange": [0, 100, 255],
                           "white": [255, 255, 255],
                           "yellow": [0, 255, 255],
                           "red": [0, 0, 255],
                           "green": [0, 104, 0],
                           "dgreen": [0, 253, 116],
                           "black": [0, 0, 0],
                           "mark": [30, 30, 30],
                           "gray": [150, 150, 150]
                           }

    def pause(self, n):
        """system pause for n FPS"""
        time.sleep(1.0 / n)

    """plots all arena contours and inner lines"""

    def plot_arena(self, field):

        # border line
        cv.rectangle(field, (self.proportion_width(5.882), self.proportion_height(0.1)),
                     (self.proportion_width(94.018), self.proportion_height(99.9)), self.colors["white"])

        # midfield line
        cv.line(field, (self.proportion_width(50), self.proportion_height(0.1)),
                (self.proportion_width(50), self.proportion_height(99.9)), self.colors["white"])

        # left goal and outfield areas
        # cv.rectangle(self.field, (self.proportion_width(0.1), self.proportion_height(0.1)), (self.proportion_width(5.882), self.proportion_height(34.615)), self.colors["white"])
        cv.rectangle(field, (self.proportion_width(0.1), self.proportion_height(34.615)),
                     (self.proportion_width(5.882), self.proportion_height(65.385)), self.colors["white"])
        # cv.rectangle(self.field, (self.proportion_width(0.1), self.proportion_height(65.385)), (self.proportion_width(5.882), self.proportion_height(99.9)), self.colors["white"])

        # right goal and outfield areas
        # cv.rectangle(self.field, (self.proportion_width(94.018), self.proportion_height(0.1)), (self.proportion_width(99.9), self.proportion_height(34.615)), self.colors["white"])
        cv.rectangle(field, (self.proportion_width(94.018), self.proportion_height(34.615)),
                     (self.proportion_width(99.9), self.proportion_height(65.385)), self.colors["white"])
        # cv.rectangle(self.field, (self.proportion_width(94.018), self.proportion_height(65.385)), (self.proportion_width(99.9), self.proportion_height(99.9)), self.colors["white"])

        # left and rigth goal areas
        cv.rectangle(field, (self.proportion_width(5.882), self.proportion_height(23.076)),
                     (self.proportion_width(14.705), self.proportion_height(76.924)), self.colors["white"])
        cv.rectangle(field, (self.proportion_width(85.295), self.proportion_height(23.076)),
                     (self.proportion_width(94.018), self.proportion_height(76.924)), self.colors["white"])

        # left and right ellipses
        cv.ellipse(field, (self.proportion_width(14.705), self.proportion_height(50.0)),
                   (self.proportion_width(2.941), self.proportion_height(7.692)), 180, 90.0, 270.0, self.colors["white"])
        cv.ellipse(field, (self.proportion_width(85.295), self.proportion_height(50.0)),
                   (self.proportion_width(2.941), self.proportion_height(7.692)), 0, 90.0, 270.0, self.colors["white"])

        # corner lines
        cv.line(field, (self.proportion_width(10), self.proportion_height(0.1)),
                (self.proportion_width(5.882), self.proportion_height(5.384)), self.colors["white"])
        cv.line(field, (self.proportion_width(90), self.proportion_height(0.1)),
                (self.proportion_width(94.018), self.proportion_height(5.384)), self.colors["white"])
        cv.line(field, (self.proportion_width(5.882), self.proportion_height(94.616)),
                (self.proportion_width(10), self.proportion_height(99.9)), self.colors["white"])
        cv.line(field, (self.proportion_width(94.018), self.proportion_height(94.616)),
                (self.proportion_width(90), self.proportion_height(99.9)), self.colors["white"])

        # midfield circle
        cv.circle(field, (self.proportion_width(50), self.proportion_height(50)), self.proportion_average(13),
                  self.colors["white"])

        # left freeball markers
        cv.line(field, (self.proportion_width(26.941), self.proportion_height(19.230)),
                (self.proportion_width(28.941), self.proportion_height(19.230)), self.colors["white"], 1)
        cv.line(field, (self.proportion_width(27.941), self.proportion_height(20.530)),
                (self.proportion_width(27.941), self.proportion_height(17.930)), self.colors["white"], 1)

        cv.line(field, (self.proportion_width(26.941), self.proportion_height(80.770)),
                (self.proportion_width(28.941), self.proportion_height(80.770)), self.colors["white"], 1)
        cv.line(field, (self.proportion_width(27.941), self.proportion_height(82.070)),
                (self.proportion_width(27.941), self.proportion_height(79.470)), self.colors["white"], 1)

        cv.line(field, (self.proportion_width(26.941), self.proportion_height(50.0)),
                (self.proportion_width(28.941), self.proportion_height(50.0)), self.colors["white"], 1)
        cv.line(field, (self.proportion_width(27.941), self.proportion_height(51.3)),
                (self.proportion_width(27.941), self.proportion_height(48.7)), self.colors["white"], 1)

        # rigth freeball markers
        cv.line(field, (self.proportion_width(71.059), self.proportion_height(19.230)),
                (self.proportion_width(73.059), self.proportion_height(19.230)), self.colors["white"], 1)
        cv.line(field, (self.proportion_width(72.059), self.proportion_height(17.930)),
                (self.proportion_width(72.059), self.proportion_height(20.530)), self.colors["white"], 1)

        cv.line(field, (self.proportion_width(71.059), self.proportion_height(80.770)),
                (self.proportion_width(73.059), self.proportion_height(80.770)), self.colors["white"], 1)
        cv.line(field, (self.proportion_width(72.059), self.proportion_height(79.470)),
                (self.proportion_width(72.059), self.proportion_height(82.070)), self.colors["white"], 1)

        cv.line(field, (self.proportion_width(71.059), self.proportion_height(50.0)),
                (self.proportion_width(73.059), self.proportion_height(50.0)), self.colors["white"], 1)
        cv.line(field, (self.proportion_width(72.059), self.proportion_height(48.7)),
                (self.proportion_width(72.059), self.proportion_height(51.3)), self.colors["white"], 1)

        # left robot markers
        cv.circle(field, (self.proportion_width(16.176), self.proportion_height(19.230)), self.mark_radius,
                  self.colors["gray"], -1)
        cv.circle(field, (self.proportion_width(16.176), self.proportion_height(80.770)), self.mark_radius,
                  self.colors["gray"], -1)
        cv.circle(field, (self.proportion_width(39.705), self.proportion_height(19.230)), self.mark_radius,
                  self.colors["gray"], -1)
        cv.circle(field, (self.proportion_width(39.705), self.proportion_height(80.770)), self.mark_radius,
                  self.colors["gray"], -1)

        # right robot markers
        cv.circle(field, (self.proportion_width(83.824), self.proportion_height(19.230)), self.mark_radius,
                  self.colors["gray"], -1)
        cv.circle(field, (self.proportion_width(83.824), self.proportion_height(80.770)), self.mark_radius,
                  self.colors["gray"], -1)
        cv.circle(field, (self.proportion_width(60.295), self.proportion_height(19.230)), self.mark_radius,
                  self.colors["gray"], -1)
        cv.circle(field, (self.proportion_width(60.295), self.proportion_height(80.770)), self.mark_radius,
                  self.colors["gray"], -1)

    def plot_ball(self, ball_center):

        self.field = copy.deepcopy(self.raw_field)

        validate = ball_center

        ball_center = unit_convert(ball_center, self.width_conv, self.height_conv)
        ball_center = position_from_origin(ball_center, self.field_origin)

        if (validate[0] < 0.1 and 45.0 < validate[1] < 85.0):
            cv.rectangle(self.field, (self.proportion_width(0.1), self.proportion_height(34.615)),
                         (self.proportion_width(5.882), self.proportion_height(65.385)), self.colors["green"], -1)

        elif (validate[0] > 150.0 and 45.0 < validate[1] < 85.0):
            cv.rectangle(self.field, (self.proportion_width(94.018), self.proportion_height(34.615)),
                         (self.proportion_width(99.9), self.proportion_height(65.385)), self.colors["green"], -1)

        elif (15.0 >= validate[0] > 0.0 and 30.0 < validate[1] < 100.0 or (
                ((validate[0] - 15) ** 2 / (10) ** 2) + ((validate[1] - 65) ** 2 / (5) ** 2) < 1)):

            cv.rectangle(self.field, (self.proportion_width(5.882), self.proportion_height(23.076)),
                         (self.proportion_width(14.705), self.proportion_height(76.924)), self.colors["dgreen"], -1)

            cv.ellipse(self.field, (self.proportion_width(14.705), self.proportion_height(50.0)),
                       (self.proportion_width(2.941), self.proportion_height(7.692)), 180, 90.0, 270.0,
                       self.colors["dgreen"], -1)

        elif (150.0 > validate[0] >= 135.0 and 30.0 < validate[1] < 100.0 or (
                ((validate[0] - 135) ** 2 / (10) ** 2) + ((validate[1] - 65) ** 2 / (5) ** 2) < 1)):
            cv.rectangle(self.field, (self.proportion_width(85.295), self.proportion_height(23.076)),
                         (self.proportion_width(94.018), self.proportion_height(76.924)), self.colors["dgreen"], -1)
            cv.ellipse(self.field, (self.proportion_width(85.295), self.proportion_height(50.0)),
                       (self.proportion_width(2.941), self.proportion_height(7.692)), 0, 90.0, 270.0,
                       self.colors["dgreen"], -1)

        else:
            pass

        if validate[0] != 0 or validate[1] !=0:
            cv.circle(self.field, ball_center, self.ball_radius, self.colors["orange"], -1)

    def plot_robots(self, robot_list, robot_vector, color, is_away=False):
        """plots all contours from all robots of a designed color given as parameter"""
        index = 0
        length = len(robot_list)

        while index < length:

            if robot_list[index][0] != 0 or robot_list[index][1] != 0:

                if is_away:
                    center = position_from_origin(
                        unit_convert(robot_list[index], self.width_conv, self.height_conv), self.field_origin)
                    cv.circle(self.field, center, self.away_team_radius, color, -1)
                    cv.putText(self.field, str(index), center, self.text_font, 0.5, self.colors["white"], 1, cv.LINE_AA)
                else:
                    angle = robot_vector[index]
                    center = position_from_origin(
                        unit_convert(robot_list[index], self.width_conv, self.height_conv), self.field_origin)
                    contour = (center, (self.robot_side_size, self.robot_side_size), -angle * self.angle_conversion_factor)
                    n_contour = cv.boxPoints(contour)
                    n_contour = np.int0(n_contour)
                    cv.drawContours(self.field, [n_contour], -1, color, -1)
                    cv.arrowedLine(self.field, center, (int(center[0] + math.cos(angle) * self.robot_side_size),
                                                        int(center[1] + math.sin(-angle) * self.robot_side_size)),
                                   self.colors["red"], 2)
                    cv.putText(self.field, str(index), center, self.text_font, 0.5, self.colors["black"], 1, cv.LINE_AA)

            index = index + 1

    def proportion_height(self, proportion):
        """Returns the Y value for the designed vertical screen proportion"""
        return int(self.height * proportion / 100)

    def proportion_width(self, proportion):
        """Returns the X value for the designed horizontal screen proportion"""
        return int(self.width * proportion / 100)

    def proportion_average(self, size):
        return int(((self.width + self.height) * 0.5) * size / 100)
