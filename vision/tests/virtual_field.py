import cv2 as cv
import math
import numpy as np
from auxiliary import *


class Virtual_Field():

	"""class constructor using a .jpg image as background"""
	def __init__(self):
		# 1 cm for 4 pixels
		self.field = np.zeros((680,760,3), np.uint8)
		self.raw_field = self.field
		self.top_left = (80,80)
		self.top_right = (680,80)
		self.bottom_left = (80,600)
		self.bottom_right = (680,600)
		self.field_origin = self.bottom_left
		self.ball_radius = 7	# pixels
		self.robot_side_size = 30	# pixels
		self.blue = [255,0,0]
		self.yellow = [0,255,255]
		self.orange = [0,100,255]
		self.white = [255,255,255]

	def plot_arena(self):
		self.field = self.raw_field

		#main field
		cv.line(self.field, self.top_left, self.top_right, self.white)
		cv.line(self.field, self.top_right, self.bottom_right, self.white)
		cv.line(self.field, self.bottom_right, self.bottom_left, self.white)
		cv.line(self.field, self.bottom_left, self.top_left, self.white)
		cv.line(self.field, (380,80), (380,600), self.white)
		cv.circle(self.field, (380,340), 80, self.white, 0)

		#goal 1
		cv.line(self.field, (40,260), (40,420), self.white)
		cv.line(self.field, (40,260), (80,260), self.white)
		cv.line(self.field, (40,420), (80,420), self.white)

		#goal 2
		cv.line(self.field, (720,260), (720,420), self.white)
		cv.line(self.field, (720,260), (680,260), self.white)
		cv.line(self.field, (720,420), (680,420), self.white)

		#corners
		cv.line(self.field, (108,80), (80,108), self.white)
		cv.line(self.field, (680,108), (652,80), self.white)
		cv.line(self.field, (80,572), (108,600), self.white)
		cv.line(self.field, (680,572), (652,600), self.white)

		#goal areas
		cv.line(self.field, (140,200), (140,480), self.white)
		cv.line(self.field, (80,200), (140,200), self.white)
		cv.line(self.field, (80,480), (140,480), self.white)
		cv.ellipse(self.field, (140,340), (20,40), 180.0, 270.0, 90.0, self.white)

		cv.line(self.field, (620,200), (620,480), self.white)
		cv.line(self.field, (620,200), (680,200), self.white)
		cv.line(self.field, (620,480), (680,480), self.white)
		cv.ellipse(self.field, (620,340), (20,40), 0.0, 90.0, 270.0, self.white)

		



	"""plot an orange 7 pixels radius circle as the ball"""
	def plot_ball(self, ball_center):

		cv.circle(self.field, ball_center, self.ball_radius, self.orange, -1)


	"""plots all contours from all robots of a designed color given as parameter"""
	def plot_robots(self, center, vector, color):

		angle = angleBetween([1,0], vector)*180/(math.pi)

		contour = (center, (self.robot_side_size, self.robot_side_size), angle)

		n_contour = cv.boxPoints(contour)
		n_contour = np.int0(n_contour)


		cv.drawContours(self.field, [n_contour], -1, color, -1)
		cv.arrowedLine(self.field, center, (int(center[0]+3*vector[0]), int(center[1]+3*vector[1])), self.orange, 2)


	