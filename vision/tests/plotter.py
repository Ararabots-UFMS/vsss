import cv2 as cv
import math
import numpy as np



class Plotter():

	"""class constructor using a .jpg image as background"""
	def __init__(self, field):
		self.field = cv.imread(field, 1)


	"""plot an orange 7 pixels radius circle as the ball"""
	def plot_ball(self, ball_center, color):

		cv.circle(self.field, ball_center, 7, color, -1)


	"""plots all contours from all robots of a designed color given as parameter"""
	def plot_robots(self,robots_contours, color):

		cv.drawContours(self.field, robots_contours, -1, color, -1)


	"""load a whole new clean arena without any contour in it"""
	def clean(self):
		self.field = cv.imread('vsss-field-600x520.jpg',1)