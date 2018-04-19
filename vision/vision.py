#!/usr/bin/env python
# -*- coding: utf-8 -*-

import camera
import cv2
import numpy as np
import json
import sys

#sys.path.append('../parameters')


class Vision():


	def __init__(self, camera, load_params=True):
		super(vision, self).__init__()
		self.raw_image = camera.read()
		self.ball_position = None
		self.ball_vector = None
		self.team_position = None
		self.team_vector = None
		self.team_color = None
		self.enemies_position = None
		self.enemies_vector = None
		self.enemies_color = None
		self.robots_speed = None

		''' load correction parameters of a .json file'''
		if load_params:
			load_parameters()



	''' apply lens correction to the frame '''
	def lens_correction(self):

		pass

	''' apply perspective correction to the frame '''
	#http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_geometric_transformations/py_geometric_transformations.html
	def warp_perspective(self):

		pass

	''' loads each camera and field parameters presents in each .json file '''
	def load_parameters(self):
		camera.load_params()
		#ler parametros do .json referente à arena


	def field_segmentation(self):
		pass

	def color_segmentation(self):
		pass

	def orientation_vectors(self):
		pass

	def ball_vector(self):
		pass

	def speed_prediction(self):
		pass

	def position_prediction(self):
		pass

	def find_robots(self):
		pass

	def enemy_tracking(self):
		pass

	def debug(self):
		pass
