import cv2 as cv
import math
import numpy as np
import time
from auxiliary import *


class Virtual_Field():

	"""class constructor using a .jpg image as background"""
	def __init__(self):
		# 1 cm for 4 pixels
		self.field = np.zeros((680,760,3), np.uint8)
		self.raw_field = np.zeros((680,760,3), np.uint8)
		self.top_left = (80,80)
		self.top_right = (680,80)
		self.bottom_left = (80,600)
		self.bottom_right = (680,600)
		self.field_origin = (40,600)
		self.ball_radius = 7	# pixels
		self.robot_side_size = 28	# pixels
		self.colors = {	"blue":[255,0,0],
						"orange":[0,100,255],
						"white":[255,255,255],
						"yellow":[0,255,255],
						"red":[0,0,255],
						"green":[0,253,116],
						"gray":[111,111,111]
					}

	"""system pause for n FPS"""
	def pause(self, n):
		time.sleep(1.0/n) 


	"""plots all arena contours and inner lines"""
	def plot_arena(self):
		self.field = np.zeros((680,760,3), np.uint8)

		#main field
		cv.line(self.field, self.top_left, self.top_right, self.colors["white"])
		cv.line(self.field, self.top_right, self.bottom_right, self.colors["white"])
		cv.line(self.field, self.bottom_right, self.bottom_left, self.colors["white"])
		cv.line(self.field, self.bottom_left, self.top_left, self.colors["white"])
		cv.line(self.field, (380,80), (380,600), self.colors["white"])
		cv.circle(self.field, (380,340), 80, self.colors["white"], 0)

		#goal 1
		cv.line(self.field, (40,260), (40,420), self.colors["white"])
		cv.line(self.field, (40,260), (80,260), self.colors["white"])
		cv.line(self.field, (40,420), (80,420), self.colors["white"])

		#goal 2
		cv.line(self.field, (720,260), (720,420), self.colors["white"])
		cv.line(self.field, (720,260), (680,260), self.colors["white"])
		cv.line(self.field, (720,420), (680,420), self.colors["white"])

		#corners
		cv.line(self.field, (108,80), (80,108), self.colors["white"])
		cv.line(self.field, (680,108), (652,80), self.colors["white"])
		cv.line(self.field, (80,572), (108,600), self.colors["white"])
		cv.line(self.field, (680,572), (652,600), self.colors["white"])

		#goal areas
		cv.line(self.field, (140,200), (140,480), self.colors["white"])
		cv.line(self.field, (80,200), (140,200), self.colors["white"])
		cv.line(self.field, (80,480), (140,480), self.colors["white"])
		cv.ellipse(self.field, (140,340), (20,40), 180.0, 270.0, 90.0, self.colors["white"])

		cv.line(self.field, (620,200), (620,480), self.colors["white"])
		cv.line(self.field, (620,200), (680,200), self.colors["white"])
		cv.line(self.field, (620,480), (680,480), self.colors["white"])
		cv.ellipse(self.field, (620,340), (20,40), 0.0, 90.0, 270.0, self.colors["white"])

		
	"""plot an orange 7 pixels radius circle as the ball"""
	def plot_ball(self, ball_center):

		ball_center = position_from_origin(unit_convert(ball_center))

		r1 = range(40,80)
		r2 = range(260,420)
		r3 = range(680,720)
		r4 = range(260,420)
		r5 = range(81,140)
		r6 = range(200,480)
		r7 = range(620,679)

		if(ball_center[0] in r1 and ball_center[1] in r2):
			cv.rectangle(self.field, (41,261), (79,419), self.colors["green"], -1) 
		elif(ball_center[0] in r3 and ball_center[1] in r4):
			cv.rectangle(self.field, (681,261), (719,419), self.colors["green"], -1)
		elif(ball_center[0] in r5 and ball_center[1] in r6):
			cv.rectangle(self.field, (81,201), (139,479), self.colors["green"], -1)
			cv.ellipse(self.field, (140,340), (19,39), 180.0, 270.0, 90.0, self.colors["green"], -1)
		elif(ball_center[0] in r7 and ball_center[1] in r6):
			cv.rectangle(self.field, (621,201), (679,479), self.colors["green"], -1)
			cv.ellipse(self.field, (620,340), (19,39), 0.0, 90.0, 270.0, self.colors["green"], -1)
		elif((ball_center[0] - 620)**2/400 + (ball_center[1] - 340)**2/1600 < 1):
			cv.rectangle(self.field, (621,201), (679,479), self.colors["green"], -1)
			cv.ellipse(self.field, (620,340), (19,39), 0.0, 90.0, 270.0, self.colors["green"], -1)
		elif((ball_center[0] - 140)**2/400 + (ball_center[1] - 340)**2/1600 < 1):
			cv.rectangle(self.field, (81,201), (139,479), self.colors["green"], -1)
			cv.ellipse(self.field, (140,340), (19,39), 180.0, 270.0, 90.0, self.colors["green"], -1)

		cv.circle(self.field, ball_center, self.ball_radius, self.colors["orange"], -1)



	"""plots all contours from all robots of a designed color given as parameter"""	
	def plot_robots(self, robot_list, color):

		for robot in robot_list:

			vector = robot[1]
			center = position_from_origin(unit_convert(robot[0]))

			angle = angle_between([1,0], vector)*180/(math.pi)

			contour = (center, (self.robot_side_size, self.robot_side_size), angle)

			n_contour = cv.boxPoints(contour)
			n_contour = np.int0(n_contour)


			cv.drawContours(self.field, [n_contour], -1, color, -1)
			cv.arrowedLine(self.field, center, (int(center[0]+3*vector[0]), int(center[1]+3*vector[1])), self.colors["red"], 2)
			cv.arrowedLine(self.field, self.field_origin, (int(center[0]), int(center[1])), self.colors["gray"], 1)



