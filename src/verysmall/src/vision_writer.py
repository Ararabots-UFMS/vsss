#!/usr/bin/env python
import rospy
import math
from verysmall.msg import things_position
from verysmall.msg import robot_pos, robot_vector
from random import uniform


class arena():
	def __init__(self):
		self.pub = rospy.Publisher('things_position', things_position, queue_size=1)
		rospy.init_node('vision', anonymous=True)
		rate = rospy.Rate(60) # 10hz

		# uint32[2]		 	        	ball_pos
		# float64[2]        				ball_vector
		# verysmall/five_robot_pos       	team_pos
		# verysmall/five_robot_vector    	team_vector
		# verysmall/five_robot_pos       	enemies_pos
		# verysmall/five_robot_vector   	enemies_vector
		# uint32[10]		 	        	robot_speed
		#example of how to use the virtual_field class

		#gerenation of objects, must be changed for ROS update os values 
		#numbers in centimeters for ball and robots
		#sin_x = 0.0
		cycle = 0
		ball_x = ball_y = 0
		moving_x = True
		while not rospy.is_shutdown():
			
			#if sin_x == 148.0:
			#	sin_x = 0.0
			#else:
			#	sin_x+=0.1

			if moving_x:
				if ball_x == 148:
					ball_x = 0
					moving_x = not moving_x
				else:
					ball_x+=2
			else:
				if ball_y == 128:
					ball_y = 0
					moving_x = not moving_x
				else:
					ball_y+=2

			#[sin_x, int((math.sin(sin_x))*32+64)] ,
			self.publish(
				[0] * 2,
				0.,
				[robot_pos() for _ in range(5)],
				[robot_vector() for _ in range(5)],
				[robot_pos() for _ in range(5)],
				[robot_vector() for _ in range(5)],
				[0] * 10
			)	

			rate.sleep()

	def publish(self, ball_pos, ball_vector, team_pos, team_vector, enemies_pos, enemies_vector, robot_speed):
		msg = things_position(
			ball_pos, ball_vector, 
			team_pos, team_vector, 
			enemies_pos, enemies_vector, 
			robot_speed
		)
		self.pub.publish(msg)

if __name__ == '__main__':
	try:
		arena()
	except rospy.ROSInterruptException:
		pass