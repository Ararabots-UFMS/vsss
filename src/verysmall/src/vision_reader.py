#!/usr/bin/env python
import rospy
from verysmall.msg import things_position
from verysmall.msg import five_robot_pos, five_robot_vector

class virtual_field():
	def __init__(self):
		rospy.Subscriber('things_position', things_position, self.read)
		rospy.init_node('virtual_field', anonymous=True)

		# uint32[2]		 	        	ball_pos
		# float64[2]        				ball_vector
		# verysmall/five_robot_pos       	team_pos
		# verysmall/five_robot_vector    	team_vector
		# verysmall/five_robot_pos       	enemies_pos
		# verysmall/five_robot_vector   	enemies_vector
		# uint32[10]		 	        	robot_speed

		rospy.spin()

	def read(self, data):
		print(data)

if __name__ == '__main__':
	try:
		virtual_field()
	except rospy.ROSInterruptException:
		pass