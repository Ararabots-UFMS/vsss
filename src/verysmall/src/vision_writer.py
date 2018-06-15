#!/usr/bin/env python
import rospy
from verysmall.msg import things_position
from verysmall.msg import five_robot_pos, five_robot_vector

class arena():
	def __init__(self):
		self.pub = rospy.Publisher('things_position', things_position, queue_size=1)
		rospy.init_node('vision', anonymous=True)
		rate = rospy.Rate(10) # 10hz

		# uint32[2]		 	        	ball_pos
		# float64[2]        				ball_vector
		# verysmall/five_robot_pos       	team_pos
		# verysmall/five_robot_vector    	team_vector
		# verysmall/five_robot_pos       	enemies_pos
		# verysmall/five_robot_vector   	enemies_vector
		# uint32[10]		 	        	robot_speed

		while not rospy.is_shutdown():
			self.publish(
				[0,0],
				[1.0,1.0],
				five_robot_pos([0,0],[0,0],[0,0],[0,0],[0,0]),
				five_robot_vector(1.0,1.0,1.0,1.0,1.0),
				five_robot_pos([0,0],[0,0],[0,0],[0,0],[0,0]),
				five_robot_vector(1.0,1.0,1.0,1.0,1.0),
				[0,0,0,0,0,0,0,0,0,0]
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