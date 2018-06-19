from robot.robot import Robot
import rospy

class Team():
	"""Crates all the robots and Trainer"""
	def __init__(self):
		self.robot_1 = Robot(1, "bluetooth_id", "robot_body" , isActive = False)
		self.robot_2 = Robot(2, "bluetooth_id_2", "robot_body_2")
		self.robot_3 = Robot(3, "bluetooth_id_3", "robot_body_3")
		
		

if __name__ == '__main__':
	rospy.init_node('team', anonymous=False)
	Team()

	rospy.spin()