import rospy
from verysmall.msg import motor_speed

class Bluetooth():
	"""docstring for Bluetooth"""
	def __init__(self, active_robots = [1,2,3]):
		self.robots_motor_speed = [0,0] * len(active_robots)
		for robot_id in active_robots:
			rospy.Subscriber('things_position', things_position, self.read_topic)