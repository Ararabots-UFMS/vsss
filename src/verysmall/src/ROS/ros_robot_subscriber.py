#!/usr/bin/python
from verysmall.msg import things_position, game_topic
import rospy
import numpy as np


class RosRobotSubscriber:
    """
    This class is responsible for reading and formatting ros messages for the robot node
    """
    def __init__(self, _robot):
        """
        :param _robot: robot object
        """
        rospy.Subscriber('things_position', things_position, self.read_topic)

        rospy.Subscriber('game_topic', game_topic, self.read_game_topic)

        self.robot = _robot

    def read_game_topic(self, data):
        """
        Read from game topic callback and open the message into robot variables
        :param data: ROS game topic message
        :return: nothing
        """
        self.robot.game_state = data.game_state
        self.robot.team_side = data.team_side
        self.robot.role = data.robot_roles[self.robot.robot_id_integer]
        self.robot.penalty_robot = data.penalty_robot
        self.robot.freeball_robot = data.freeball_robot
        self.robot.meta_robot = data.meta_robot
        self.robot.changed_game_state = True

    def read_topic(self, data):
        """
        This class formats the things position into np arrays and replaces any nan to None
        :param data: ROS Things position message
        :return: nothing
        """
        self.robot.ball_position = np.nan_to_num(np.array(data.ball_pos))
        self.robot.ball_speed = np.nan_to_num(np.array(data.ball_speed))
        self.robot.team_pos = np.nan_to_num(np.array(data.team_pos)).reshape((5, 2))
        self.robot.team_orientation = np.nan_to_num(np.array(data.team_orientation))
        self.robot.team_speed = np.nan_to_num(np.array(data.team_speed)).reshape((5, 2))
        self.robot.position = self.robot.team_pos[self.robot.tag]
        self.robot.orientation = self.robot.team_orientation[self.robot.tag]
        self.robot.enemies_position = np.nan_to_num(data.enemies_pos).reshape((5, 2))
        self.robot.enemies_orientation = np.nan_to_num(data.enemies_orientation)
        self.robot.enemies_speed = np.nan_to_num(data.enemies_pos).reshape((5, 2))

        self.robot.run()
