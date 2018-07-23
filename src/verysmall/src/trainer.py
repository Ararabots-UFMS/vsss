#!/usr/bin/python
from robot.robot import Robot
from verysmall.srv import manage_mac
import rospy
import roslaunch


class Trainer:
    """Creates the Trainer"""

    def __init__(self, _robot_params, _robot_bluetooth, _robot_roles, _game_opt, _launcher=None,):

        # Save the parameters for future use
        self.robot_params = _robot_params
        self.robot_bluetooth = _robot_bluetooth
        self.robot_roles = _robot_roles
        self.game_opt = _game_opt

        if _launcher is None:
            # Create roslaunch from API
            self.launcher = roslaunch.scriptapi.ROSLaunch()
            self.launcher.start()
        else:
            self.launcher = _launcher

        # Allocate robots process
        self.player_process = {}

        # Doing loops for creating the robot nodes
        for robot in self.robot_params.keys():
            # arguments for the node
            bluetooth_number = self.robot_params[robot]['bluetooth_mac_address']
            variables = robot + ' ' + self.robot_bluetooth[bluetooth_number] + " " + self.robot_params[robot]['body_id']

            # creates a node with robot list arguments
            node = roslaunch.core.Node('verysmall', 'robot_node.py',
                                       name=robot,
                                       args=variables)
            # launches the node and stores it in the given memory space
            self.player_process[robot] = self.launcher.launch(node)