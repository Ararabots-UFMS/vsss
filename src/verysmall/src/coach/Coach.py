#!/usr/bin/python
from robot.robot import Robot
from sys import path
from os import environ
old_path = path[0]
path[0] = environ['ROS_ARARA_ROOT'] + "src/"
from ROS.ros_coach import RosCoach
path[0] = old_path


class Coach:
    """Creates the Coach"""

    def __init__(self, _robot_params, _robot_bluetooth, _robot_roles, _game_opt, _game_topic_publisher, _launcher=None):

        # Save the parameters for future use
        self.robot_params = _robot_params
        self.robot_bluetooth = _robot_bluetooth
        self.robot_roles = _robot_roles
        self.game_opt = _game_opt
        self.pub = _game_topic_publisher

        # Fast access array to use a dict as an simple array
        self.faster_hash = ['robot_' + str(x) for x in range(1, 6)]

        self.ros_functions = RosCoach(_launcher)

        self.create_robots()

    def create_robots(self):
        """
        Reading from database, creates nodes and store them in process
        :return: nothing
        """
        # Doing loops for creating the robot nodes
        for robot in self.robot_params.keys():
            # arguments for the node
            try:
                bluetooth_number = self.robot_params[robot]['bluetooth_mac_address']
                bluetooth_address = self.robot_bluetooth[bluetooth_number]
            except KeyError:
                bluetooth_address = str(-1)

            try:
                tag = self.robot_params[robot]['tag_number']
                body = self.robot_params[robot]['body_id']
                variables = robot + ' ' + str(tag) + ' ' + bluetooth_address + " " + body
            except KeyError:
                variables = ""
                self.robot_params[robot]['active'] = False

            # Creates a player node
            self.ros_functions.create_and_store_node(robot, self.robot_params[robot]['active'], variables)

    def change_robot_role(self, robot_id, role):
        """
        This function changes the coach role sugestion and publishes through game_topic
        :param robot_id: int
        :param role: int
        :return: nothing
        """
        # TODO: alterar objeto Coach
        # TODO: alterar funcao para receber string e transformar em int para publicar
        self.pub.set_robot_role(robot_id, role)
        self.pub.publish()

    def set_robot_parameters(self, robot_id):
        """
        With robot_id, sets all the argument for node
        :param robot_id: int
        :return: nothing
        """
        robot = self.faster_hash[robot_id]

        # arguments for the node
        bluetooth_number = self.robot_params[robot]['bluetooth_mac_address']
        tag = self.robot_params[robot]['tag_number']
        body = self.robot_params[robot]['body_id']
        variables = robot + ' ' + str(tag) + ' ' + self.robot_bluetooth[bluetooth_number] + " " + body

        self.ros_functions.change_arguments_of_node(robot, variables)

    def set_robot_active(self, robot_id, should_be_active):
        """
        Sets a defined robot_id node to start or stop
        :param robot_id: int
        :param should_be_active: bool
        :return: nothing
        """
        robot_name = self.faster_hash[robot_id]
        self.ros_functions.toggle_node_life(robot_name, should_be_active)

    #TODO: Definir run aqui e importala de outro arquivo

    #TODO: Criar pasta para estrategias, paralelo a robot na raiz
