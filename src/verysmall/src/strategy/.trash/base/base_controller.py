import sys
import os
import rospy
import numpy as np
from base_state_machine import RobotStateMachine, MyModel
sys.path[0] = path = root_path = os.environ['ROS_ARARA_ROOT']+"src/robot/"
from movement.functions.movement import Movement
from utils.json_handler import JsonHandler
path += '../parameters/bodies.json'

jsonHandler = JsonHandler()
bodies_unpack = jsonHandler.read(path)


class RobotStateMachineController():

    def __init__(self, _robot_body, _debug_topic = None):
        self.position = None
        self.orientation = None
        self.robot_speed = None
        self.enemies_position = None
        self.enemies_speed = None
        self.ball_position = None
        self.team_side = None
        self.pid_list = [0, 0, 0]
        self.robot_body = _robot_body
        self.update_pid()

        self.stop = MyModel(state='Stop')
        self.RobotStateMachine = RobotStateMachine(self.stop)

        self.movement = Movement(self.pid_list, 10)

    def update_game_information(self, robot):
        """
        Update game variables
        :param position:
        :param orientation:
        :param robot_speed:
        :param enemies_position:
        :param enemies_speed:
        :param ball_position:
        """
        self.position = robot.position
        self.orientation = robot.orientation
        self.robot_speed = robot.robot_speed
        self.enemies_position = robot.enemies_position
        self.enemies_speed = robot.enemies_speed
        self.ball_position = robot.ball_position
        self.team_side = robot.team_side
        self.movement.univet_field.update_attack_side(not self.team_side)

    def update_pid(self):
        """
        Update pid
        :return:
        """
        self.pid_list = [bodies_unpack[self.robot_body]['KP'], bodies_unpack[self.robot_body]['KD'], bodies_unpack[self.robot_body]['KI']]
        self.movement.update_pid(self.pid_list)

    def set_to_stop_game(self):
        """
        Set state stop in the state machine

        :return: int, int
        """
        self.stop.state = 'stop'
        return 0, 0

    def in_normal_game(self):
        """
        Transitions in normal game state

        :return: int, int
        """
        if self.RobotStateMachine.is_stop:
            self.RobotStateMachine.stop_to_normal()

        if self.RobotStateMachine.is_normal:
            self.RobotStateMachine.normal_to_univector()

    def in_freeball_game(self):
        """
        Transitions in freeball state

        :return: int, int
        """
        if self.RobotStateMachine.is_stop:
            self.RobotStateMachine.stop_to_freeball()

        if self.RobotStateMachine.is_freeball:
            self.RobotStateMachine.freeball_to_normal()

        if self.RobotStateMachine.is_normal:
            return self.in_normal_game()

    def in_penalty_game(self):
        """
        Transitions in penalty state

        :return: int, int
        """
        if self.RobotStateMachine.is_stop:
            self.RobotStateMachine.stop_to_penalty()

        if self.RobotStateMachine.is_penalty:
            self.RobotStateMachine.penalty_to_normal()

        if self.RobotStateMachine.is_normal:
            return self.in_normal_game()

    def in_meta_game(self):
        """
        Transitions in meta state

        :return: int, int
        """
        if self.RobotStateMachine.is_stop:
            self.RobotStateMachine.stop_to_meta()

        if self.RobotStateMachine.is_meta:
            self.RobotStateMachine.meta_to_normal()

        if self.RobotStateMachine.is_normal:
            return self.in_normal_game()
