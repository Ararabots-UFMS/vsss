import sys
import os
import rospy
import numpy as np
from base_state_machine import RobotStateMachine, MyModel
sys.path[0] = path = root_path = os.environ['ROS_ARARA_ROOT']+"src/robot/"
from movement.functions.movement import Movement
from utils.json_handler import JsonHandler
path += '../parameters/robots_pid.json'

jsonHandler = JsonHandler()
univector_list = jsonHandler.read(path)

KP = univector_list['robot_1']['KP']
KD = univector_list['robot_1']['KD']
KI = univector_list['robot_1']['KI']


class RobotStateMachineController():

    def __init__(self):
        self.position = None
        self.orientation = None
        self.robot_speed = None
        self.enemies_position = None
        self.enemies_speed = None
        self.ball_position = None

        self.stop = MyModel(state='Stop')
        self.RobotStateMachine = RobotStateMachine(self.stop)

        self.movement = Movement([KP, KD, KI], 10)

    def update_game_information(self, position, orientation, robot_speed, enemies_position, enemies_speed, ball_position, team_side):
        """
        Update game variables
        :param position:
        :param orientation:
        :param robot_speed:
        :param enemies_position:
        :param enemies_speed:
        :param ball_position:
        """
        self.position = position
        self.orientation = orientation
        self.robot_speed = robot_speed
        self.enemies_position = enemies_position
        self.enemies_speed = enemies_speed
        self.ball_position = ball_position

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
