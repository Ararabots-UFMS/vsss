import sys
import os
import rospy
import numpy as np
from naive_keeper import NaiveGK, MyModel
sys.path[0] = path = root_path = os.environ['ROS_ARARA_ROOT']+"src/robot/"
from movement.functions.movement import Movement
from utils.json_handler import JsonHandler
import arena_sections as sections

path += '../parameters/robots_pid.json'

jsonHandler = JsonHandler()
univector_list = jsonHandler.read(path)

KP = univector_list['robot_1']['KP']
KD = univector_list['robot_1']['KD']
KI = univector_list['robot_1']['KI']


class NaiveGKController():

    def __init__(self):
        self.position = None
        self.orientation = None
        self.robot_speed = None
        self.enemies_position = None
        self.enemies_speed = None
        self.ball_position = None
        self.team_side = None

        self.stop = MyModel(state='Stop')
        self.NaiveGK = NaiveGK(self.stop)

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
        self.team_side = team_side

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
        if self.NaiveGK.is_stop:
            self.NaiveGK.stop_to_normal()

        if self.NaiveGK.is_normal:
            if self.team_side == LEFT: #left

                if section.section(self.ball_position) == LEFT_GOAL_AREA:
                    #defend
                else:
                    #trackeia

            else: #right
                if section.section(self.ball_position) == RIGHT_GOAL_AREA:
                    #defend
                else:
                    #trackeia


    def in_freeball_game(self):
        """
        Transitions in freeball state

        :return: int, int
        """
        if self.NaiveGK.is_stop:
            self.NaiveGK.stop_to_freeball()

        if self.NaiveGK.is_freeball:
            self.NaiveGK.freeball_to_normal()

        if self.NaiveGK.is_normal:
            return self.in_normal_game()

    def in_penalty_game(self):
        """
        Transitions in penalty state

        :return: int, int
        """
        if self.NaiveGK.is_stop:
            self.NaiveGK.stop_to_penalty()

        if self.NaiveGK.is_penalty:
            self.NaiveGK.penalty_to_normal()

        if self.NaiveGK.is_normal:
            return self.in_normal_game()

    def in_meta_game(self):
        """
        Transitions in meta state

        :return: int, int
        """
        if self.NaiveGK.is_stop:
            self.NaiveGK.stop_to_meta()

        if self.NaiveGK.is_meta:
            self.NaiveGK.meta_to_normal()

        if self.NaiveGK.is_normal:
            return self.in_normal_game()
