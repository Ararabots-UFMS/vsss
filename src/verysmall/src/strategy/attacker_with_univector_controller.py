from attacker_with_univector import AttackerWithUnivector, MyModel
import sys
import os
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT']+"src/"
from robot.movement.functions.movement import Movement
from utils.json_handler import JsonHandler
import rospy
import numpy as np



class AttackerWithUnivectorController():

    def __init__(self):
        self.position = None
        self.orientation = None
        self.robot_speed = None
        self.enemies_position = None
        self.enemies_speed = None
        self.ball_position = None

        self.stop = MyModel(state='Stop')
        self.AttackerWithUnivector = AttackerWithUnivector(self.stop)


    def update_game_information(self, position, orientation, robot_speed, enemies_position, enemies_speed, ball_position):
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
        self.stop.state = 'stop'
        return 0, 0

    def in_normal_game(self):
        if self.AttackerWithUnivector.is_stop:
            self.AttackerWithUnivector.stop_to_normal()

        elif self.AttackerWithUnivector.is_normal:
            self.AttackerWithUnivector.normal_to_univector()

        elif self.AttackerWithUnivector.is_univector:
            self.AttackerWithUnivector.univector_to_univector()

    def in_freeball_game(self):
        self.AttackerWithUnivector.stop_to_freeball()
        self.AttackerWithUnivector.freeball_to_normal()

    def in_penalty_game(self):
        self.AttackerWithUnivector.stop_to_penalty()
        self.AttackerWithUnivector.penalty_to_normal()

    def in_meta_game(self):
        self.AttackerWithUnivector.stop_to_meta()
        self.AttackerWithUnivector.meta_to_normal()

    def in_univector_state(self):
        self.AttackerWithUnivector.univector_to_univector()
