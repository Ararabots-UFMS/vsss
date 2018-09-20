from attacker_with_univector import AttackerWithUnivector, MyModel
import sys
import os
sys.path[0] = path = root_path = os.environ['ROS_ARARA_ROOT']+"src/"
path += 'parameters/robots_pid.json'
from robot.movement.functions.movement import Movement
from utils.json_handler import JsonHandler
import rospy
import numpy as np

jsonHandler = JsonHandler()
univector_list = jsonHandler.read(path)

KP = univector_list['robot_1']['KP']
KD = univector_list['robot_1']['KD']
KI = univector_list['robot_1']['KI']


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

        self.movement = Movement([])

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
        if self.AttackerWithUnivector.is_stop:
            self.AttackerWithUnivector.stop_to_normal()

        if self.AttackerWithUnivector.is_normal:
            self.AttackerWithUnivector.normal_to_univector()

        if self.AttackerWithUnivector.is_univector:
            return self.in_univector_state()

    def in_freeball_game(self):
        """
        Transitions in freeball state

        :return: int, int
        """
        if self.AttackerWithUnivector.is_stop:
            self.AttackerWithUnivector.stop_to_freeball()

        if self.AttackerWithUnivector.is_freeball:
            self.AttackerWithUnivector.freeball_to_normal()

        if self.AttackerWithUnivector.is_normal:
            return self.in_normal_game()

    def in_penalty_game(self):
        """
        Transitions in penalty state

        :return: int, int
        """
        if self.AttackerWithUnivector.is_stop:
            self.AttackerWithUnivector.stop_to_penalty()

        if self.AttackerWithUnivector.is_penalty:
            self.AttackerWithUnivector.penalty_to_normal()

        if self.AttackerWithUnivector.is_normal:
            return self.in_normal_game()

    def in_meta_game(self):
        """
        Transitions in meta state

        :return: int, int
        """
        if self.AttackerWithUnivector.is_stop:
            self.AttackerWithUnivector.stop_to_meta()

        if self.AttackerWithUnivector.is_meta:
            self.AttackerWithUnivector.meta_to_normal()

        if self.AttackerWithUnivector.is_normal:
            return self.in_normal_game()

    def in_univector_state(self):
        """
        State univector return left wheel and right wheel speeds

        :return: int, int
        """
        self.AttackerWithUnivector.univector_to_univector()
        left, right, _ = self.movement.do_univector(
            speed = 150,
            robot_position=self.position,
            robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
            robot_speed=[0, 0],
            obstacle_position=np.resize(self.enemies_position, (5, 2)),
            obstacle_speed=[[0,0]]*5,
            ball_position=self.ball_position
        )
        return left, right