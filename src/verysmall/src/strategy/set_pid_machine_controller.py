import sys
import os
import rospy
import numpy as np
from .set_pid_machine import SetPIDMachine, MyModel
from robot_module.movement.functions.movement import Movement
from utils.json_handler import JsonHandler
bodies_unpack = JsonHandler().read("parameters/bodies.json", escape = True)

class SetPIDMachineController():

    def __init__(self, _robot_obj ,_robot_body = "Nenhum", _debug_topic = None):
        self.position = None
        self.robot = _robot_obj
        self.orientation = None
        self.robot_speed = None
        self.enemies_position = None
        self.enemies_speed = None
        self.ball_position = None
        self.robot_body = _robot_body
        rospy.logfatal(self.robot_body)
        self.pid_list = [bodies_unpack[self.robot_body]['KP'], bodies_unpack[self.robot_body]['KI'],
                         bodies_unpack[self.robot_body]['KD']]

        #Attack_in left side
        self.attack_goal = np.array([0.0, 65.0])

        self.stop = MyModel(state='stop')
        self.SetPIDMachine = SetPIDMachine(self.stop)

        self.movement = Movement(self.pid_list, error=10, attack_goal=self.attack_goal, _debug_topic = _debug_topic)

    def update_game_information(self):
        """
        Update game variables
        :param robot: robot object
        """
        self.position = self.robot.position
        self.orientation = self.robot.orientation
        self.team_speed = self.robot.team_speed
        self.enemies_position = self.robot.enemies_position
        self.enemies_speed = self.robot.enemies_speed
        self.ball_position = self.robot.ball_position
        self.team_side = self.robot.team_side
        self.attack_goal[0] = 0.0 + (not self.team_side)*150
        self.movement.univet_field.update_attack_side(not self.team_side)


    def update_pid(self):
        """
        Update pid
        :return:
        """
        self.pid_list = [bodies_unpack[self.robot_body]['KP'], bodies_unpack[self.robot_body]['KI'],
                         bodies_unpack[self.robot_body]['KD']]

        self.movement.update_pid(self.pid_list)


    def set_to_stop_game(self):
        """
        Set state stop in the state machine

        :return: int, int
        """
        self.stop.state = 'stop'
        return 0, 0, 0

    def in_normal_game(self):
        """
        Transitions in normal game state

        :return: int, int
        """
        if self.SetPIDMachine.is_stop:
            self.SetPIDMachine.stop_to_normal()

        if self.SetPIDMachine.is_normal:
            self.SetPIDMachine.normal_to_univector()

        if self.SetPIDMachine.is_univector:
            return self.in_univector_state()

    def in_freeball_game(self):
        """
        Transitions in freeball state

        :return: int, int
        """
        if self.SetPIDMachine.is_stop:
            self.SetPIDMachine.stop_to_freeball()

        if self.SetPIDMachine.is_freeball:
            self.SetPIDMachine.freeball_to_normal()

        if self.SetPIDMachine.is_normal:
            return self.in_normal_game()

    def in_penalty_game(self):
        """
        Transitions in penalty state

        :return: int, int
        """
        if self.SetPIDMachine.is_stop:
            self.SetPIDMachine.stop_to_penalty()

        if self.SetPIDMachine.is_penalty:
            self.SetPIDMachine.penalty_to_normal()

        if self.SetPIDMachine.is_normal:
            return self.in_normal_game()

    def in_meta_game(self):
        """
        Transitions in meta state

        :return: int, int
        """
        if self.SetPIDMachine.is_stop:
            self.SetPIDMachine.stop_to_meta()

        if self.SetPIDMachine.is_meta:
            self.SetPIDMachine.meta_to_normal()

        if self.SetPIDMachine.is_normal:
            return self.in_normal_game()

    def in_univector_state(self):
        """
        State univector return left wheel and right wheel speeds

        :return: int, int
        """
        self.SetPIDMachine.univector_to_univector()
        #
        # left, right, _ = self.movement.do_univector(
        #     speed=100,
        #     robot_position=self.position,
        #     robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
        #     robot_speed=np.array([0, 0]),
        #     obstacle_position=self.enemies_position,
        #     obstacle_speed=[[0,0]]*5,
        #     ball_position=self.ball_position
        # )
        #left, right, _ =  self.movement.follow_vector(speed=200,
        #             robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
        #             goal_vector=np.array([1 + -2*self.team_side,0]))

        left, right, param_c = self.movement.move_to_point(100, np.array(self.position),[np.cos(self.orientation), np.sin(self.orientation)],  np.array([130, 50]))
        rospy.logfatal("Speed " + str(left)+" "+str(right))
        return left, right, 0
        # return 100, 0, 0
