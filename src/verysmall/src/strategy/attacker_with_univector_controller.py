import sys
import os
import rospy
import numpy as np
from attacker_with_univector import AttackerWithUnivector, MyModel
sys.path[0] = path = root_path = os.environ['ROS_ARARA_ROOT']+"src/robot/"
from movement.functions.movement import Movement
from utils.json_handler import JsonHandler
from rospy import logfatal
path += '../parameters/bodies.json'

jsonHandler = JsonHandler()
bodies_unpack = jsonHandler.read(path, escape=True)

SOFTWARE = 0
HARDWARE = 1

class AttackerWithUnivectorController():

    def __init__(self, _robot_obj ,_robot_body="Nenhum", _debug_topic = None):
        self.pid_type = SOFTWARE
        self.robot = _robot_obj
        self.position = None
        self.orientation = None
        self.speed = None
        self.team_speed = None
        self.enemies_position = None
        self.enemies_speed = None
        self.ball_position = None
        self.team_side = None
        self.robot_body = _robot_body
        rospy.logfatal(self.robot_body)
        self.pid_list = [bodies_unpack[self.robot_body]['KP'],
                         bodies_unpack[self.robot_body]['KI'],
                         bodies_unpack[self.robot_body]['KD']]

        #Attack_in left side
        self.attack_goal = np.array([150.0, 65.0])


        self.stop = MyModel(state='stop')
        self.AttackerWithUnivector = AttackerWithUnivector(self.stop)

        self.movement = Movement(self.pid_list, error=10, attack_goal=self.attack_goal, _pid_type=self.pid_type, _debug_topic=_debug_topic)

    def set_pid_type(self, _type):
        """
        Change pid type
        :return:
        """
        self.pid_type = _type
        self.movement.set_pid_type(_type=self.pid_type)

    def update_game_information(self):
        """
        Update game variables
        :param robot: robot obj
        """
        self.position = self.robot.position
        self.orientation = self.robot.orientation
        self.speed = self.robot.speed
        self.team_speed = self.robot.team_speed
        self.enemies_position = self.robot.enemies_position
        self.enemies_speed = self.robot.enemies_speed
        self.ball_position = self.robot.ball_position
        self.team_side = self.robot.team_side
        self.movement.univet_field.update_attack_side(not self.team_side)

    def update_pid(self):
        """
        Update pid
        :return:
        """
        self.pid_list = [bodies_unpack[self.robot_body]['KP'], bodies_unpack[self.robot_body]['KI'], bodies_unpack[self.robot_body]['KD']]
        self.movement.update_pid(self.pid_list)

    def set_to_stop_game(self):
        """
        Set state stop in the state machine

        :return: int, int
        """
        self.stop.state = 'stop'
        return 0, 0, SOFTWARE

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
        self.AttackerWithUnivector.univector_to_univector()
        param_a, param_b, _ = self.movement.do_univector_velo(
            speed=200,
            robot_position=self.position,
            robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
            robot_speed=np.array([0, 0]),
            obstacle_position=self.enemies_position,
            obstacle_speed=[[0,0]]*5,
            ball_position=self.ball_position
        )
        param_a, param_b, _ = self.movement.do_univector(
            speed=100,
            robot_position=self.position,
            robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
            robot_speed=np.array([0, 0]),
            obstacle_position=self.enemies_position,
            obstacle_speed=[[0, 0]]*5,
            ball_position=self.ball_position
        )
        # param_a, param_b, _ = self.movement.move_to_point(100, self.position, [np.cos(self.orientation), np.sin(self.orientation)], [65, 65])
        # logfatal(str(param_a))
        # logfatal(str(param_b))

        return param_a, param_b, self.pid_type
