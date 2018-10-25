import sys
import os
import rospy
import numpy as np
from attacker_with_univector import AttackerWithUnivector, MyModel
sys.path[0] = path = root_path = os.environ['ROS_ARARA_ROOT']+"src/robot/"
from movement.functions.movement import Movement
from utils.json_handler import JsonHandler
path += '../parameters/bodies.json'

jsonHandler = JsonHandler()
<<<<<<< HEAD
bodies_unpack = jsonHandler.read(path, escape = True)
=======
univector_list = jsonHandler.read(path)
HARDWARE = 1

KP = univector_list['robot_1']['KP']
KD = univector_list['robot_1']['KD']
KI = univector_list['robot_1']['KI']

>>>>>>> running-update

class AttackerWithUnivectorController():

    def __init__(self, _robot_body = "Nenhum", _debug_topic = None):
        self.position = None
        self.orientation = None
        self.robot_speed = None
        self.enemies_position = None
        self.enemies_speed = None
        self.ball_position = None
        self.robot_body = _robot_body
        rospy.logfatal(self.robot_body)
        self.pid_list = [bodies_unpack[self.robot_body]['KP'], bodies_unpack[self.robot_body]['KD'],
                         bodies_unpack[self.robot_body]['KI']]

        #Attack_in left side
        self.attack_goal = np.array([150.0, 65.0])

        self.stop = MyModel(state='stop')
        self.AttackerWithUnivector = AttackerWithUnivector(self.stop)

<<<<<<< HEAD
        self.movement = Movement(self.pid_list, error=10, attack_goal=self.attack_goal, _debug_topic = _debug_topic)
=======
        self.movement = Movement([KP, KD, KI], error=10, attack_goal=self.attack_goal,_pid_type = HARDWARE, _debug_topic = _debug_topic)
>>>>>>> running-update

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
        self.movement.univet_field.update_attack_side(not self.team_side)

<<<<<<< HEAD

    def update_pid(self):
        """
        Update pid
        :return:
        """
        self.pid_list = [bodies_unpack[self.robot_body]['KP'], bodies_unpack[self.robot_body]['KD'], bodies_unpack[self.robot_body]['KI']]
        self.movement.update_pid(self.pid_list)

        
=======
>>>>>>> running-update
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
<<<<<<< HEAD
        left, right, _ = self.movement.do_univector(
            speed = 100,
=======
        left, right= self.movement.do_univector(
            speed = 180,
>>>>>>> running-update
            robot_position=self.position,
            robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
            robot_speed=[0, 0],
            obstacle_position=self.enemies_position,
            obstacle_speed=[[0,0]]*5,
            ball_position=self.ball_position
        )
        return left, right
