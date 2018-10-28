import sys
import os
import rospy
import numpy as np
from zagueiro import Zagueiro, MyModel
sys.path[0] = path = root_path = os.environ['ROS_ARARA_ROOT']+"src/robot/"
from movement.functions.movement import Movement
from utils.json_handler import JsonHandler
path += '../parameters/robots_pid.json'
from arena_sections import *
from ball_range import *

jsonHandler = JsonHandler()
univector_list = jsonHandler.read(path)

KP = univector_list['robot_1']['KP']
KD = univector_list['robot_1']['KD']
KI = univector_list['robot_1']['KI']

ZAGUEIRO_SPEED = 100
DEF_X_POS = 75.0/2.0

class ZagueiroController():

    def __init__(self):
        self.position = None
        self.orientation = None
        self.robot_speed = None
        self.enemies_position = None
        self.enemies_speed = None
        self.ball_position = None

        self.defend_position = np.array([0,0])

        self.stop = MyModel(state='stop')
        self.zagueiro = Zagueiro(self.stop)

        self.movement = Movement([KP, KD, KI], 10)

    def update_game_information(self, position, orientation, team_speed, enemies_position, enemies_speed, ball_position , team_side):
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
        self.team_speed = team_speed
        self.enemies_position = enemies_position
        self.enemies_speed = enemies_speed
        self.ball_position = ball_position
        self.team_side = team_side
        self.movement.univet_field.update_attack_side(not self.team_side)





    def set_to_stop_game(self):
        """
        Set state stop in the state machine

        :return: int, int
        """
        self.stop.state = 'stop'
        return 0, 0, 0

    def in_normal_game(self):
        #rospy.logfatal(self.zagueiro.current_state)

        """
        Transitions in normal game state

        :return: int, int
        """
        if self.zagueiro.is_stop:
            self.zagueiro.stop_to_normal()

        if self.zagueiro.is_normal:
            
            if(not self.team_side): ############    team_side =  0
                #rospy.logfatal("team side 0")
                if((section(self.ball_position) not in [LEFT_GOAL, LEFT_GOAL_AREA]) and self.ball_position[0] <= 75.0):
                    self.zagueiro.normal_to_defend()
                else:
                     self.zagueiro.normal_to_wait_ball()
            
            else:               ############    team_side = 1
                if((section(self.ball_position) not in [RIGHT_GOAL, RIGHT_GOAL_AREA]) and self.ball_position[0] > 75.0):
                    self.zagueiro.normal_to_defend()
                else:
                     self.zagueiro.normal_to_wait_ball()

        if self.zagueiro.is_defend:
            return self.in_defend()
        else:
            return self.in_wait_ball()


    def in_defend(self):
        #rospy.logfatal(self.zagueiro.current_state)

        if(not ((not self.team_side) and (section(self.ball_position) not in [LEFT_GOAL, LEFT_GOAL_AREA]) and self.ball_position[0] <= 75.0 or (self.team_side) and(section(self.ball_position) not in [RIGHT_GOAL, RIGHT_GOAL_AREA]) and self.ball_position[0] > 75.0)):
            self.zagueiro.defend_to_wait_ball()
            return self.in_wait_ball()
        else:
            if(near_ball(self.ball_position, self.position)):
                self.zagueiro.defend_to_spin()
                rospy.logfatal(self.zagueiro.current_state)
                return self.is_spin()
            else:
                self.zagueiro.defend_to_move()
                rospy.logfatal(self.zagueiro.current_state)
                return self.is_move();


    def is_spin(self):
        self.zagueiro.spin_to_defend()
        
        param1, param2, param3 = self.movement.spin(ZAGUEIRO_SPEED, spin_direction(self.ball_position, self.position, self.team_side))

        return param1, param2, param3

    def is_move(self):
        self.zagueiro.move_to_defend()
        param1, param2, param3 = self.movement.move_to_point(
            ZAGUEIRO_SPEED,
            self.position,
            [np.cos(self.orientation), np.sin(self.orientation)],
            self.ball_position
        )

        return param1, param2, param3

    def in_wait_ball(self):
        rospy.logfatal(self.zagueiro.current_state)
        
        if((not self.team_side) and (section(self.ball_position) not in [LEFT_GOAL, LEFT_GOAL_AREA]) and self.ball_position[0] <= 75.0 or (self.team_side) and(section(self.ball_position) not in [RIGHT_GOAL, RIGHT_GOAL_AREA]) and self.ball_position[0] > 75.0):
            self.zagueiro.wait_ball_to_defend()
            return self.in_defend()
        else:

            self.defend_position[0] = DEF_X_POS
            self.defend_position[1] = self.ball_position[1]

            param1, param2, param3 = self.movement.move_to_point(
                ZAGUEIRO_SPEED,
                self.position,
                [np.cos(self.orientation), np.sin(self.orientation)],
                self.defend_position
            )


            return param1, param2, param3




    def in_freeball_game(self):
        """
        Transitions in freeball state

        :return: int, int
        """
        if self.zagueiro.is_stop:
            self.zagueiro.stop_to_freeball()

        if self.zagueiro.is_freeball:
            self.zagueiro.freeball_to_normal()

        if self.zagueiro.is_normal:
            return self.in_normal_game()

    def in_penalty_game(self):
        """
        Transitions in penalty state

        :return: int, int
        """
        if self.zagueiro.is_stop:
            self.zagueiro.stop_to_penalty()

        if self.zagueiro.is_penalty:
            self.zagueiro.penalty_to_normal()

        if self.zagueiro.is_normal:
            return self.in_normal_game()

    def in_meta_game(self):
        """
        Transitions in meta state

        :return: int, int
        """
        if self.zagueiro.is_stop:
            self.zagueiro.stop_to_meta()

        if self.zagueiro.is_meta:
            self.zagueiro.meta_to_normal()

        if self.zagueiro.is_normal:
            return self.in_normal_game()
