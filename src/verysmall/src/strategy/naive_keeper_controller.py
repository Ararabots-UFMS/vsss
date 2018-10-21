import sys
import os
import rospy
import numpy as np
from arena_sections import *
from ball_range import *
from naive_keeper import NaiveGK, MyModel
sys.path[0] = path = root_path = os.environ['ROS_ARARA_ROOT']+"src/robot/"
from movement.functions.movement import Movement
from utils.json_handler import JsonHandler
from arena_sections import *

path += '../parameters/robots_pid.json'

jsonHandler = JsonHandler()
univector_list = jsonHandler.read(path)

KP = univector_list['robot_1']['KP']
KD = univector_list['robot_1']['KD']
KI = univector_list['robot_1']['KI']


GOAL_KEEPER_SPEED = 255

class NaiveGKController():

    def __init__(self):
        self.position = None
        self.orientation = None
        self.robot_speed = None
        self.enemies_position = None
        self.enemies_speed = None
        self.ball_position = None
        self.team_side = None

        self.defend_position = np.array([0,0])

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
            rospy.logfatal(self.NaiveGK.current_state)
            self.NaiveGK.stop_to_normal()

        if self.NaiveGK.is_normal:
            rospy.logfatal(self.NaiveGK.current_state)

            if section(self.ball_position) in [LEFT_GOAL_AREA,RIGHT_GOAL_AREA]:
                if not on_attack_side(self.ball_position, self.team_side):
                    self.NaiveGK.normal_to_defend_ball() #defend
                else:
                    self.NaiveGK.normal_to_track_ball()#trackeia    
            else:
                self.NaiveGK.normal_to_track_ball()#trackeia

        if self.NaiveGK.is_defend_ball:
            #rospy.logfatal(self.NaiveGK.current_state)
            #return 0,0
            return self.in_defend_ball()
        else:
            #rospy.logfatal(self.NaiveGK.current_state)    
            #return 0,0
            return self.in_track_ball()

    def in_defend_ball(self):
        rospy.logfatal(self.NaiveGK.current_state)
        
        if section(self.ball_position) in [LEFT_GOAL_AREA,RIGHT_GOAL_AREA]:
           return self.push_ball()
        else:
            self.NaiveGK.defend_ball_to_track()#trackeia    
            return self.in_track_ball()#trackeia



    def push_ball(self):

        #if behind_ball(self.ball_position, self.position, self.team_side):
        if near_ball(self.ball_position, self.position):
            rospy.logfatal("SPIN")
            param1, param2, bool = self.movement.spin(255,spin_direction(self.ball_position, self.position, self.team_side))
            return param1, param2
        else:
            rospy.logfatal("MVTP")
            param_1, param_2 , _ = self.movement.move_to_point(
                speed = GOAL_KEEPER_SPEED,
                robot_position = self.position,
                robot_vector = [np.cos(self.orientation),np.sin(self.orientation)],
                goal_position = self.defend_position

            )
            return param_1, param_2


    def in_track_ball(self):
        rospy.logfatal(self.NaiveGK.current_state)

        if section(self.ball_position) in [LEFT_GOAL_AREA,RIGHT_GOAL_AREA]:
            if not on_attack_side(self.ball_position, self.team_side):
                self.NaiveGK.track_to_defend_ball()
                return self.in_defend_ball() #defend
            else:
                return self.follow_ball()    
        elif section(self.ball_position) in [LEFT_GOAL,RIGHT_GOAL]:
            rospy.logfatal("GOL!!")
            return 0,0
        else:
            return self.follow_ball()


    def follow_ball(self):

        self.defend_position[0] = 15.0 + 118.0*self.team_side
    
        if self.ball_position[1] >= 38.0 and self.ball_position[1] <= 92.0:
            rospy.logfatal("frente area")
            self.defend_position[1] = self.ball_position[1]
        else:
            if self.ball_position[1] > 92.0:
                rospy.logfatal("esq area")
                self.defend_position[1] = 92.0

            else: #self.defend_position[1] < 38:
                rospy.logfatal("dir area")
                self.defend_position[1] = 38.0

        param_1, param_2 , _ = self.movement.move_to_point(
            speed = GOAL_KEEPER_SPEED,
            robot_position = self.position,
            robot_vector = [np.cos(self.orientation),np.sin(self.orientation)],
            goal_position = self.defend_position

            )
        return param_1, param_2 


    def in_freeball_game(self):
        """
        Transitions in freeball state

        :return: int, int
        """

        rospy.logfatal(self.NaiveGK.current_state)

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
