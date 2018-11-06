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

path += '../parameters/bodies.json'

jsonHandler = JsonHandler()
bodies_unpack = jsonHandler.read(path, escape=True)



SOFTWARE = 0
HARDWARE = 1


GOALKEEPER_SPEED    = 70
MIN_X               = 5.0

MIN_Y               = 45.0
MAX_Y               = 85.0

GG_DIFF             = 140.0

SPIN_DIST           = 9.0

class NaiveGKController():

    def __init__(self, _robot_obj ,_robot_body="Nenhum", _debug_topic = None):
        self.pid_type = SOFTWARE
        self.robot = _robot_obj
        self.position = None
        self.orientation = None
        self.team_speed = None
        self.enemies_position = None
        self.enemies_speed = None
        self.ball_position = None
        self.team_side = None
        self.robot_body = _robot_body
        self.defend_position = np.array([0,0])

         #Attack_in right side
        self.attack_goal = np.array([150.0, 65.0])
         #Attack_in left side
        self.attack_goal = np.array([0.0, 65.0])


        self.pid_list = [bodies_unpack[self.robot_body]['KP'],
                         bodies_unpack[self.robot_body]['KI'],
                         bodies_unpack[self.robot_body]['KD']]


        self.stop = MyModel(state='stop')
        self.NaiveGK = NaiveGK(self.stop)
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
        :param robot: robot object
        """
        self.position = self.robot.position
        self.orientation = self.robot.orientation
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
        return 0, 0, 0

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

        if (near_ball(self.ball_position, self.position)):
            rospy.logfatal("SPIN")
            param1, param2, param3 = self.movement.spin(255, not spin_direction(self.ball_position, self.position, self.team_side))
            return param1, param2, param3
        else:
            rospy.logfatal("MVTP")


            param_1, param_2 , param3 = self.movement.move_to_point(
                GOALKEEPER_SPEED,
                self.position,
                [np.cos(self.orientation), np.sin(self.orientation)],
                self.ball_position
                )
            return param_1, param_2, param3


    def in_track_ball(self):
        #rospy.logfatal(self.NaiveGK.current_state)

        if section(self.ball_position) in [LEFT_GOAL_AREA,RIGHT_GOAL_AREA]:
            if not on_attack_side(self.ball_position, self.team_side):
                self.NaiveGK.track_to_defend_ball()
                return self.in_defend_ball() #defend
            else:
                return self.follow_ball()    
        elif section(self.ball_position) in [LEFT_GOAL,RIGHT_GOAL]:
            rospy.logfatal("GOL!!")
            return 0,0,0
        else:
            return self.follow_ball()


    def follow_ball(self):

        self.defend_position[0] = MIN_X + GG_DIFF*self.team_side
    
        if self.ball_position[1] >= MIN_Y and self.ball_position[1] <= MAX_Y:
            rospy.logfatal("frente area")
            self.defend_position[1] = self.ball_position[1]
        else:
            if self.ball_position[1] > MAX_Y:
                rospy.logfatal("esq area")
                self.defend_position[1] = MAX_Y

            else: #self.defend_position[1] < 38:
                rospy.logfatal("dir area")
                self.defend_position[1] = MIN_Y

        param_1, param_2 , param3 = self.movement.move_to_point(
            speed = GOALKEEPER_SPEED,
            robot_position = self.position,
            robot_vector = [np.cos(self.orientation),np.sin(self.orientation)],
            goal_position = self.defend_position

            )
        return param_1, param_2, param3


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
