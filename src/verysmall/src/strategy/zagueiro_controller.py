import sys
import os
import rospy
import numpy as np
from zagueiro import Zagueiro, MyModel
sys.path[0] = path = root_path = os.environ['ROS_ARARA_ROOT']+"src/robot/"
from math import pi
from movement.functions.movement import Movement
from utils.math_utils import angleBetween
from utils.json_handler import JsonHandler
from utils.math_utils import angleBetween
path += '../parameters/bodies.json'
from arena_sections import *
from ball_range import *

jsonHandler = JsonHandler()
bodies_unpack = jsonHandler.read(path, escape=True)

SOFTWARE = 0
HARDWARE = 1

ZAGUEIRO_SPEED = 140
DEF_X_POS = [75.0/2.0, 75 + 75.0/2.0]

def stuck_border(robot_position):
    return False

class ZagueiroController():

    def __init__(self, _robot_obj, _robot_body="Nenhum", _debug_topic = None):
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
        rospy.logfatal(self.robot_body)
        self.pid_list = [bodies_unpack[self.robot_body]['KP'],
                         bodies_unpack[self.robot_body]['KI'],
                         bodies_unpack[self.robot_body]['KD']]

        # if(not self.team_side):
        #     #Attack_in right side
        #     self.attack_goal = np.array([150.0, 65.0])
        # else:
        #     #Attack_in left side
        #     self.attack_goal = np.array([0.0, 65.0])

        #univector without rotation
        self.attack_goal = not self.team_side

        self.defend_position = np.array([0,0])

        self.stop = MyModel(state='stop')
        self.zagueiro = Zagueiro(self.stop)

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
        :param position:
        :param orientation:
        :param team_speed:
        :param enemies_position:
        :param enemies_speed:
        :param ball_position:
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
        return 0, 0, SOFTWARE

    def in_normal_game(self):
        rospy.logfatal(self.zagueiro.current_state)

        """
        Transitions in normal game state

        :return: int, int
        """
        if self.zagueiro.is_stop:
            self.zagueiro.stop_to_normal()

        # verify if the robot is on defense area
        if section(self.position) in [LEFT_GOAL, LEFT_GOAL_AREA] or section(self.position) in [RIGHT_GOAL, RIGHT_GOAL_AREA]:
            self.zagueiro.normal_to_area()

        if self.zagueiro.is_normal:
            #verify if the robot is stuck on border
            if stuck_border(self.position):
                self.zagueiro.normal_to_stuck()
                return self.in_stuck()

            if(self.team_side == LEFT):
                if((section(self.ball_position) not in [LEFT_GOAL, LEFT_GOAL_AREA]) and self.ball_position[0] <= 75.0):
                    s = section(self.ball_position)
                    if s in xrange(LEFT_UP_CORNER,DOWN_BORDER+1) or s in xrange(LEFT_DOWN_BOTTOM_LINE, RIGHT_UP_BOTTOM_LINE+1):
                        self.zagueiro.normal_to_border()
                    else:
                        self.zagueiro.normal_to_defend()
                else:
                     self.zagueiro.normal_to_wait_ball()

            else: # TEAM_SIDE == RIGHT
                if((section(self.ball_position) not in [RIGHT_GOAL, RIGHT_GOAL_AREA]) and self.ball_position[0] > 75.0):
                    if(near_ball(self.ball_position, self.position)):
                        self.zagueiro.normal_to_do_spin()
                    else:
                        self.zagueiro.normal_to_defend()
                else:
                     self.zagueiro.normal_to_wait_ball()

        if self.zagueiro.is_area:
            return self.in_area()
        elif self.zagueiro.is_defend:
            return self.in_defend()
        elif (self.zagueiro.is_wait_ball):
            return self.in_wait_ball()
        elif self.zagueiro.is_do_spin :
            return self.in_spin()
        elif self.zagueiro.is_move:
            return self.in_move()
        elif self.zagueiro.is_border:
            return self.in_border()
        else:
            rospy.logfatal("aqui deu ruim, hein moreno")
            return 0, 0, self.pid_type


    def in_defend(self):
        rospy.logfatal(self.zagueiro.current_state)
        #verify if the robot is stuck on border
        if stuck_border(self.position):
            self.zagueiro.normal_to_stuck()
            return self.in_stuck()

        #verify if the robot is in the area
        if section(self.position) in [LEFT_GOAL, LEFT_GOAL_AREA] or section(self.position) in [RIGHT_GOAL, RIGHT_GOAL_AREA]:
            self.zagueiro.defend_to_area()
            return self.in_area()

        if(not ((self.team_side == LEFT) and (section(self.ball_position) not in [LEFT_GOAL, LEFT_GOAL_AREA]) and self.ball_position[0] <= 75.0 or (self.team_side == RIGHT) and(section(self.ball_position) not in [RIGHT_GOAL, RIGHT_GOAL_AREA]) and self.ball_position[0] > 75.0)):
            self.zagueiro.defend_to_wait_ball()
            return self.in_wait_ball()
        else:
            sb = section(self.ball_position)
            if sb in xrange(LEFT_UP_CORNER,DOWN_BORDER+1) or sb in xrange(LEFT_DOWN_BOTTOM_LINE, RIGHT_UP_BOTTOM_LINE+1):
                self.zagueiro.defend_to_border()
                return self.in_border()
            if(near_ball(self.ball_position, self.position)):
                self.zagueiro.defend_to_do_spin()
                return self.in_spin()
            else:
                self.zagueiro.defend_to_move()
                return self.in_move()


    def in_spin(self):
        rospy.logfatal(self.zagueiro.current_state)
        self.zagueiro.do_spin_to_defend()
        if self.team_side == LEFT:
            if self.ball_position[1] < 65:
                ccw = True
            else:
                ccw = False
        else:
            if self.ball_position[1] < 65:
                ccw = False
            else:
                ccw = True


        param1, param2, param3 = self.movement.spin(ZAGUEIRO_SPEED, ccw)

        return param1, param2, self.pid_type

    def in_move(self):
        rospy.logfatal(self.zagueiro.current_state)
        if section(self.position) in [LEFT_GOAL, LEFT_GOAL_AREA] or section(self.position) in [RIGHT_GOAL, RIGHT_GOAL_AREA]:
            self.zagueiro.move_to_area()
            return self.in_area()

        #verify if the robot is stuck on border
        if stuck_border(self.position):
            self.zagueiro.normal_to_stuck()
            return self.in_stuck()

        # self.zagueiro.move_to_move()
        rospy.logfatal("A POSICAO DA BOLA EH: "+str(self.ball_position))
        param1, param2, param3 = self.movement.do_univector(
            speed=ZAGUEIRO_SPEED,
            robot_position=self.position,
            robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
            robot_speed=np.array([0, 0]),
            obstacle_position=self.enemies_position,
            obstacle_speed=[[0,0]]*5,
            ball_position=self.ball_position,
            only_forward=False)
        if near_ball(self.position, self.ball_position, 7.5):
            self.zagueiro.move_to_do_spin()
            return self.in_spin()
        return param1, param2, self.pid_type

    def in_wait_ball(self):
        rospy.logfatal(self.zagueiro.current_state)
        if section(self.position) in [LEFT_GOAL, LEFT_GOAL_AREA] or section(self.position) in [RIGHT_GOAL, RIGHT_GOAL_AREA]:
            self.zagueiro.wait_ball_to_area()
            return self.in_area()

        if((self.team_side == LEFT) and (section(self.ball_position) not in [LEFT_GOAL, LEFT_GOAL_AREA]) and self.ball_position[0] <= 75.0 or (self.team_side == RIGHT) and(section(self.ball_position) not in [RIGHT_GOAL, RIGHT_GOAL_AREA]) and self.ball_position[0] > 75.0):
            self.zagueiro.wait_ball_to_defend()
            return self.in_defend()
        else:
            self.defend_position[0] = DEF_X_POS[self.team_side]
            self.defend_position[1] = self.ball_position[1]

            param1, param2, param3 = self.movement.move_to_point(
                ZAGUEIRO_SPEED,
                self.position,
                [np.cos(self.orientation), np.sin(self.orientation)],
                self.defend_position
            )

            return param1, param2, self.pid_type


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

    def in_border(self):
        rospy.logfatal(self.zagueiro.current_state)
        if section(self.position) in [LEFT_GOAL, LEFT_GOAL_AREA] or section(self.position) in [RIGHT_GOAL, RIGHT_GOAL_AREA]:
            self.zagueiro.border_to_area()
            return self.in_area()
        sb = section(self.ball_position)
        if sb in xrange(LEFT_UP_CORNER,DOWN_BORDER+1) or sb in xrange(LEFT_DOWN_BOTTOM_LINE, RIGHT_UP_BOTTOM_LINE+1):
            rospy.logfatal("to na borda")
            robot_vector = [np.cos(self.orientation), np.sin(self.orientation)]
            if(near_ball(self.ball_position, self.position, 7.5)):
                self.zagueiro.border_to_do_spin()
                return self.in_spin()
            else:
                param1, param2, param3 = self.movement.move_to_point(ZAGUEIRO_SPEED, self.position, robot_vector, self.ball_position)
                rospy.logfatal(param3)
                return param1, param2, self.pid_type
        else:
            self.zagueiro.border_to_normal()
            return self.in_normal_game()

    def in_area(self):
        rospy.logfatal(self.zagueiro.current_state)
        if section(self.position) in [LEFT_GOAL, LEFT_GOAL_AREA] or section(self.position) in [RIGHT_GOAL, RIGHT_GOAL_AREA]:
            robot_vector = [np.cos(self.orientation), np.sin(self.orientation)]
            param1, param2, param3 = self.movement.move_to_point(ZAGUEIRO_SPEED, self.position, robot_vector, self.defend_position)
            return param1, param2, self.pid_type
        else:
            self.zagueiro.area_to_normal()
            return 0, 0, self.pid_type

    def in_stuck(self):
        rospy.logfatal(self.zagueiro.current_state)
        robot_vector = [np.cos(self.orientation), np.sin(self.orientation)]
        goal_vector = np.array(self.ball_position - self.position)
        param1, param2, param3 = self.movement.head_to(robot_vector, goal_vector)

        #verify if the robot is aligned with the ball
        if param3:
            #go to the defense routine
            self.zagueiro.stuck_to_defend()
            return param1, param2, self.pid_type
        else:
            #keep turning
            return param1, param2, self.pid_type

        # def head_to(self, robot_vector, goal_vector):
