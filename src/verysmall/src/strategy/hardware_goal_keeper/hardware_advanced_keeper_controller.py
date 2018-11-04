import sys
import os
import rospy
import numpy as np
from ..arena_sections import *
from ..ball_range import *
from hardware_advanced_keeper import Hardware_AdvancedGK, MyModel

sys.path[0] = path = root_path = os.environ['ROS_ARARA_ROOT'] + "src/robot/"
from movement.functions.movement import Movement
from utils.json_handler import JsonHandler
from utils.math_utils import angleBetween

path += '../parameters/bodies.json'

jsonHandler = JsonHandler()
bodies_unpack = jsonHandler.read(path, escape=True)

SOFTWARE = 0
HARDWARE = 1

DISTANCE = 8.0

GOALKEEPER_SPEED = 80
MIN_X = 10.0

MIN_Y = 45.0
MAX_Y = 85.0

MIN_Y_AREA = 30.0
MAX_Y_AREA = 100.0

GG_DIFF = 133.0

SPIN_DIST = 9.0

SPIN_SPEED = 255


class Hardware_AdvancedGKController():

    def __init__(self, _robot_body="Nenhum", _debug_topic=None):
        self.pid_type = HARDWARE
        self.position = [None, None]
        self.orientation = None
        self.speed = None
        self.team_speed = None
        self.enemies_position = None
        self.enemies_speed = None
        self.ball_position = [0,0]
        self.team_side = None
        self.robot_body = _robot_body
        self.defend_position = np.array([0, 0])
        self.last_state = None

        # Attack_in right side
        self.attack_goal = np.array([150.0, 65.0])
        # Attack_in left side
        self.attack_goal = np.array([0.0, 65.0])

        self.pid_list = [bodies_unpack[self.robot_body]['KP'],
                         bodies_unpack[self.robot_body]['KI'],
                         bodies_unpack[self.robot_body]['KD']]

        self.stop = MyModel(state='stop')
        self.AdvancedGK = Hardware_AdvancedGK(self.stop)
        self.movement = Movement(self.pid_list, error=7, attack_goal=self.attack_goal, _pid_type=self.pid_type,
                                 _debug_topic=_debug_topic)

    def set_pid_type(self, _type):
        """
        Change pid type
        :return:
        """
        self.pid_type = _type
        self.movement.set_pid_type(_type=self.pid_type)

    def update_game_information(self, position, orientation, speed, team_speed, enemies_position, enemies_speed,
                                ball_position, team_side):
        """
        Update game variables
        :param position:
        :param orientation:
        :param speed:
        :param team_speed:
        :param enemies_position:
        :param enemies_speed:
        :param ball_position:
        :param team_side:
        """

        if (position[0] != 0.0) and (position[1] != 0.0):
            self.position = position
        # else:
        #     rospy.logfatal("Deu ruim")

        if orientation != 0.0:
            self.orientation = orientation

        self.orientation = orientation
        self.speed = speed
        self.team_speed = team_speed
        self.enemies_position = enemies_position
        self.enemies_speed = enemies_speed
        self.ball_position = ball_position
        if (ball_position[0] != 0.0) and (ball_position[1] != 0.0):
            self.ball_position= ball_position
        self.team_side = team_side
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
        rospy.logfatal(self.AdvancedGK.current_state)

        """
        Transitions in normal game state

        :return: int, int
        """
        if self.AdvancedGK.is_stop:
            self.last_state = self.AdvancedGK.current_state
            self.AdvancedGK.stop_to_normal()

        return self.follow_ball()

        if self.AdvancedGK.is_normal:
            ball_section = section(self.ball_position)
            keeper_section = section(self.position)

            if keeper_section not in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA] or keeper_section in [LEFT_GOAL, RIGHT_GOAL]:
                self.last_state = self.AdvancedGK.current_state
                self.AdvancedGK.normal_to_out_of_area()

            elif ball_section in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA]:
                if not on_attack_side(self.ball_position, self.team_side):
                    self.last_state = self.AdvancedGK.current_state
                    self.AdvancedGK.normal_to_defend_ball()  # defend
                else:
                    self.last_state = self.AdvancedGK.current_state
                    self.AdvancedGK.normal_to_seek_ball()  # seekeia

            elif ball_section in [LEFT_GOAL, RIGHT_GOAL]:
                self.last_state = self.AdvancedGK.current_state
                self.AdvancedGK.normal_to_goal()

            else:
                self.last_state = self.AdvancedGK.current_state
                self.AdvancedGK.normal_to_seek_ball()  # trackeia

        if self.AdvancedGK.is_defend_ball:
            return self.in_defend_ball()
        elif self.AdvancedGK.is_seek_ball:
            return self.in_seek_ball()
        elif self.AdvancedGK.is_goal:
            return self.in_goal()
        else:
            return self.in_out_of_area()

    def in_defend_ball(self):
        rospy.logfatal(self.AdvancedGK.current_state)

        ball_section = section(self.ball_position)

        if section(self.position) not in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA]:
            self.last_state = self.AdvancedGK.current_state
            self.AdvancedGK.defend_ball_to_out_of_area()
            return self.in_out_of_area()

        elif ball_section in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA]:
            if not on_attack_side(self.ball_position, self.team_side):
                if near_ball(self.ball_position, self.position, DISTANCE):
                    self.last_state = self.AdvancedGK.current_state
                    self.AdvancedGK.defend_ball_to_spin()
                    return self.in_spin()
                else:
                    self.last_state = self.AdvancedGK.current_state
                    self.AdvancedGK.defend_ball_to_go_to_ball()
                    return self.in_go_to_ball()
            else:
                self.last_state = self.AdvancedGK.current_state
                self.AdvancedGK.defend_ball_to_seek_ball()
                return self.in_seek_ball()

        elif ball_section in [LEFT_GOAL, RIGHT_GOAL]:
            self.last_state = self.AdvancedGK.current_state
            self.AdvancedGK.defend_ball_to_goal()
            return self.in_goal()
        else:
            self.last_state = self.AdvancedGK.current_state
            self.AdvancedGK.defend_ball_to_seek_ball()  # trackeia
            return self.in_seek_ball()  # trackeia

    def in_seek_ball(self):
        rospy.logfatal(self.AdvancedGK.current_state)

        # rospy.logfatal(self.AdvancedGK.current_state)

        if section(self.ball_position) in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA]:
            if not on_attack_side(self.ball_position, self.team_side):
                self.last_state = self.AdvancedGK.current_state
                self.AdvancedGK.seek_ball_to_defend_ball()
                return self.in_defend_ball()  # defend
            else:
                return self.follow_ball()

        elif(( section(self.position) not in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA]) and ((self.team_side == LEFT and self.position[0] > 35.0 ) | (self.team_side == RIGHT and self.position < 115.0 )) ):
            self.last_state = self.AdvancedGK.current_state
            self.AdvancedGK.seek_ball_to_out_of_area()
            return self.in_out_of_area()

        else:
            return self.follow_ball()

    def in_spin(self):
        rospy.logfatal(self.AdvancedGK.current_state)
        self.last_state = self.AdvancedGK.current_state
        self.AdvancedGK.spin_to_defend_ball()

        param1, param2, param3 = self.movement.spin(SPIN_SPEED, not spin_direction(self.ball_position, self.position, self.team_side))


        return param1, param2, self.pid_type

    def in_go_to_ball(self):
        rospy.logfatal(self.AdvancedGK.current_state)
        self.last_state = self.AdvancedGK.current_state
        self.AdvancedGK.go_to_ball_to_defend_ball()

        param1, param2, param3 = self.movement.move_to_point(GOALKEEPER_SPEED,
                                           self.position,
                                           [np.cos(self.orientation), np.sin(self.orientation)],
                                           np.array([self.position[0], self.ball_position[1]]))

        return param1, param2, self.pid_type

    def follow_ball(self):

        self.defend_position[0] = MIN_X + GG_DIFF * self.team_side

        if self.ball_position[1] >= MIN_Y and self.ball_position[1] <= MAX_Y:
            self.defend_position[1] = self.ball_position[1]
        else:
            if self.ball_position[1] > MAX_Y:
                self.defend_position[1] = MAX_Y

            else:  # self.defend_position[1] < 38:
                self.defend_position[1] = MIN_Y

        param_1 = self.orientation - 1.5
        vector = (self.defend_position[1]-self.position[1])
        param_2 = 30

        if vector < 0:
            param_2 *= -1

        if abs(vector) < 10:
            param_2 = 0

        rospy.logfatal(self.orientation)
        rospy.logfatal(param_1)
        rospy.logfatal(param_2)

        return param_1, param_2, self.pid_type

    def in_goal(self):
        rospy.logfatal(self.AdvancedGK.current_state)

        # if section(self.ball_position) in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA]:
        if section(self.ball_position) not in [LEFT_GOAL, RIGHT_GOAL]:
            self.last_state = self.AdvancedGK.current_state
            self.AdvancedGK.goal_to_defend_ball()
        return 0, 0, 0

    def in_out_of_area(self):
        rospy.logfatal(self.AdvancedGK.current_state)
        # rospy.logfatal(self.position)

        goal_position = np.array([0, 0])
        goal_position[0] = MIN_X + GG_DIFF * self.team_side

        if self.position[1] >= MIN_Y_AREA and self.position[1] <= MAX_Y_AREA:
            # rospy.logfatal("frente area")
            goal_position[1] = self.position[1]
        else:
            if self.position[1] > MAX_Y_AREA:
                # rospy.logfatal("esq area")
                goal_position[1] = MAX_Y_AREA - 10

            else:  # self.defend_position[1] < 38:
                # rospy.logfatal("dir area")
                goal_position[1] = MIN_Y_AREA + 20

        param1, param2, param3 = self.movement.move_to_point(
            GOALKEEPER_SPEED,
            self.position,
            [np.cos(self.orientation), np.sin(self.orientation)],
            goal_position
        )

        if param3:
            self.last_state = self.AdvancedGK.current_state
            self.AdvancedGK.out_of_area_to_seek_ball()

        return param1, param2, self.pid_type

    def in_freeball_game(self):
        """
        Transitions in freeball state

        :return: int, int
        """

        rospy.logfatal(self.AdvancedGK.current_state)

        if self.AdvancedGK.is_stop:
            self.AdvancedGK.stop_to_freeball()

        if self.AdvancedGK.is_freeball:
            self.AdvancedGK.freeball_to_normal()

        if self.AdvancedGK.is_normal:
            return self.in_normal_game()

    def in_penalty_game(self):
        """
        Transitions in penalty state

        :return: int, int
        """
        if self.AdvancedGK.is_stop:
            self.AdvancedGK.stop_to_penalty()

        if self.AdvancedGK.is_penalty:
            self.AdvancedGK.penalty_to_normal()

        if self.AdvancedGK.is_normal:
            return self.in_normal_game()

    def in_meta_game(self):
        """
        Transitions in meta state

        :return: int, int
        """
        if self.AdvancedGK.is_stop:
            self.AdvancedGK.stop_to_meta()

        if self.AdvancedGK.is_meta:
            self.AdvancedGK.meta_to_normal()

        if self.AdvancedGK.is_normal:
            return self.in_normal_game()
