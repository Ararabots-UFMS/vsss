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
path += '../parameters/robots_pid.json'

jsonHandler = JsonHandler()
univector_list = jsonHandler.read(path)

KP = univector_list['robot_1']['KP']
KD = univector_list['robot_1']['KD']
KI = univector_list['robot_1']['KI']

CENTER_Y = 65
CENTER_X = 75
SPEED_DEFAULT = 180
MAX_X = 150

class NaiveGKController():

    def __init__(self):
        self.position = None
        self.orientation = None
        self.robot_speed = None
        self.enemies_position = None
        self.enemies_speed = None
        self.ball_position = None
        self.team_side = None

        self.stop = MyModel(state='Stop')
        self.RobotStateMachine = NaiveGK(self.stop)

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
        :param team_side:
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
        rospy.logfatal(self.RobotStateMachine.current_state) 
        if self.RobotStateMachine.is_stop:
            self.RobotStateMachine.stop_to_normal()

        if self.RobotStateMachine.is_normal:
            if on_attack_side(self.ball_position, self.team_side):

                if section(self.ball_position) in [UP_BORDER, DOWN_BORDER]:
                    self.RobotStateMachine.normal_to_border()
                    rospy.logfatal("to na borda")
                else:
                    rospy.logfatal("nao to na borda"  )
                    self.RobotStateMachine.normal_to_reach_ball()
            else:
                self.RobotStateMachine.normal_to_point()

        if self.RobotStateMachine.is_border:
            return self.in_border()
        elif self.RobotStateMachine.is_reach_ball:
            return self.in_reach_ball()
        elif self.RobotStateMachine.is_go_to_point:
            return self.in_point()
        elif self.RobotStateMachine.is_follow_ball:
            return self.in_follow_ball()
        elif self.RobotStateMachine.is_spin:
            return self.in_spin()
        elif self.RobotStateMachine.is_walk_border:
            return self.in_walk_border()


    def in_border(self):

        rospy.logfatal("entrei nessa porra"  )
        if not (on_attack_side(self.ball_position, self.team_side)):
            self.RobotStateMachine.border_to_point()
            return self.in_point()
        else:
            if(section(self.position) in [UP_BORDER]):
                rospy.logfatal("to na borda de cima"  )
                #turn the front side to face the ball
                left, right, _ = self.movement.head_to(
                robot_vector=([np.cos(self.orientation), np.sin(self.orientation)]),
                goal_vector=(np.cos(self.ball_position[0]), np.sin(self.ball_position[1])))
                self.RobotStateMachine.border_to_walk_border()
                return self.in_walk_border()
            else:
                self.RobotStateMachine.border_to_normal()
                return self.in_normal_game()


    def in_walk_border(self):

        if not (on_attack_side(self.ball_position, self.team_side)):
            self.RobotStateMachine.border_to_point()
            return self.in_point()

        left, right, _ = self.movement.do_univector(
            speed = SPEED_DEFAULT,
            robot_position=self.position,
            robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
            robot_speed=[0, 0],
            obstacle_position=np.resize(self.enemies_position, (5, 2)),
            obstacle_speed=[[0,0]]*5,
            ball_position=self.ball_position
        )
        self.RobotStateMachine.walk_border_to_normal()
        return left, right


    def in_point(self):

        wait_position_x = CENTER_X + ((-1)**self.team_side)*20
        position_center = [wait_position_x, CENTER_Y]

        if (self.position[0] == wait_position_x) and (self.position[1] == CENTER_Y):
            if on_attack_side(self.ball_position, self.team_side):
                self.RobotStateMachine.point_to_reach_ball()
            else:
                self.RobotStateMachine.point_to_follow_ball()
        else:
            if on_attack_side(self.ball_position, self.team_side):
                self.RobotStateMachine.point_to_reach_ball()
            else:
                left, right, _ = self.movement.do_univector(
                    speed = SPEED_DEFAULT,
                    robot_position=self.position,
                    robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
                    robot_speed=[0, 0],
                    obstacle_position=np.resize(self.enemies_position, (5, 2)),
                    obstacle_speed=[[0,0]]*5,
                    ball_position=position_center
                )

                return left, right

        if self.RobotStateMachine.is_reach_ball:
            return self.in_reach_ball()

        elif self.RobotStateMachine.is_follow_ball:
            return self.in_follow_ball()

    def in_follow_ball(self):

        if on_attack_side(self.ball_position, self.team_side):
            self.RobotStateMachine.follow_to_reach_ball()
            return self.in_reach_ball()
        elif (section(self.ball_position) in [LEFT_GOAL,RIGHT_GOAL]):
            self.RobotStateMachine.follow_ball_to_stop()
            return set_to_stop_game()
        return 0,0
    def in_spin(self):

        self.RobotStateMachine.spin_to_normal()

        if(spin_direction(self.ball_position, self.position, self.team_side) == CCW):
            left, right, _ = self.movement.spin(255, CCW) 
            return left, right
        else:
            left, right, _ = self.movement.spin(255, CW) 
            return left, right

    def in_reach_ball(self):
        """
        Transitions in the reach ball state

        :return: int, int
        """
        if not on_attack_side(self.ball_position, self.team_side):
            self.RobotStateMachine.reach_ball_to_point()
            return self.in_point()

        if(section(self.position) in [UP_BORDER, DOWN_BORDER,
            LEFT_DOWN_BOTTOM_LINE, LEFT_UP_BOTTOM_LINE, RIGHT_DOWN_BOTTOM_LINE,
            RIGHT_UP_BOTTOM_LINE]):
            self.RobotStateMachine.reach_ball_to_border()
            return self.in_border()

        if (behind_ball(self.ball_position, self.position, self.team_side)):
            self.RobotStateMachine.reach_ball_to_spin()
            return self.in_spin()
        else:

            left, right, _ = self.movement.do_univector(
                speed = SPEED_DEFAULT,
                robot_position=self.position,
                robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
                robot_speed=[0, 0],
                obstacle_position=np.resize(self.enemies_position, (5, 2)),
                obstacle_speed=[[0,0]]*5,
                ball_position=self.ball_position
            )

            return left, right

    def in_freeball_game(self):
        """
        Transitions in freeball state

        :return: int, int
        """
        if self.RobotStateMachine.is_stop:
            self.RobotStateMachine.stop_to_freeball()

        if self.RobotStateMachine.is_freeball:
            self.RobotStateMachine.freeball_to_normal()

        if self.RobotStateMachine.is_normal:
            return self.in_normal_game()

    def in_penalty_game(self):
        """
        Transitions in penalty state

        :return: int, int
        """
        if self.RobotStateMachine.is_stop:
            self.RobotStateMachine.stop_to_penalty()

        if self.RobotStateMachine.is_penalty:
            self.RobotStateMachine.penalty_to_normal()

        if self.RobotStateMachine.is_normal:
            return self.in_normal_game()

    def in_meta_game(self):
        """
        Transitions in meta state

        :return: int, int
        """
        if self.RobotStateMachine.is_stop:
            self.RobotStateMachine.stop_to_meta()

        if self.RobotStateMachine.is_meta:
            self.RobotStateMachine.meta_to_normal()

        if self.RobotStateMachine.is_normal:
            return self.in_normal_game()
