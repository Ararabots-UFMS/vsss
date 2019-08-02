import sys
import os
import rospy
import numpy as np
from .zagueiro import Zagueiro, MyModel
from math import pi
from robot_module.movement.functions.movement import Movement
from utils.math_utils import angle_between, distancePoints
from utils.json_handler import JsonHandler
from strategy.strategy_utils import *

bodies_unpack = JsonHandler().read("parameters/bodies.json", escape=True)

SOFTWARE = 0
HARDWARE = 1

ZAGUEIRO_SPEED = 70
DEF_X_POS = [75.0/2.0, 75 + 75.0/2.0]
ENEMY_POS = np.array([[[7.5,38.0],[7.5, 51.0], [7.5, 64], [7.5, 77], [7.5, 92.0]],[[142.5,38.0],[142.5, 51.0], [142.5, 64], [142.5, 77], [142.5, 92.0]]])
BOTTOM_LEFT_LINE_POS = [[5.0, 20.0],[5.0, 105.0]]
BOTTOM_RIGHT_LINE_POS = [[145.0, 20.0],[145.0, 105.0]]


class ZagueiroController():

    def __init__(self, _robot_obj, _robot_body="Nenhum", _debug_topic = None):
        self.pid_type = SOFTWARE
        self.robot = _robot_obj
        #self.robot.get_stuck = self.robot.get_fake_stuck
        self.position = None
        self.orientation = None
        self.team_speed = None
        self.enemies_position = None
        self.enemies_speed = None
        self.ball_position = None
        self.team_side = None
        self.speed = None
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

        self.position_buffer = []

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
        self.get_stuck = self.robot.get_stuck
        self.speed = self.robot.speed
        self.robot.add_to_buffer(self.position_buffer, 60, self.robot.position)
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

    def speed_correction(self):
        speed = ZAGUEIRO_SPEED
        if distancePoints(self.position, self.defend_position) < 1.5 * self.movement.error_margin:
            speed = ZAGUEIRO_SPEED / 2.0
        return speed

    def in_normal_game(self):
        # rospy.logfatal(str(self.position))
        rospy.logfatal(self.zagueiro.current_state)

        """
        Transitions in normal game state

        :return: int, int
        """
        if self.zagueiro.is_stop:
            self.zagueiro.stop_to_normal()

        # verify if the robot is on defense area


        if self.zagueiro.is_normal:
            if extended_area(self.position, self.team_side)in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA, LEFT_GOAL, RIGHT_GOAL]:
                self.zagueiro.normal_to_area()
                return self.in_area()
            #verify if the robot is stuck on border
            if self.get_stuck(self.position):
                rospy.logfatal("if do stuck")
                self.zagueiro.normal_to_stuck()
                return self.in_stuck()

            if(self.team_side == LEFT):
                if((section(self.ball_position) not in [LEFT_GOAL, LEFT_GOAL_AREA]) and self.ball_position[0] <= 75.0):
                    s = section(self.ball_position)
                    if s in range(LEFT_UP_CORNER,DOWN_BORDER+1) or s in range(LEFT_DOWN_BOTTOM_LINE, RIGHT_UP_BOTTOM_LINE+1):
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
        elif self.zagueiro.is_stuck:
            return self.in_stuck()
        else:
            rospy.logfatal("aqui deu ruim, hein moreno")
            return 0, 0, self.pid_type


    def in_defend(self):
        rospy.logfatal(self.zagueiro.current_state)
        #verify if the robot is stuck on border
        if self.get_stuck(self.position):
            rospy.logfatal("if do stuck")
            self.zagueiro.defend_to_stuck()
            return self.in_stuck()

        #verify if the robot is in the area
        if extended_area(self.position, self.team_side) in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA, LEFT_GOAL, RIGHT_GOAL]:
            self.zagueiro.defend_to_area()
            return self.in_area()

        # se a bola esta dentro da area ou do gol, vou para o wait ball trackear a bola
        if(not ((self.team_side == LEFT) and (section(self.ball_position) not in [LEFT_GOAL, LEFT_GOAL_AREA]) and self.ball_position[0] <= 75.0 or (self.team_side == RIGHT) and(section(self.ball_position) not in [RIGHT_GOAL, RIGHT_GOAL_AREA]) and self.ball_position[0] > 75.0)):
            self.zagueiro.defend_to_wait_ball()
            return self.in_wait_ball()
        else:
            sb = section(self.ball_position)
            if sb in range(LEFT_UP_CORNER,DOWN_BORDER+1) or sb in range(LEFT_DOWN_BOTTOM_LINE, RIGHT_UP_BOTTOM_LINE+1):
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

        # Se eu estiver struck, saio do struck com in_struck()
        if self.get_stuck(self.position):
            rospy.logfatal("if do stuck")
            self.zagueiro.move_to_stuck()
            return self.in_stuck()

        # Se nao to stuck posso estar na area e sai correndo!
        if extended_area(self.position, self.team_side)in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA, LEFT_GOAL, RIGHT_GOAL]:
            self.zagueiro.move_to_area()
            return self.in_area()

        # Se a bola esta somente no ataque eu sigo a posicao y dela
        if self.team_side == LEFT:
            if self.ball_position[0] > 75:
                self.zagueiro.move_to_wait_ball()
                return self.in_wait_ball()
        else:
            if self.ball_position[0] <= 75:
                self.zagueiro.move_to_wait_ball()
                return self.in_wait_ball()

        # Se eu estiver perto da bola, spin na danada!
        if near_ball(self.position, self.ball_position, 7.5):
            self.zagueiro.move_to_do_spin()
            return self.in_spin()

        # Se n estou na area e nem stuck e nem perto da bola monto os inimigos
        # imaginarios para seguir o univector e possivelmente nunca entrar na area
        enemies_position = list(self.enemies_position)
        enemies_speed = list(self.enemies_speed)
        for i in range(ENEMY_POS[self.team_side].shape[0]):
            enemies_position.append(ENEMY_POS[self.team_side][i])
            enemies_speed.append(np.array([0,0]))

        self.enemies_position = np.asarray(enemies_position)
        self.enemies_speed = np.asarray(enemies_speed)

        param1, param2, param3 = self.movement.do_univector(
            speed=ZAGUEIRO_SPEED,
            robot_position=self.position,
            robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
            # robot_speed=self.speed,
            robot_speed=np.array([0,0]),
            obstacle_position=self.enemies_position,
            obstacle_speed=self.enemies_speed,
            ball_position=self.ball_position)
        return param1, param2, self.pid_type

    def in_wait_ball(self):
        rospy.logfatal(self.zagueiro.current_state)

        if extended_area(self.position, self.team_side)in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA, LEFT_GOAL, RIGHT_GOAL]:
            self.zagueiro.wait_ball_to_area()
            return self.in_area()

        #verify if the robot is stuck on border
        if self.get_stuck(self.position):
            rospy.logfatal("if do stuck")
            self.zagueiro.wait_ball_to_stuck()
            return self.in_stuck()

        # bola na defesa e nao esta na area
        if((self.team_side == LEFT) and (section(self.ball_position) not in [LEFT_GOAL, LEFT_GOAL_AREA]) and self.ball_position[0] <= 75.0 or (self.team_side == RIGHT) and(section(self.ball_position) not in [RIGHT_GOAL, RIGHT_GOAL_AREA]) and self.ball_position[0] > 75.0):
            self.zagueiro.wait_ball_to_defend()
            return self.in_defend()
        else:
            self.defend_position[0] = DEF_X_POS[self.team_side]
            self.defend_position[1] = self.ball_position[1]

            speed =ZAGUEIRO_SPEED
            self.movement.error_margin = 5.0
            speed = self.speed_correction()

            param1, param2, param3 = self.movement.move_to_point(
                speed,
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
        robot_vector = [np.cos(self.orientation), np.sin(self.orientation)]
        if(near_ball(self.ball_position, self.position)):
            self.zagueiro.border_to_do_spin()
            return self.in_spin()

        if extended_area(self.position, self.team_side)in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA, LEFT_GOAL, RIGHT_GOAL]:
            self.zagueiro.border_to_area()
            return self.in_area()

        if self.get_stuck(self.position):
            rospy.logfatal("if do stuck")
            self.zagueiro.border_to_stuck()
            return self.in_stuck()

        sb = section(self.ball_position)

        #estrategia para ir para a ponta da area se a bola estiver no fundo, atualmente nao esta convergindo
        # if self.ball_position[0] < 75.0 and self.team_side == LEFT:
        #     if sb in [LEFT_UP_CORNER, LEFT_DOWN_CORNER, LEFT_UP_BOTTOM_LINE, LEFT_DOWN_BOTTOM_LINE]:
        #         if self.ball_position[1] < 65:
        #             point = BOTTOM_LEFT_LINE_POS[0]
        #         else:
        #             point = BOTTOM_LEFT_LINE_POS[1]
        #         speed = self.speed_correction()
        #         param1, param2, param3 = self.movement.move_to_point(speed, self.position, robot_vector, point)
        #         if not param3:
        #             return param1, param2, self.pid_type
        #
        # elif self.ball_position[0] >= 75 and self.team_side == RIGHT:
        #     if sb in [RIGHT_UP_CORNER, RIGHT_DOWN_CORNER, RIGHT_UP_BOTTOM_LINE, RIGHT_DOWN_BOTTOM_LINE]:
        #         if self.ball_position[1] < 65:
        #             point = BOTTOM_RIGHT_LINE_POS[0]
        #         else:
        #             point = BOTTOM_RIGHT_LINE_POS[1]
        #         speed = self.speed_correction()
        #         param1, param2, param3 = self.movement.move_to_point(speed, self.position, robot_vector, point)
        #         if not param3:
        #             return param1, param2, self.pid_type

        if self.ball_position[0] < 75 and self.team_side == LEFT or self.ball_position[0] >=75 and self.team_side == RIGHT:
            if sb in [UP_BORDER, DOWN_BORDER] or sb in [LEFT_UP_CORNER, LEFT_DOWN_CORNER, LEFT_UP_BOTTOM_LINE, LEFT_DOWN_BOTTOM_LINE] or sb in [RIGHT_UP_CORNER, RIGHT_DOWN_CORNER, RIGHT_UP_BOTTOM_LINE, RIGHT_DOWN_BOTTOM_LINE]:
                if self.position[1] < 65 and self.ball_position[1] >=65 or self.position[1]> 65 and self.ball_position[1] <= 65:
                    # Se n estou na area e nem stuck e nem perto da bola monto os inimigos
                    # imaginarios para seguir o univector e possivelmente nunca entrar na area
                    enemies_position = list(self.enemies_position)
                    enemies_speed = list(self.enemies_speed)
                    for i in range(ENEMY_POS[self.team_side].shape[0]):
                        enemies_position.append(ENEMY_POS[self.team_side][i])
                        enemies_speed.append(np.array([0,0]))

                    self.enemies_position = np.asarray(enemies_position)
                    self.enemies_speed = np.asarray(enemies_speed)

                    param1, param2, param3 = self.movement.do_univector(
                        speed=ZAGUEIRO_SPEED,
                        robot_position=self.position,
                        robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
                        # robot_speed=self.speed,
                        robot_speed=np.array([0,0]),
                        obstacle_position=self.enemies_position,
                        obstacle_speed=self.enemies_speed,
                        ball_position=self.ball_position)
                    return param1, param2, self.pid_type
                else:
                    speed = self.speed_correction()
                    param1, param2, param3 = self.movement.move_to_point(speed, self.position, robot_vector, self.ball_position)
                if not param3:
                    return param1, param2, self.pid_type
            else:
                self.zagueiro.border_to_defend()
        return 0, 0, self.pid_type

    def in_area(self):
        robot_vector = [np.cos(self.orientation), np.sin(self.orientation)]
        speed = self.speed_correction()
        param1, param2, param3 = self.movement.move_to_point(speed, self.position, robot_vector, self.defend_position)
        if not param3:
            if (self.team_side == LEFT and self.position[0] < DEF_X_POS[self.team_side]):
                return param1, param2, self.pid_type
            elif (self.team_side == RIGHT and self.position[0] > DEF_X_POS[self.team_side]):
                return param1, param2, self.pid_type
        self.zagueiro.area_to_normal()
        return 0, 0, self.pid_type

    def in_stuck(self):
        # rospy.logfatal(self.zagueiro.current_state)
        if near_ball(self.ball_position, self.team_side):
            if section(self.ball_position) == UP_BORDER or DOWN_BORDER:
                if self.team_side == LEFT:
                    if self.ball_position[0] > self.position[0]:
                        self.stuck_to_spin()
                        return self.in_spin()
                if self.team_side == RIGHT:
                    if self.ball_position[0] < self.position[0]:
                        self.stuck_to_spin()
                        return self.in_spin()


        if self.get_stuck(self.position) and section(self.position) != CENTER:

            # robot_vector = [np.cos(self.orientation), np.sin(self.orientation)]
            # goal_vector = np.array(np.array([75,65]) - self.position)
            # param1, param2, param3 = self.movement.head_to(robot_vector, goal_vector)
            #
            # #verify if the robot is aligned with the ball
            # if param3:
            #     #go to the defense routine
            #     self.zagueiro.stuck_to_defend()
            #     return param1, param2, self.pid_type
            # else:
            #     #keep turning
            #     rospy.logfatal(str(param1)+"  "+str(param2))
            #     return param1, param2, self.pid_type

            # if angleBetween([self.position[0], 150], self.position) < pi/2 + error
            rospy.logfatal("if do in_stuck")
            sr = section(self.position)
            if sr == UP_BORDER:
                point = [self.position[0], self.position[1]-10]
            elif sr == DOWN_BORDER:
                point = [self.position[0], self.position[1]+10]
            elif sr in [LEFT_UP_BOTTOM_LINE, LEFT_DOWN_BOTTOM_LINE]:
                point = [self.position[0]+10, self.position[1]]
            elif sr in [RIGHT_UP_BOTTOM_LINE, RIGHT_DOWN_BOTTOM_LINE]:
                point = [self.position[0]-10, self.position[1]]
            else:
                point = [65,75]
            speed = self.speed_correction()
            param1, param2, param3 = self.movement.move_to_point(
                speed,
                self.position,
                [np.cos(self.orientation), np.sin(self.orientation)],
                point
            )
            return param1, param2, self.pid_type
        else:
            self.zagueiro.stuck_to_defend()
            return 0,0, self.pid_type
