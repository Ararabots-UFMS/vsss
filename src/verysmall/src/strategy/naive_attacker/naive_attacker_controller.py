import sys
import os
import rospy
import math
import numpy as np
from strategy.arena_sections import *
from strategy.ball_range import *
from naive_attacker_strategy import NaiveAttacker, MyModel
sys.path[0] = path = root_path = os.environ['ROS_ARARA_ROOT']+"src/robot/"
from movement.functions.movement import Movement
from utils.json_handler import JsonHandler
path += '../parameters/robots_pid.json'

jsonHandler = JsonHandler()
univector_list = jsonHandler.read(path)

KP = univector_list['robot_5']['KP']
KD = univector_list['robot_5']['KD']
KI = univector_list['robot_5']['KI']

CENTER_Y = 65
CENTER_X = 75
SPEED_DEFAULT = 180
MAX_X = 150

class NaiveAttackerController():

    def __init__(self):
        self.position = None
        self.orientation = None
        self.robot_speed = None
        self.enemies_position = None
        self.enemies_speed = None
        self.ball_position = None
        self.team_side = None
        self.borders = [UP_BORDER, DOWN_BORDER, LEFT_DOWN_BOTTOM_LINE, LEFT_UP_BOTTOM_LINE, RIGHT_DOWN_BOTTOM_LINE, RIGHT_UP_BOTTOM_LINE]


        self.stop = MyModel(state='Stop')
        self.RobotStateMachine = NaiveAttacker(self.stop)

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
            return self.in_pid()
            # # Caso a bola esteja no campo de ataque
            # if on_attack_side(self.ball_position, self.team_side):
            #     # Verifica se a bola esta nas bordas
            #     if (section(self.position) in self.borders):
            #         self.RobotStateMachine.normal_to_border() 
            #     else:
            #         self.RobotStateMachine.normal_to_reach_ball()
            # # Caso o a bola esteja na defesa manda o atacante para um ponto fixo
            # else:
            #     self.RobotStateMachine.normal_to_point()

        # Se estado esta como borda
        if self.RobotStateMachine.is_border:
            return self.in_border()

        # Caso o estado estaja normal
        elif self.RobotStateMachine.is_reach_ball:
            return self.in_reach_ball()

        # Caso a bola esteja no campo de defesa
        elif self.RobotStateMachine.is_go_to_point:
            return self.in_point()

        # Estado de espera da bola
        elif self.RobotStateMachine.is_wait_ball:
            return self.in_wait_ball()

        # Estado de chute
        elif self.RobotStateMachine.is_spin:
            return self.in_spin()

        # Estado de corrida do robo na borda
        elif self.RobotStateMachine.is_walk_border:
            return self.in_walk_border()


    def in_pid(self):

        rospy.logfatal(self.position)

        point = [140,25-(7.5/2.0)]

        left, right, done = self.movement.move_to_point(
            speed = 150,
            robot_position=self.position,
            robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
            goal_position = point
        )
        # left, right, done = self.movement.spin(255, 1)

        if (distance_point(self.position, point) < 20):
            return 0, 0

        return left, right


    def in_border(self):

        # Caso a bola esteja na defesa, manda o robo para o estado de espera
        if not (on_attack_side(self.ball_position, self.team_side)):
            self.RobotStateMachine.border_to_point()
            return self.in_point()
        else:
            # Caso a bola ainda esteja na borda
            if(section(self.position) in self.borders):
                
                robot_vector = [np.cos(self.orientation), np.sin(self.orientation)]
                goal_vector  = [self.ball_position[0]-self.position[0], self.ball_position[1]-self.position[1]]

                #turn the front side to face the ball
                left, right, done = self.movement.head_to(
                    robot_vector=(robot_vector),
                    goal_vector=(goal_vector)
                )
                
                # Caso vetor de direcao do robo esteja apontada para a bola 
                if done:
                    self.RobotStateMachine.border_to_walk_border()
                
                return left, right
                # return self.in_walk_border()


            # Caso a bola nao esteja na borda
            else:
                self.RobotStateMachine.border_to_reach_ball()
                return self.in_reach_ball()


    def in_walk_border(self):

        # Caso a bola nao esteja na defesa
        if not (on_attack_side(self.ball_position, self.team_side)):
            self.RobotStateMachine.walk_border_to_point()
            return self.in_point()

        

        # Segue a bola com o univector
        left, right, _ = self.movement.move_to_point(
            speed = SPEED_DEFAULT,
            robot_position=self.position,
            robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
            goal_position = self.ball_position
        )

        # Caso a bola nao esteja na borda, troca para o estado inicial
        if not (section(self.ball_position) in self.borders):
            self.RobotStateMachine.walk_border_to_normal()
        
        return left, right


    def in_point(self):

        # Posicao de espera do robo
        wait_position_x = CENTER_X + ((-1)**self.team_side)*20
        position_center = [wait_position_x, CENTER_Y]

        # Caso a bola estja no ataque muda o estado para o reach ball
        if on_attack_side(self.ball_position, self.team_side):
            self.RobotStateMachine.point_to_reach_ball()

        # Verifica se a posicao atual do robo esta em uma margem de erro aceitavel
        if (distance_point(self.position, position_center) < 10):
            if not on_attack_side(self.ball_position, self.team_side):
                self.RobotStateMachine.point_to_wait_ball()
        else:
            if not on_attack_side(self.ball_position, self.team_side):
                # Manda o robo para a posicao de espera com o univector

                left, right, done = self.movement.move_to_point(
                    speed = SPEED_DEFAULT,
                    robot_position=self.position,
                    robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
                    goal_position = position_center
                )
                
                return left, right

        if self.RobotStateMachine.is_reach_ball:
            return self.in_reach_ball()

        elif self.RobotStateMachine.is_wait_ball:
            return self.in_wait_ball()

    def in_wait_ball(self):

        # Caso a bola passe para o ataque, troca de espado para ataque
        if on_attack_side(self.ball_position, self.team_side):
            self.RobotStateMachine.wait_to_reach_ball()
            return self.in_reach_ball()

        # Verifica se houve gol de qualquer equipe
        elif (section(self.ball_position) in [LEFT_GOAL,RIGHT_GOAL]):
            self.RobotStateMachine.wait_ball_to_stop()
            return set_to_stop_game()

        # Caso contrario, apenas espera com o robo parado
        robot_vector = [np.cos(self.orientation), np.sin(self.orientation)]
        goal_vector  = [self.ball_position[0]-self.position[0], self.ball_position[1]-self.position[1]]

        #turn the front side to face the ball
        left, right, done = self.movement.head_to(
            robot_vector=(robot_vector),
            goal_vector=(goal_vector)
        )
        return left, right


    def in_spin(self):

        # Caso a bola nao estaja no range de chute persegue a bola
        if not (behind_ball(self.ball_position, self.position, self.team_side)):
            self.RobotStateMachine.spin_to_reach_ball()

        if self.RobotStateMachine.is_reach_ball:
            return self.in_reach_ball()

        # Pega o lado que o robo tem que girar
        spin_side = spin_direction(self.ball_position, self.position, self.team_side)  

        # Chama a funcao de spin
        left, right, _ = self.movement.spin(255, not spin_side) 
        return left, right

    def in_reach_ball(self):
        """
        Transitions in the reach ball state

        :return: int, int
        """
        if not on_attack_side(self.ball_position, self.team_side):
            self.RobotStateMachine.reach_ball_to_point()
            return self.in_point()

        # Caso o robo esteja na borda
        if(section(self.position) in self.borders):

            self.RobotStateMachine.reach_ball_to_border()
            return self.in_border()

        # Caso a bola esteja no range de chute
        if (behind_ball(self.ball_position, self.position, self.team_side)):
            self.RobotStateMachine.reach_ball_to_spin()
            return self.in_spin()
        else:
            
            if (self.ball_position[0] > self.position[0]):
                self.position[0] -= 30
            # Segue a bola com o univector
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