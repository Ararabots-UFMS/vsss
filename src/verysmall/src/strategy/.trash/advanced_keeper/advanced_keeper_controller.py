import sys
import os
import rospy
import numpy as np
from strategy.strategy_utils import *
from .advanced_keeper import AdvancedGK, MyModel
from robot_module.movement.functions.movement import Movement
from utils.json_handler import JsonHandler
from utils.math_utils import angle_between

bodies_unpack = JsonHandler().read("parameters/bodies.json", escape=True)

SOFTWARE = 0
HARDWARE = 1

DISTANCE = 8.0
GOALKEEPER_SPEED = 110
MIN_X = 7.0
MAX_X = 150.0
GG_DIFF = 136.0
SPIN_SPEED = 255
DEFENCE_THRESHOLD = 5.0
PLUS  = 1
MINUS = -1
SAME = 0

class AdvancedGKController():

    def __init__(self, _robot_obj ,_robot_body="Nenhum", _debug_topic=None):
        self.pid_type = SOFTWARE
        self.robot = _robot_obj
        self.position = [None, None]
        self.orientation = None
        self.speed = None
        self.team_speed = None
        self.enemies_position = None
        self.enemies_speed = None
        self.ball_position = None
        self.ball_speed = None
        self.team_side = 0
        self.robot_body = _robot_body

        self.pid_list = [bodies_unpack[self.robot_body]['KP'],
                         bodies_unpack[self.robot_body]['KI'],
                         bodies_unpack[self.robot_body]['KD']]

        self.stop = MyModel(state='stop')
        self.AdvancedGK = AdvancedGK(self.stop)
        self.movement = Movement(self.pid_list, error=3, attack_goal= np.array([0,0]), _pid_type=self.pid_type,
                                 _debug_topic=_debug_topic)

        self.buffer = []
        self.position_buffer = []

        self.position_buffer_size = 10
        #self.buffer_size = 50
        self.buffer_size = 20

        self.fixed_positions = [
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 40.0],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 42.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 47.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 52.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 57.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 62.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 67.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 72.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 77.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 82.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 87.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 90.0],
                                ]



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
        if (self.robot.position[0] != 0.0) and (self.robot.position[1] != 0.0):
            self.position = self.robot.position
        # else:
        #     rospy.logfatal("Deu ruim")

        rospy.logfatal(self.AdvancedGK.current_state)

        self.orientation = self.robot.orientation
        self.speed = self.robot.speed
        self.team_speed = self.robot.team_speed
        self.enemies_position = self.robot.enemies_position
        self.enemies_speed = self.robot.enemies_speed
        self.ball_position = self.robot.ball_position
        self.ball_speed = self.robot.ball_speed
        self.team_side = self.robot.team_side

        self.movement.univet_field.update_attack_side(not self.team_side)

        self.robot.add_to_buffer(self.buffer, self.buffer_size, self.ball_position)
        self.robot.add_to_buffer(self.position_buffer, self.position_buffer_size, self.position)

        self.fixed_positions = [
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 40.0],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 42.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 47.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 52.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 57.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 62.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 67.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 72.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 77.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 82.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 87.5],
                                [MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 90.0],
                                ]

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
        #rospy.logfatal(self.AdvancedGK.current_state)

        """
        Transitions in normal game state

        :return: int, int
        """



        if self.AdvancedGK.is_stop:

            self.AdvancedGK.stop_to_normal()

        if self.AdvancedGK.is_normal:
            ball_section = section(self.ball_position)
            keeper_section = section(self.position)

            if keeper_section not in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA] or keeper_section in [LEFT_GOAL, RIGHT_GOAL]:

                self.AdvancedGK.normal_to_out_of_area()

            elif ball_section in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA]:
                if not on_attack_side(self.ball_position, self.team_side):

                    self.AdvancedGK.normal_to_defend_ball()  # defend
                else:

                    self.AdvancedGK.normal_to_seek_ball()  # seekeia

            elif ball_section in [LEFT_GOAL, RIGHT_GOAL]:

                self.AdvancedGK.normal_to_goal()

            else:

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
        #rospy.logfatal(self.AdvancedGK.current_state)

        ball_section = section(self.ball_position)


        # quando o goleiro sai da area
        if section(self.position) not in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA]:

            self.AdvancedGK.defend_ball_to_out_of_area()
            return self.in_out_of_area()


        # quando a bola entra na area
        elif ball_section in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA]:

            #quando bola na area de defesa
            if not on_attack_side(self.ball_position, self.team_side):

                if near_ball(self.ball_position, self.position, DISTANCE)  and (not inside_range(45.0, 85.0, self.ball_position[1]) ):

                    self.AdvancedGK.defend_ball_to_spin()
                    return self.in_spin() # proximo a bola

                ## TODO: EMPURRAR BOLA DA AREA
                elif (inside_range(45.0, 85.0, self.ball_position[1])):

                    if  behind_ball(self.ball_position, self.position, DISTANCE):

                        self.AdvancedGK.defend_ball_to_spin()
                        return self.in_spin()

                    else:
                        if self.ball_position[1] < self.position[1]:
                            self.defend_position = np.array([MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 40.0])
                        else:
                            self.defend_position = np.array([MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X), 90.0])


                        param1, param2, _ = self.movement.move_to_point(
                                                speed=GOALKEEPER_SPEED,
                                                robot_position=self.position,
                                                robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
                                                goal_position=self.defend_position
                                            )

                else:

                    self.AdvancedGK.defend_ball_to_go_to_ball()
                    return self.in_go_to_ball() #loge da bola
            else:

                # quando bola na area de ataque
                self.AdvancedGK.defend_ball_to_seek_ball()
                return self.in_seek_ball()

        #quando saii gol
        elif ball_section in [LEFT_GOAL, RIGHT_GOAL]:

            self.AdvancedGK.defend_ball_to_goal()
            return self.in_goal()
        else:
            #caso geral, tracking de bola, bola fora da area e robo dentro da area

            self.AdvancedGK.defend_ball_to_seek_ball()  # trackeia
            return self.in_seek_ball()  # trackeia

    def in_seek_ball(self):
        #rospy.logfatal(self.AdvancedGK.current_state)

        not_inside_left_limit = (self.team_side == LEFT and self.ball_position[0] > 30.0)
        not_inside_right_limit = (self.team_side == RIGHT and self.ball_position[0] < 120.0)

        inside_y_limit = inside_range(30.0, 100.0, self.ball_position[1])

        keeper_is_out_of_area = section(self.position) not in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA]

        # quando a bola entra em uma das area
        if section(self.ball_position) in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA]:

            # quando a bola esta na area de defesa
            if not on_attack_side(self.ball_position, self.team_side):

                self.AdvancedGK.seek_ball_to_defend_ball()
                return self.in_defend_ball()

            else:
                # caso generico, tracking de bola
                return self.follow_ball()

        # goleiro fora da area com histerese de 15 cm (TESTAR FUNCIONAMENTO da HISTERESE)
        elif((keeper_is_out_of_area and ( not_inside_right_limit or not_inside_left_limit )) or (not inside_y_limit) ):

            self.AdvancedGK.seek_ball_to_out_of_area()
            return self.in_out_of_area()

        else:
            # caso geral, tracking de bola
            return self.follow_ball()
            ## TODO: APOS CHEGAR FAZER HEAD TO para [0,1]

    def in_spin(self):
        # faz spin e retorna para defendball
        #rospy.logfatal(self.AdvancedGK.current_state)

        self.AdvancedGK.spin_to_defend_ball()
        param1, param2, param3 = self.movement.spin(SPIN_SPEED, not spin_direction(self.ball_position, self.position, self.team_side))
        return param1, param2, self.pid_type

    def in_go_to_ball(self):

        #rospy.logfatal(self.AdvancedGK.current_state)
        self.AdvancedGK.go_to_ball_to_defend_ball()
        param1, param2, param3 = self.movement.move_to_point(
            GOALKEEPER_SPEED,
            self.position,
            [np.cos(self.orientation), np.sin(self.orientation)],
            np.array([self.position[0], self.ball_position[1]])
            )

        return param1, param2, self.pid_type



    ##### TODO:
    ## - melhorar previsao
    ## - lentidao na resposta
    ## - melhorar velocidade
    ## - melhorar andamento
    ## - head_to antes de andar ou depois de atingir objetivo
    ## - implementar empurrar bola dentro da area na frente do gol
    ## -


    def follow_ball(self):

        mean = self.robot.buffer_mean(self.buffer)

        if distance_point(self.ball_position, mean) > DEFENCE_THRESHOLD:
            polynom = None#self.robot.buffer_polyfit(self.buffer, 1)

            position = polynom(MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X))

            if position > 90.0:
                position = 90.0
            elif position < 40.0:
                position = 40.0

            self.defend_position[0] = MAX_X*self.team_side + ((-1)**(self.team_side)*MIN_X)
            self.defend_position[1] = position

        else:

            if self.ball_speed[1] > (2.0):    discount = PLUS
            elif self.ball_speed[1] < (-2.0):  discount = MINUS
            else:   discount = SAME

            if inside_range(0.0,40.0,self.ball_position[1]):
                self.defend_position = self.fixed_positions[0 if (0+discount <= 0) else discount]
            elif inside_range(40.1,45.0,self.ball_position[1]):
                self.defend_position = self.fixed_positions[1+discount]
            elif inside_range(45.1,50.0,self.ball_position[1]):
                self.defend_position = self.fixed_positions[2+discount]
            elif inside_range(50.1,55.0,self.ball_position[1]):
                self.defend_position = self.fixed_positions[3+discount]
            elif inside_range(55.1,60.0,self.ball_position[1]):
                self.defend_position = self.fixed_positions[4+discount]
            elif inside_range(60.1,65.0,self.ball_position[1]):
                self.defend_position = self.fixed_positions[5+discount]
            elif inside_range(65.1,70.0,self.ball_position[1]):
                self.defend_position = self.fixed_positions[6+discount]
            elif inside_range(70.1,75.0,self.ball_position[1]):
                self.defend_position = self.fixed_positions[7+discount]
            elif inside_range(75.1,80.0,self.ball_position[1]):
                self.defend_position = self.fixed_positions[8+discount]
            elif inside_range(80.1,85.0,self.ball_position[1]):
                self.defend_position = self.fixed_positions[9+discount]
            elif inside_range(85.1,90.0,self.ball_position[1]):
                self.defend_position = self.fixed_positions[10+discount]
            else:# inside_range(90.1,130.0,self.ball_position[1]):
                self.defend_position = self.fixed_positions[11 if (11+discount >= 11) else 11+discount]

        rospy.logfatal(self.defend_position)
        param_1, param_2, param3 = self.movement.move_to_point(
            speed=GOALKEEPER_SPEED,
            robot_position=self.position,
            robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
            goal_position=self.defend_position
        )

        return param_1, param_2, self.pid_type



    def in_goal(self):
        #rospy.logfatal(self.AdvancedGK.current_state)

        # if section(self.ball_position) in [LEFT_GOAL_AREA, RIGHT_GOAL_AREA]:
        if section(self.ball_position) not in [LEFT_GOAL, RIGHT_GOAL]:
            self.AdvancedGK.goal_to_defend_ball()

        return 0, 0, 0



    def in_out_of_area(self):
        #rospy.logfatal(self.AdvancedGK.current_state)

        goal_position = np.array([0, 0])

        range_left = inside_range(self.position[0], 0.0, self.ball_position[0])
        range_right = inside_range(self.position[0], 150.0, self.ball_position[0])

        # bola entre goleiro e gol,
        if (( range_left and self.team_side == LEFT) or ( range_right and self.team_side == RIGHT)):


            ## bola alianhada com goleiro, volta para ponto fixo
            if (inside_range(self.ball_position[1] + 4, self.ball_position[1] - 4, self.position[1]) ):
                goal_position = np.array([MIN_X + self.team_side*GG_DIFF, 40.0])

            ## bola acima/abaixo do goleiro, volta reto pro gol
            else:
                goal_position = np.array([MIN_X + self.team_side*GG_DIFF,  self.position[1]])


        # goleiro entre bola e gol, volta para x calculado e y da bola
        else:

            goal_position[0] = MIN_X + self.team_side * GG_DIFF
            goal_position[1] = self.ball_position[1]




        param1, param2, param3 = self.movement.move_to_point(
            GOALKEEPER_SPEED,
            self.position,
            [np.cos(self.orientation), np.sin(self.orientation)],
            goal_position
        )

        if param3:

            self.AdvancedGK.out_of_area_to_seek_ball()

        return param1, param2, self.pid_type

    def in_freeball_game(self):

        """
        Transitions in freeball state

        :return: int, int
        """

        #rospy.logfatal(self.AdvancedGK.current_state)

        if self.AdvancedGK.is_stop:
            self.AdvancedGK.stop_to_freeball()

        if self.AdvancedGK.is_freeball:
            self.AdvancedGK.freeball_to_normal()

        if self.AdvancedGK.is_normal:
            return self.in_normal_game()

    def in_penalty_game(self):
        #rospy.logfatal(self.AdvancedGK.current_state)


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
        #rospy.logfatal(self.AdvancedGK.current_state)


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


    def half_speed(self, position, goal_position, base_speed):
        if distance_point(position, goal_position) <= self.movement.error_margin*1.5:
            return base_speed*0.5
        else:
            return base_speed
