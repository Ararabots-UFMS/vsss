import rospy
import sys
import numpy as np
import random
import os
from robot_module.hardware import RobotHardware

from robot_module.comunication.sender import Sender, STDMsg
from ROS.ros_robot_subscriber_and_publiser import RosRobotSubscriberAndPublisher
from strategy.attacker_with_univector.attacker_with_univector_controller import AttackerWithUnivectorController
from strategy.advanced_keeper.advanced_keeper_controller import AdvancedGKController
from strategy.set_pid_machine_controller import SetPIDMachineController
from strategy.zagueiro.zagueiro_controller import ZagueiroController
from strategy.strategy_utils import behind_ball, on_attack_side, spin_direction
from strategy.naive_attacker.naive_attacker_controller import NaiveAttackerController
from strategy.naive_keeper.naive_keeper_controller import NaiveGKController

SOFTWARE = 0
HARDWARE = 1

class Robot():

    def __init__(self, robot_id:int,
                tag: int, 
                robot_body: str,
                team_side: int,
                team_color: int,
                robot_role: int,
                _game_topic_name: str,
                socket_id:int = -1,
                should_debug:int = 0):

        # Parameters
        self.id = robot_id
        self.robot_body = robot_body
        self.tag = tag
        self._socket_id = socket_id
        self._should_debug = should_debug

        self.hardware = RobotHardware()

        # True position for penalty
        self.true_pos = np.array([.0,.0])
        self.velocity_buffer = []
        self.position_buffer = []

        # Receive from vision
        self.ball_position = None
        self.ball_speed = None
        self.team_pos = None
        self.team_orientation = None
        self.team_speed = None
        self.position = None
        self.orientation = None
        self.speed = None
        self.enemies_position = None
        self.enemies_orientation = None
        self.enemies_speed = None

        # Receive from game topic
        self.team_color = team_color
        self.team_side = team_side
        self.role = robot_role
        self.game_state = 0
        self.penalty_robot = None
        self.freeball_robot = None
        self.meta_robot = None
        # right(1) or left(0)
        self.left_side = 0
        self.right_side = not self.left_side #TODO: pq isso aqui?
        
        self.left_speed = self.right_speed = 0

        # Open bluetooth socket
        if self._socket_id == -1:
            rospy.logfatal("Using fake bluetooth")
            self.sender = None
        else:
            self.sender = Sender(self._socket_id)

        self.subsAndPubs = RosRobotSubscriberAndPublisher(self, _game_topic_name, self._should_debug)

        self.changed_game_state = True
        self.game_state_string = ["stop",
                                  "Normal Play",
                                  "Freeball",
                                  "Penaly",
                                  "Univector",
                                  "Running",
                                  "Border",
                                  "Point",
                                  "Meta"]
        self.strategies = [
            NaiveAttackerController(_robot_obj = self, _robot_body = self.robot_body),
            AttackerWithUnivectorController(_robot_obj = self, _robot_body = self.robot_body),
            AdvancedGKController(_robot_obj = self, _robot_body = self.robot_body),
            ZagueiroController(_robot_obj=self, _robot_body=self.robot_body),
            SetPIDMachineController(_robot_obj = self, _robot_body=self.robot_body),
            NaiveGKController(_robot_obj = self, _robot_body=self.robot_body)
        ]

        self.state_machine = self.strategies[self.role]

        self.stuck_counter = 0

    def run(self):

        self.state_machine.update_game_information()

        if self.game_state == 0:  # Stopped
            param_A, param_B, param_C = self.state_machine.set_to_stop_game()

        elif self.game_state == 1:  # Normal Play
            param_A, param_B, param_C = self.state_machine.in_normal_game()
            # rospy.logfatal(str(param_A)+" "+ str(param_B))

        elif self.game_state == 2:  # Freeball

            if self.id == self.freeball_robot:
                rospy.logfatal(str(self.id)+" Vo bate freeball")
                param_A, param_B, param_C = self.freeball_routine()
            else:
                self.game_state = 1
                param_A, param_B, param_C = self.state_machine.in_freeball_game()

        elif self.game_state == 3:  # Penalty

            if self.id == self.penalty_robot:
                rospy.logfatal(str(self.id)+" Vo bate penalty")
                param_A, param_B, param_C = self.penalty_routine()
            else:
                self.game_state = 1
                param_A, param_B, param_C = self.state_machine.in_penalty_game()

        elif self.game_state == 4:  # meta
            param_A, param_B, param_C = self.state_machine.in_meta_game()

        else:  # I really really really Dont Know
            print("wut")
        # ========================================================
        #             SOFTWARE        |    HARDWARE
        # Param A :    LEFT           |      Theta
        # Param B :    RIGHT          |      Speed
        # ========================================================

        self.add_to_buffer(self.velocity_buffer, 10, param_A)
        self.add_to_buffer(self.velocity_buffer, 10, param_B)

        #self.add_to_buffer(self.position_buffer, 10, self.position)
        if param_C: # if is hardware
            self.left_speed = param_B
            self.right_speed = param_B
        else:
            self.left_speed = param_A
            self.right_speed = param_B
            msg = STDMsg(self.left_speed, self.right_speed)

        if self.sender is not None:
            priority = self.get_priority()
            self.sender.send(priority, self.hardware.encode(msg))

        if self._should_debug:
            pass

        if self.changed_game_state:
            # rospy.logfatal("Robo_" + self.robot_name + ": Run("+self.game_state_string[self.game_state]+") side: " +
            #                str(self.team_side))
            self.changed_game_state = False

    def get_priority(self) -> int:
        distance = np.linalg.norm(self.position - self.ball_position)
        return int(distance) & 0xFF

    def read_parameters(self):
        #TODO: ler parametros do robot na funcao init
        pass

    def debug(self):
        pass

    def add_to_buffer(self, buffer, buffer_size, element):
        if(len(buffer) > buffer_size):
            buffer.pop(0)

        buffer.append(element)


    # ATTENTION: for now pass just np arrays or numbers please !!
    def buffer_mean(self, buffer):
        sum = 0.
        length = len(buffer)
        for i in range(length):
            sum += buffer[i]
        if length != 0:
            return sum/length
        else:
            return sum/1
# ATTENTION: just use if you have a list of np.arrays of dim 2 !!
    def buffer_polyfit(self, buffer, degree):
        x = []
        y = []
        for element in buffer:
            x.append(element[0])
            y.append(element[1])

        return np.poly1d(np.polyfit(np.asarray(x), np.asarray(y), degree))

    def penalty_routine(self):
        if np.all(self.position):
            self.true_pos = self.position

        if behind_ball(self.ball_position, self.true_pos, self.team_side, _distance = 25):
            param_1, param_2, _ = self.state_machine.movement.move_to_point(
                220, np.array(self.position),
                [np.cos(self.orientation), np.sin(self.orientation)],
                np.array([(not self.team_side)*150, 65]))

            return param_1, param_2, SOFTWARE #0.0, 250, HARDWARE
        else:
            self.game_state = 1
            return self.state_machine.in_penalty_game()

    def freeball_routine(self):
        if np.all(self.position):
            self.true_pos = self.position
        if behind_ball(self.ball_position, self.true_pos, self.team_side, _distance=15):
            param_a, param_b, _ = self.state_machine.movement.do_univector(
                speed=250,
                robot_position=self.position,
                robot_vector=[np.cos(self.orientation), np.sin(self.orientation)],
                robot_speed=np.array([0, 0]),
                obstacle_position=self.enemies_position,
                obstacle_speed=[[0,0]]*5,
                ball_position=self.ball_position,
                only_forward=False,
                speed_prediction=True)
            rospy.logfatal(str(param_a))
            return param_a, param_b, SOFTWARE  # 0.0, 250, HARDWARE
        else:
            self.game_state = 1
            return self.state_machine.in_freeball_game()

    # def get_stuck(self, position):
    #     """
    #     Returns if the robot is stuck or not based on its wheels velocity and the velocity seen by
    #     the vision node
    #     :param position: int
    #     :return: nothing
    #     """
    #     if sum(self.velocity_buffer):
    #
    #         # position_sum = self.buffer_mean(self.position_buffer)
    #         # if np.any( abs(position_sum - np.array(position)) < 3 ):
    #         #     return True
    #
    #         if (self.speed[0] < 1) and (self.speed[1] < 1):
    #             return True
    #
    #     return False
    #

    def get_stuck(self, position):
        """
        Returns if the robot is stuck or not based on its wheels velocity and the velocity seen by
        the vision node
        :param position: int
        :return: nothing
        """
        if sum(self.velocity_buffer):

        #     # if np.any( abs(position_sum - np.array(position)) < 3 ):
        #     #     return True

        #     # position_sum = self.buffer_mean(self.position_buffer)

        #     # diff = abs(position_sum - np.array(position))
        #     # diff = (diff[0] < 0.5) and (diff[1] < 0.5)

            if ((self.speed[0] < 1) and (self.speed[1] < 1)):
                self.stuck_counter+=1
                return (self.stuck_counter > 60)
            else:
                if (self.stuck_counter > 0):
                    self.stuck_counter -= 1
                else:
                    self.stuck_counter = 0

        return False

    def get_fake_stuck(self, position):
            return False
