import rospy
import sys
import numpy as np
import random
from comunication.sender import Sender
import os
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT'] + "src/"
from ROS.ros_robot_subscriber_and_publiser import RosRobotSubscriberAndPublisher
from strategy.attacker_with_univector_controller import AttackerWithUnivectorController
from strategy.base_controller import RobotStateMachineController
from strategy.set_pid_machine_controller import SetPIDMachineController

class Robot():
    """docstring for Robot"""

    def __init__(self, _robot_name, _tag, _mac_address, _robot_body, _game_topic_name, _should_debug = 0):
        # Parameters
        self.robot_name = _robot_name
        self.robot_id_integer = int(self.robot_name.split("_")[1]) - 1
        self.mac_address = _mac_address # Mac address
        self.robot_body = _robot_body
        self.tag = int(_tag)
        self.should_debug = _should_debug

        # Receive from vision
        self.ball_position = None
        self.ball_speed = None
        self.team_pos = None
        self.team_orientation = None
        self.team_speed = None
        self.position = None
        self.orientation = None
        self.enemies_position = None
        self.enemies_orientation = None
        self.enemies_speed = None

        # Receive from game topic
        self.behaviour_type = None
        self.game_state = 0
        self.role = None
        self.penalty_robot = None
        self.freeball_robot = None
        self.meta_robot = None
        self.team_side = 0
        # right(1) or left(0)
        self.left_side = 0
        self.right_side = not self.left_side

        # Open bluetooth socket
        if self.mac_address == '-1':
            rospy.logfatal("Using fake bluetooth")
            self.bluetooth_sender = None
        else:
            self.bluetooth_sender = Sender(self.robot_id_integer, self.mac_address)
            self.bluetooth_sender.connect()

        self.subsAndPubs = RosRobotSubscriberAndPublisher(self, _game_topic_name, _should_debug)

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
            AttackerWithUnivectorController(_robot_body = self.robot_body, _debug_topic = self.subsAndPubs),
            AttackerWithUnivectorController(_robot_body = self.robot_body, _debug_topic = self.subsAndPubs),
            AttackerWithUnivectorController(_robot_body = self.robot_body, _debug_topic = self.subsAndPubs),
            SetPIDMachineController(_robot_body=self.robot_body, _debug_topic=self.subsAndPubs)
        ]

        self.state_machine = AttackerWithUnivectorController(_robot_body=self.robot_body, _debug_topic = self.subsAndPubs)

    def run(self):
        #rospy.logfatal(str(self.robot_body))
        self.state_machine.update_game_information(position=self.position, orientation=self.orientation,
                                                   team_speed=[0, 0], enemies_position=self.enemies_position,
                                                   enemies_speed=self.enemies_speed, ball_position=self.ball_position, team_side = self.team_side)
        if self.game_state == 0:  # Stopped
            param_A, param_B, param_C = self.state_machine.set_to_stop_game()
        elif self.game_state == 1:  # Normal Play
            param_A, param_B, param_C = self.state_machine.in_normal_game()
            # rospy.logfatal(str(param_A)+" "+ str(param_B))
        elif self.game_state == 2:  # Freeball
            param_A, param_B, param_C = self.state_machine.in_freeball_game()
        elif self.game_state == 3:  # Penalty
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
        if self.bluetooth_sender:
            self.bluetooth_sender.send_movement_package([param_A, param_B], param_C)

        if self.should_debug:
            pass

        if self.changed_game_state:
            rospy.logfatal("Robo_" + self.robot_name + ": Run("+self.game_state_string[self.game_state]+") side: " +
                           str(self.team_side))
            self.changed_game_state = False

    def read_parameters(self):
        #TODO: ler parametros do robot na funcao init
        pass

    def debug(self):
        pass

    def bluetooth_detach(self):
        if self.bluetooth_sender is not None:
            self.bluetooth_sender.closeSocket()
