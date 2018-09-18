import rospy
import sys
import time
import numpy as np
from comunication.sender import Sender
import os
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT'] + "src/"
from ROS.ros_robot_subscriber import RosRobotSubscriber
from strategy.univector_statemachine import AttackerWithUnivector
from utils.json_handler import JsonHandler


class Robot():
    """docstring for Robot"""

    def __init__(self, _robot_name, _tag, _mac_address, _robot_body, isAdversary=False):
        # Parameters
        self.robot_name = _robot_name
        self.robot_id_integer = int(self.robot_name.split("_")[1]) - 1
        self.mac_address = _mac_address # Mac address
        self.robot_body = _robot_body
        self.tag = _tag

        # Receive from vision
        self.position = None
        self.vector = None
        self.ball_position = None
        self.team_vector = None
        self.team_position = None
        self.enemies_vector = None
        self.enemies_position = None

        # Calculated inside robot
        self.motor_speed = None
        self.PID = None

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
        self.bluetooth_sender = Sender(self.robot_id_integer, self.mac_address)
        self.bluetooth_sender.connect()

        self.subs = RosRobotSubscriber(self)

        self.changed_game_state = True
        self.game_state_string = ["Stopped",
                                  "Normal Play",
                                  "Freeball",
                                  "Penaly",
                                  "Meta"]

        self.state_machine = AttackerWithUnivector()

    def run(self):
        if self.game_state == 0:  # Stopped
            self.state_machine.statemachine.current_state = self.state_machine.statemachine.stop
            self.bluetooth_sender.sendPacket(0, 0)
        elif self.game_state == 1:  # Normal Play
            left, right, _ = self.state_machine.action(200, self.position, self.orientation, 0, self.enemies_position, self.enemies_speed, self.ball_position)
            self.bluetooth_sender.sendPacket(left, right)
        elif self.game_state == 2:  # Freeball
            pass
        elif self.game_state == 3:  # Penalty
            pass
        elif self.game_state == 4:  # Meta
            pass
        else:  # I really really really Dont Know
            print("wut")

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
