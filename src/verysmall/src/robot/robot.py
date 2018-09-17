import rospy
import sys
import time
import numpy as np
from verysmall.msg import things_position, game_topic
from comunication.sender import Sender
import os
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT'] + "src/"
from strategy.attacker_with_univector import AttackerWithUnivector
from utils.json_handler import JsonHandler


class Robot():
    """docstring for Robot"""

    def __init__(self, robot_name, mac_address, robot_body, isAdversary=False):
        # Parameters
        self.robot_name = robot_name
        self.robot_id_integer = int(self.robot_name.split("_")[1]) - 1
        self.mac_address = mac_address # Mac address
        self.robot_body = robot_body

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

        # Open bluetooth socket
        self.bluetooth_sender = Sender(self.robot_id_integer, self.mac_address)
        self.bluetooth_sender.connect()

        rospy.Subscriber('things_position', things_position, self.read_topic)

        rospy.Subscriber('game_topic', game_topic, self.read_game_topic)

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
            self.state_machine.statemachine.stop_to_normal()
        elif self.game_state == 2:  # Freeball
            pass
        elif self.game_state == 3:  # Penalty
            pass
        elif self.game_state == 4:  # Meta
            pass
        else:  # I really really really Dont Know
            print("wut")

        if self.changed_game_state:
            rospy.logfatal("Robo_" + self.robot_name + ": Run("+self.game_state_string[self.game_state]+")")
            self.changed_game_state = False

    def read_parameters(self):
        #TODO: ler parametros do robot na funcao init
        pass

    def read_game_topic(self, data):
        self.game_state = data.game_state
        self.penalty_robot = data.penalty_robot
        self.freeball_robot = data.freeball_robot
        self.meta_robot = data.meta_robot
        self.role = data.robot_roles[self.robot_id_integer]
        self.changed_game_state = True

    def read_topic(self, data):
        self.ball_position = np.nan_to_num(np.array(data.ball_pos))
        self.ball_speed = np.nan_to_num(np.array(data.ball_speed))
        self.team_pos = np.nan_to_num(np.array(data.team_pos)).reshape((5, 2))
        self.team_orientation = np.nan_to_num(np.array(data.team_orientation))
        self.team_speed = np.nan_to_num(np.array(data.team_speed)).reshape((5, 2))
        self.position = self.team_pos[self.robot_id_integer]
        self.orientation = self.team_orientation[self.robot_id_integer]
        self.enemies_position = np.nan_to_num(data.enemies_pos).reshape((5, 2))
        self.enemies_orientation = np.nan_to_num(data.enemies_orientation)
        self.enemies_speed = np.nan_to_num(data.enemies_pos).reshape((5, 2))

        self.run()

    def debug(self):
        pass

    def bluetooth_detach(self):
        if self.bluetooth_sender is not None:
            self.bluetooth_sender.closeSocket()
