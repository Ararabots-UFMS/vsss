import rospy
import numpy as np
from robot_module.hardware import RobotHardware

from robot_module.comunication.sender import Sender, STDMsg
from ROS.ros_robot_subscriber_and_publiser import RosRobotSubscriberAndPublisher

from strategy.behaviour import BlackBoard
from strategy.strategy_utils import GameStates
from strategy.attack_with_univector import AttackerWithUnivectorBT
from utils.json_handler import JsonHandler
from robot_module.control import Control

bodies_unpack = JsonHandler().read("parameters/bodies.json", escape=True)


class Robot:

    def __init__(self, robot_id: int,
                 tag: int,
                 robot_body: str,
                 team_side: int,
                 team_color: int,
                 robot_role: int,
                 _game_topic_name: str,
                 socket_id: int = -1,
                 should_debug: int = 0):

        # Parameters
        self.id = robot_id
        self.robot_body = robot_body
        self.tag = tag
        self._socket_id = socket_id
        self._should_debug = should_debug

        self.pid_list = bodies_unpack[self.robot_body]
        constants = [(255, self.pid_list['KP'], self.pid_list['KI'], self.pid_list['KD'])]

        self._hardware = RobotHardware()
        self._controller = Control(self, constants)

        # True position for penalty
        self.true_pos = np.array([.0, .0])
        self.velocity_buffer = []
        self.position_buffer = []

        # Receive from vision
        self.ball_position = None
        self.ball_speed = None
        
        self.position = None
        self.orientation = None
        self.speed = None

        self.team_pos = None
        self.team_orientation = None
        self.team_speed = None
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

        self.left_speed = self.right_speed = 0

        # Open bluetooth socket
        if self._socket_id == -1:
            rospy.logfatal("Using fake bluetooth")
            self._sender = None
        else:
            self._sender = Sender(self._socket_id)

        self.subsAndPubs = RosRobotSubscriberAndPublisher(self, _game_topic_name, self._should_debug)

        self.behaviour_trees = [
            AttackerWithUnivectorBT(),
            AttackerWithUnivectorBT(),
            AttackerWithUnivectorBT(),
            AttackerWithUnivectorBT(),
            AttackerWithUnivectorBT(),
            AttackerWithUnivectorBT()
        ]

        self.behaviour_tree = self.behaviour_trees[0]

        self.blackboard = BlackBoard()

        self.stuck_counter = 0

    def update_game_state_blackboard(self):
        self.blackboard.game_state = GameStates(self.game_state)
        self.blackboard.team_side = self.team_side
        self.blackboard.attack_goal = not self.team_side
        self.blackboard.team_color = self.team_color

        self.blackboard.freeball_robot_id = self.freeball_robot
        self.blackboard.meta_robot_id = self.meta_robot
        self.blackboard.penalty_robot_id = self.penalty_robot

    def update_game_info_blackboard(self):
        self.blackboard.ball_position = self.ball_position
        self.blackboard.ball_speed = self.ball_speed

        self.blackboard.my_id = self.id
        self.blackboard.role = self.role
        self.blackboard.position = self.position
        if np.all(self.position):
            self.blackboard.true_pos = self.position

        self.blackboard.orientation = self.orientation
        self.blackboard.speed = self.speed

        self.blackboard.team_pos = self.team_pos
        self.blackboard.team_orientation = self.team_orientation
        self.blackboard.team_speed = self.team_speed

        self.blackboard.enemies_position = self.enemies_position
        self.blackboard.enemies_orientation = self.enemies_orientation
        self.blackboard.enemies_speed = self.enemies_speed

    def run(self):
        op_code, angle, speed, dist = self.behaviour_tree.run(self.blackboard)
        left, right = self._controller.get_wheels_speeds(op_code, speed, angle, dist)
        msg = self._hardware.normalize_speeds(STDMsg(left, right))
        
        if self._sender is not None:
            priority = self.get_priority()
            self._sender.send(priority, self._hardware.encode(msg))
            
    def get_priority(self) -> int:
        distance = np.linalg.norm(self.blackboard.position - self.blackboard.ball_position)
        return int(distance) & 0xFF