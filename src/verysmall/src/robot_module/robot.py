import time
from typing import List, Tuple
import rospy
import numpy as np
import cv2

from robot_module.hardware import RobotHardware
from robot_module.comunication.sender import Sender, STDMsg
from ROS.ros_robot_subscriber_and_publiser import RosRobotSubscriberAndPublisher

from strategy.behaviour import BlackBoard, TaskStatus, OpCodes
from strategy.strategy_utils import GameStates
from strategy.defender import Defender
from strategy.attacker import Attacker
from strategy.pid_calibration import CalibrationTree
from strategy.goalkeeper import GoalKeeper
from utils.json_handler import JsonHandler
from robot_module.control import Control
from robot_module.movement.univector.debug import drawRobot, drawBall
from interface.virtualField import virtualField, unit_convert, position_from_origin

class Robot:

    def __init__(self, robot_id: int,
                 tag: int,
                 robot_body: str,
                 team_side: int,
                 team_color: int,
                 robot_role: int,
                 _owner_name: str,
                 socket_id: int = -1,
                 should_debug: int = 0):

        # Parameters
        self.id = robot_id

        self.robot_body = robot_body
        self.tag = tag
        self._socket_id = socket_id
        self._should_debug = should_debug
        self.owner_name = _owner_name

        constants = self.get_pid_constants_set()
        rospy.logwarn(constants)
        self._max_fine_movment_speed = 80
        self._controller = Control(self, constants, self._max_fine_movment_speed)

        self._hardware = RobotHardware()

        self.blackboard = BlackBoard()
        self.blackboard.my_id = self.id
        self.blackboard.home_goal.side = team_side
        self.blackboard.enemy_goal.side = not team_side
        self.blackboard.robot.role = robot_role

        self.velocity_buffer = []
        self.position_buffer = []

        # Receive from game topic
        self.team_color = team_color

        self.left_speed = self.right_speed = 0

        # Open bluetooth socket
        if self._socket_id == -1:
            rospy.logfatal("Using fake bluetooth")
            self._sender = None
        else:
            self._sender = Sender(self._socket_id, self.owner_name)

        # ROBOTO VISION
        self.imgField = virtualField(4*150, 4*130)  # cv2.imread('src/robot_module/movement/univector/img/vss-field.jpg')

        self.behaviour_trees = [
            Attacker(),
            Attacker(),
            GoalKeeper(),
            Attacker(),
            CalibrationTree(),
            Defender()
        ]

        self.behaviour_tree = self.behaviour_trees[robot_role]
        self.stuck_counter = 0

        self.subsAndPubs = RosRobotSubscriberAndPublisher(self, 'game_topic_' + str(self.owner_name),
                                                          self._should_debug)

    @property
    def position(self):
        return self.blackboard.robot.position

    @property
    def speed(self):
        return self.blackboard.robot.speed

    @property
    def orientation(self):
        return self.blackboard.robot.orientation

    def get_pid_constants_set(self) -> List[Tuple]:
        pid_set = []
        bodies = JsonHandler.read("parameters/bodies.json", escape=True)

        pid_dict = bodies[self.robot_body]
        for speed in pid_dict:
            ctes = pid_dict[speed]
            pid_set.append((int(speed), ctes["KP"], ctes["KI"], ctes["KD"]))

        return pid_set

    def run(self):
        task_status, action = self.behaviour_tree.run(self.blackboard)
        if task_status == TaskStatus.FAILURE or task_status is None:
            action = (OpCodes.STOP, 0.0, 0, 0)

        left, right = self._controller.get_wheels_speeds(*action)
        msg = self._hardware.normalize_speeds(STDMsg(left, right))
        if self._sender is not None:
            priority = self.get_priority()
            self._sender.send(priority, self._hardware.encode(msg))

        self.roboto_vision()

    def get_priority(self) -> int:
        distance = np.linalg.norm(self.blackboard.robot.position - self.blackboard.ball.position)
        return int(distance) & 0xFF

    def get_next_speed(self):
        return (self._hardware._allowed_speed + self._hardware._speed_step) % 256

    def roboto_vision(self):
        self.imgField.plot_ball(self.blackboard.ball.position)
        temp = time.time()
        t = self.blackboard.ball.get_time_on_axis(0, self.blackboard.ball.position[0])
        rospy.logfatal(t)
        ma_ball = self.blackboard.ball.get_predicted_position_over_seconds(t)

        ma_ball = unit_convert(ma_ball, self.imgField.width_conv, self.imgField.height_conv)
        ma_ball = position_from_origin(ma_ball, self.imgField.field_origin)
        rospy.logfatal(ma_ball)
        rospy.logfatal(self.blackboard.ball.position)
        #rospy.logwarn(ma_ball)
        cv2.circle(self.imgField.field, ma_ball, self.imgField.ball_radius,
                   self.imgField.colors["red"], -1)
        cv2.imshow('Robot\'o Vision', self.imgField.field)
        cv2.waitKey(1)
