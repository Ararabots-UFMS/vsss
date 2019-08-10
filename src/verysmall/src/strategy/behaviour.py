from typing import Tuple
from enum import Enum
from abc import abstractmethod
import rospy
import numpy as np


from robot_module.movement.definitions import OpCodes


angle = distance = float
speed = int
ACTION = Tuple[OpCodes, angle, speed, distance]
NO_ACTION = (-1, 0, 0, 0)

class TaskStatus(Enum):
    SUCCESS = 0
    FAILURE = 1
    RUNNING = 2


class BlackBoard:
    """docstring for BlackBoard"""

    def __init__(self):
        self.game_state = None
        self.team_side = None
        
        self._attack_goal = None
        self.attack_goal_pos = None
        self.home_goal_pos = None

        self.freeball_robot_id = None
        self.meta_robot_id = None
        self.penalty_robot_id = None

        self.ball_position = None
        self.ball_speed = None

        self.my_id = None
        self.role = None
        self.position = None
        self.true_pos = None
        self.orientation = None
        self.speed = None

        self.team_color = None
        self.team_pos = None
        self.team_orientation = None
        self.team_speed = None

        self.enemies_position = None
        self.enemies_orientation = None
        self.enemies_speed = None
    
    @property
    def attack_goal(self) -> int:
        return self._attack_goal
    
    @attack_goal.setter
    def attack_goal(self, value: int) -> None:
        self._attack_goal = value
        self.attack_goal_pos = np.array([self._attack_goal * 150, 65])
        self.home_goal_pos = np.array([(1 - self._attack_goal) * 150, 65])

    def __repr__(self):
        return 'BlackBoard:\n' + \
               '\t--self.game_state: ' + str(self.game_state) + '\n' + \
               '\t--self.team_side: ' + str(self.team_side) + '\n' + \
               '\t--self.attack_goal: ' + str(self.attack_goal) + '\n' + \
               '\t--self.freeball_robot_id: ' + str(self.freeball_robot_id) + '\n' + \
               '\t--self.meta_robot_id: ' + str(self.meta_robot_id) + '\n' + \
               '\t--self.penalty_robot_id: ' + str(self.penalty_robot_id) + '\n' + \
               '\t--self.ball_position: ' + str(self.ball_position) + '\n' + \
               '\t--self.ball_speed: ' + str(self.ball_speed) + '\n' + \
               '\t--self.my_id: ' + str(self.my_id) + '\n' + \
               '\t--self.role: ' + str(self.role) + '\n' + \
               '\t--self.position: ' + str(self.position) + '\n' + \
               '\t--self.orientation: ' + str(self.orientation) + '\n' + \
               '\t--self.speed: ' + str(self.speed) + '\n' + \
               '\t--self.team_color: ' + str(self.team_color) + '\n' + \
               '\t--self.team_pos: ' + str(self.team_pos) + '\n' + \
               '\t--self.team_orientation: ' + str(self.team_orientation) + '\n' + \
               '\t--self.team_speed: ' + str(self.team_speed) + '\n' + \
               '\t--self.enemies_position: ' + str(self.enemies_position) + '\n' + \
               '\t--self.enemies_orientation: ' + str(self.enemies_orientation) + '\n' + \
               '\t--self.enemies_speed: ' + str(self.enemies_speed) + '\n'

class TreeNode:
    def __init__(self, name):
        self.name = name
        self.children = []
    
    def add_child(self, node) -> None:
        self.children.append(node)

    @abstractmethod
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        raise Exception("subclass must override run")

class Sequence(TreeNode):
    """
        A sequence runs each task in order until one fails,
        at which point it returns FAILURE. If all tasks succeed, a SUCCESS
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """

    def __init__(self, name):
        super().__init__(name)

    def run(self, blackboard):
        for c in self.children:
            status, action = c.run(blackboard)

            if status != TaskStatus.SUCCESS:
                return status, action

        return TaskStatus.SUCCESS, NO_ACTION


class Selector(TreeNode):
    """
        A selector runs each task in order until one succeeds,
        at which point it returns SUCCESS. If all tasks fail, a FAILURE
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """

    def __init__(self, name: str):
        super().__init__(name)

    def run(self, blackboard) -> Tuple[TaskStatus, ACTION]:

        for c in self.children:
            status, action = c.run(blackboard)
            if status != TaskStatus.FAILURE:
                return status, action

        return TaskStatus.FAILURE, NO_ACTION