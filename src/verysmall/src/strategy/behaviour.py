from typing import Tuple
import time
from enum import Enum
from abc import abstractmethod, ABC
import numpy as np
from strategy.arena_utils import RIGHT, LEFT
from strategy.strategy_utils import GameStates
from robot_module.movement.definitions import OpCodes
from rospy import logwarn
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
        self.game = Game()

        self.enemy_goal = Goal()
        self.home_goal = Goal()

        self.ball = MovingBody()

        self.robot = FriendlyRobot()

        self.home_team = HomeTeam()
        self.enemy_team = EnemyTeam()

    def set_robot_variables(self, robot_position, robot_speed, robot_orientation):
        self.robot.position = robot_position
        self.robot.speed = robot_speed
        self.robot.orientation = robot_orientation

    def __repr__(self):
        return 'BlackBoard:\n' + str(self.game) + "\n" + str(self.home_team) + str(self.enemy_team)


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


class MovingBody:
    def __init__(self, buffer_size=30):
        self.position = np.array([0, 0])
        self._position_buffer_length = buffer_size / 60
        self.position_buffer_x = [0 for _ in range(buffer_size)]
        self.position_buffer_y = [0 for _ in range(buffer_size)]
        self.time_buffer = [float(time.time()) for i in range(buffer_size)]

        # self.position_buffer_time = [0 for _ in range(position_buffer_length)]
        self.speed = np.array([0, 0])
        self.orientation = .0

    def __setattr__(self, key, value):
        if key == 'position' and (value[0] or value[1]):
            self.position_buffer_x.pop(0)
            self.position_buffer_x.append(value[0])
            self.position_buffer_y.pop(0)
            self.position_buffer_y.append(value[1])

            self.time_buffer.pop(0)
            self.time_buffer.append(float(time.time()))
        super().__setattr__(key, value)

    def get_predicted_position_over_seconds(self, seconds_in_future=0.5):

        fitx = np.polyfit(self.time_buffer, self.position_buffer_x, 1)
        fity = np.polyfit(self.time_buffer, self.position_buffer_y, 1)
        px = np.poly1d(fitx)
        py = np.poly1d(fity)
        p = px(time.time() + seconds_in_future), py(time.time() + seconds_in_future)
        return np.array(p)

    def __repr__(self):
        return "--position: " + str(self.position) + \
               "--speed: " + str(self.speed) + \
               "--orientation: " + str(self.orientation)


class FriendlyRobot(MovingBody):
    def __init__(self):
        super().__init__()
        self.id = 0
        self.role = 0
        self.last_know_location = None

    def __setattr__(self, key, value):
        if key == 'position' and (value[0] or value[1]):
            self.last_know_location = value
        super().__setattr__(key, value)


class Goal:
    def __init__(self):
        self.side = RIGHT
        self.position = np.array([0, 0])

    def __setattr__(self, key, value):
        if key == 'side':
            super().__setattr__(key, value)
            super().__setattr__('position', np.array([value * 150, 65]))


class Game:
    def __init__(self):
        self.state = GameStates.STOPPED
        self.meta_robot_id = 0
        self.freeball_robot_id = 0
        self.penalty_robot_id = 0

    def __repr__(self):
        return "-GameState: " + \
               "\n--state: " + str(self.state) + \
               "\n--meta_robot_id: " + str(self.meta_robot_id) + \
               "\n--freeball_robot_id: " + str(self.freeball_robot_id) + \
               "\n--penalty_robot_id: " + str(self.penalty_robot_id)


class Team(ABC):
    def __init__(self):
        self._positions = np.array([[0, 0] for _ in range(5)])
        self._speeds = np.array([[0, 0] for _ in range(5)])
        self._orientations = np.array([0 for _ in range(5)])
        self.robots = []
        self.number_of_robots = 0
        self.maximum_number_of_robots = 5

    def __repr__(self):
        return "--positions: " + str(self._positions.tolist()) + \
               "\n--speeds: " + str(self._speeds.tolist()) + \
               "\n--orientations: " + str(self._orientations.tolist()) + \
               "\n--number_of_robots: " + str(self.number_of_robots) + \
               "\n--maximum_number_of_robots: " + str(self.maximum_number_of_robots) + \
               "\n--robots: " + str(self.robots)

    def create_new_robot(self):
        self._positions = np.append(self._positions, [[0, 0]], axis=0)
        self._speeds = np.append(self._speeds, [[0, 0]], axis=0)
        self._orientations = np.append(self._orientations, [0], axis=0)

    def set_team_variables(self, robot_positions, robot_orientations, robot_speeds):

        self.number_of_robots = 0

        for robot_position, robot_orientation, robot_speed in zip(robot_positions, robot_orientations, robot_speeds):
            if np.any(robot_position):
                self._positions[self.number_of_robots] = robot_position
                self._orientations[self.number_of_robots] = robot_orientation
                self._speeds[self.number_of_robots] = robot_speed

                self.robots[self.number_of_robots].position = robot_position
                self.robots[self.number_of_robots].orientation = robot_orientation
                self.robots[self.number_of_robots].speed = robot_speed

                self.number_of_robots += 1

                if self.maximum_number_of_robots <= self.number_of_robots:
                    self.maximum_number_of_robots = self.number_of_robots
                    self.create_new_robot()

    def __len__(self):
        return self.number_of_robots

    def __getitem__(self, item):
        if self.number_of_robots:
            return self.robots[item]
        else:
            raise StopIteration

    @property
    def positions(self):
        return self._positions[:self.number_of_robots]

    @property
    def orientations(self):
        return self._orientations[:self.number_of_robots]

    @property
    def speeds(self):
        return self._speeds[:self.number_of_robots]


class EnemyTeam(Team):
    def __init__(self):
        super().__init__()
        self.robots = [MovingBody() for _ in range(5)]

    def create_new_robot(self):
        self.robots.append(MovingBody())
        super().create_new_robot()

    def __repr__(self):
        return "-EnemyTeam:\n" + super().__repr__()


class HomeTeam(Team):
    def __init__(self):
        super().__init__()
        self.robots = [FriendlyRobot() for _ in range(5)]

    def create_new_robot(self):
        self.robots.append(FriendlyRobot())
        super().create_new_robot()

    def __repr__(self):
        return "-HomeTeam:\n" + super().__repr__()