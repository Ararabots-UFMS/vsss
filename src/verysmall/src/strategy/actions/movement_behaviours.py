from strategy.behaviour import TaskStatus, BlackBoard
from robot_module.movement.univector.un_field import UnivectorField
from robot_module.movement.definitions import OpCodes
from strategy.strategy_utils import spin_direction
from strategy.arena_utils import LEFT_GOAL_CENTER_X, RIGHT_GOAL_CENTER_X, HALF_ARENA_HEIGHT
from strategy.behaviour import ACTION, TreeNode
from utils.json_handler import JsonHandler
from utils.math_utils import predict_speed, angle_between, clamp
from abc import ABC, abstractmethod
from typing import List, Tuple
import numpy as np
from rospy import logfatal
import math


class StopAction(TreeNode):

    def __init__(self, name: str = 'Stop Task'):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        return TaskStatus.RUNNING, (OpCodes.STOP, .0, 0, .0)


class SpinTask(TreeNode):
    def __init__(self, name='Spin Task'):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        return TaskStatus.RUNNING, (spin_direction(blackboard.ball.position, blackboard.robot.position,
                                                   team_side=blackboard.home_goal.side), 0.0, 255, .0)


class UnivectorTask(ABC):
    def __init__(self, name: str,
                 max_speed: int = 250,
                 acceptance_radius: float = 10.0,
                 speed_prediction: bool = True):
        self.name = name
        self.speed = max_speed
        self.speed_prediction = speed_prediction
        self.acceptance_radius = acceptance_radius

        univector_list = JsonHandler().read("parameters/univector_constants.json")

        # univector
        RADIUS = univector_list['RADIUS']
        KR = univector_list['KR']
        K0 = univector_list['K0']
        DMIN = univector_list['DMIN']
        LDELTA = univector_list['LDELTA']

        self.univector_field = UnivectorField()
        self.univector_field.update_constants(RADIUS, KR, K0, DMIN, LDELTA)

    @abstractmethod
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        raise Exception("subclass must override run method")
        pass

    def go_to_objective(self, blackboard: BlackBoard, objective_position):
        distance_to_ball = np.linalg.norm(blackboard.robot.position - objective_position)

        if distance_to_ball < self.acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.STOP, 0, 0, 0) 

        self.univector_field.update_obstacles(blackboard.enemy_team.positions, [[0, 0]] * 5)  # blackboard.enemies_speed)
        angle = self.univector_field.get_angle_with_ball(blackboard.robot.position, np.array([0, 0]),
                                                         # blackboard.speed,
                                                         objective_position, _attack_goal=blackboard.enemy_goal.side)
        speed = self.speed
        if self.speed_prediction:
            raio = predict_speed(blackboard.robot.position,
                                 [np.cos(blackboard.robot.orientation), np.sin(blackboard.robot.orientation)],
                                 objective_position, self.univector_field.get_attack_goal_axis(blackboard.enemy_goal.side))
            cte = 90
            speed = (raio * cte) ** 0.5 + 10

        status = TaskStatus.RUNNING

        return status, (OpCodes.SMOOTH, angle, speed, distance_to_ball)


class GoToPositionUsingUnivector(UnivectorTask):

    def __init__(self, name="Go to position", max_speed: int = 75, acceptance_radius: float = 10.0,
                 speed_prediction: bool = False, position=None):
        super().__init__(name, max_speed, acceptance_radius, speed_prediction)
        self.position = position

    def set_position(self, new_pos):
        self.position = new_pos

    def run(self, blackboard: BlackBoard) -> (TaskStatus, (OpCodes, float, int, float)):
        return self.go_to_objective(blackboard, self.position)


class GoToBallUsingUnivector(UnivectorTask):

    def __init__(self, name: str = "Follow Ball", max_speed: int = 250, acceptance_radius: float = 10.0, speed_prediction: bool = True):
        super().__init__(name, max_speed, acceptance_radius, speed_prediction)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        return self.go_to_objective(blackboard, blackboard.ball.position)


class GoToAttackGoalUsingUnivector(UnivectorTask):
    def __init__(self, name, max_speed: int = 250, acceptance_radius: float = 10.0, speed_prediction: bool = True):
        super().__init__(name, max_speed, acceptance_radius, speed_prediction)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        return self.go_to_objective(blackboard, blackboard.enemy_goal.position)


class ChargeWithBall(TreeNode):
    def __init__(self, name='ChargeWithBall', max_speed: int = 255):
        super().__init__(name)
        self.max_speed = max_speed
        self.x_vector = np.array([1.0, 0.0])

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        logfatal("CHARGE")
        goal_vector = blackboard.enemy_goal.position - blackboard.robot.position

        angle = angle_between(
            self.x_vector,
            goal_vector,
            False
        )

        distance_to_goal = np.linalg.norm(goal_vector)

        return TaskStatus.RUNNING, (OpCodes.SMOOTH, angle, self.max_speed, distance_to_goal)


class MarkBallOnAxis(TreeNode):
    def __init__(self, name: str = "AlignWithYAxis",
                 max_speed: int = 255,
                 axis: np.ndarray = np.array([.0, 1.0]),
                 acceptance_radius: float = 5,
                 clamp_min: float = None,
                 clamp_max: float = None
                 ):
        super().__init__(name)
        self._acceptance_radius = acceptance_radius
        self._max_speed = max_speed
        self._angle_to_correct = angle_between(np.array([1.0, 0.0]), axis)
        self.turn_off_clamp = clamp_min is None and clamp_max is None
        self._clamp_min = clamp_min
        self._clamp_max = clamp_max

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:

        if self.turn_off_clamp:
            direction = blackboard.ball.position[1] - blackboard.robot.position[1]
        else:
            direction = clamp(blackboard.ball.position[1], self._clamp_min, self._clamp_max) - \
                        blackboard.robot.position[1]

        distance = abs(direction)

        if distance < self._acceptance_radius:
            return TaskStatus.RUNNING, (
            OpCodes.SMOOTH, -self._angle_to_correct if direction < 0 else self._angle_to_correct,
            0, distance)

        return TaskStatus.RUNNING, (OpCodes.SMOOTH,
                                    -self._angle_to_correct if direction < 0 else self._angle_to_correct,
                                    self._max_speed,
                                    .0)


class AlignWithAxis(TreeNode):
    def __init__(self, name: str = "AlignWithYAxis",
                 max_speed: int = 0,
                 axis: np.ndarray = np.array([.0, 1.0]),
                 acceptance_radius: float = 0.0872665):
        super().__init__(name)
        self.max_speed = max_speed
        self.acceptance_radius = acceptance_radius
        self.angle_to_correct = angle_between(np.array([1.0, 0.0]), axis)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if abs(self.angle_to_correct - abs(blackboard.robot.orientation)) <= self.acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.INVALID, .0, 0, .0)
        else:
            return TaskStatus.RUNNING, (OpCodes.SMOOTH, self.angle_to_correct, self.max_speed, .0)


class GoToGoalCenter(TreeNode):
    def __init__(self, name: str = "GoToGoalCenter",
                 max_speed: int = 255,
                 acceptance_radius: float = 10.0):
        super().__init__(name)
        self.acceptance_radius = acceptance_radius
        self.max_speed = max_speed

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        direction = blackboard.home_goal.position - blackboard.robot.position
        distance = np.linalg.norm(direction)

        if distance < self.acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.SMOOTH, 0, 0, distance)

        theta = math.atan2(direction[1], direction[0])
        return TaskStatus.RUNNING, (OpCodes.SMOOTH, theta, self.max_speed, distance)


class GoToPosition(TreeNode):

<<<<<<< HEAD
    def __init__(self, name: str = 'Straight Line Movement',
                 max_speed: int = 255,
=======
    def __init__(self, name: str = "Straight Line Movement",
                 max_speed: int = 80,
>>>>>>> 1dc398a974798ac662ba8c8e656c96bde7cab3f9
                 acceptance_radius: float = 10.0,
                 position: list = None,
                 target_pos: list = None):
        super().__init__(name)
        self.acceptance_radius = acceptance_radius
        self.max_speed = max_speed
        self.position = position
        self.target_pos = target_pos

    def set_new_target_pos(self, new_pos):
        self.target_pos = new_pos

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        path = self.target_pos - blackboard.robot.position
        distance = np.linalg.norm(path)
        theta = math.atan2(path[1], path[0])

        if distance <= self.acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.SMOOTH, 0, 0, .0)

        return TaskStatus.RUNNING, (OpCodes.SMOOTH, theta, self.max_speed, distance)

class GoToGoalAreaCenter(TreeNode):

    def __init__(self, name: str = "Go to goal area center",
                 max_speed: int = 255,
                 acceptance_radius: float = 10.0):
        super().__init__(name)
        self.acceptance_radius = acceptance_radius
        self.max_speed = max_speed
        self.target_pos = None


    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]: 
        team_goal_side = blackboard.home_goal
        
        if team_goal_side:
            self.target_pos = (RIGHT_GOAL_CENTER_X, HALF_ARENA_HEIGHT)
        else:
            self.target_pos = (LEFT_GOAL_CENTER_X, HALF_ARENA_HEIGHT)

        path = self.target_pos - blackboard.robot.position
        distance = np.linalg.norm(path)
        theta = math.atan2(path[1], path[0])

        

        if distance <= self.acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.SMOOTH, 0, 0, .0)

        return TaskStatus.RUNNING, (OpCodes.SMOOTH, theta, self.max_speed, distance)