from strategy.behaviour import TaskStatus, BlackBoard
from robot_module.movement.univector.un_field import UnivectorField
from robot_module.movement.definitions import OpCodes
from strategy.strategy_utils import spin_direction, ball_on_attack_side, robot_behind_ball
from strategy.behaviour import ACTION, TreeNode
from utils.json_handler import JsonHandler
from utils.math_utils import predict_speed, angle_between, clamp
from abc import ABC, abstractmethod
from typing import List, Tuple
import numpy as np
from rospy import logfatal
import math


class StopAction:

    def __init__(self, name = 'Stop Task'):
        self.name = name

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        return TaskStatus.RUNNING, (OpCodes.STOP, .0, 0, .0)


class SpinTask:
    def __init__(self, name = 'Spin Task'):
        self.name = name

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        return TaskStatus.RUNNING, (spin_direction(blackboard.ball_position,blackboard.position,
                                                   team_side=blackboard.team_side), 0.0, 255, .0)


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
        distance_to_ball = np.linalg.norm(blackboard.position - objective_position)

        if distance_to_ball < self.acceptance_radius:
            return TaskStatus.SUCCESS, None

        self.univector_field.update_obstacles(blackboard.enemies_position, [[0, 0]] * 5)  # blackboard.enemies_speed)
        angle = self.univector_field.get_angle_with_ball(blackboard.position, np.array([0, 0]),  # blackboard.speed,
                                                         objective_position, _attack_goal=blackboard.attack_goal)
        speed = self.speed
        if self.speed_prediction:
            raio = predict_speed(blackboard.position, [np.cos(blackboard.orientation), np.sin(blackboard.orientation)],
                                 objective_position, self.univector_field.get_attack_goal_axis(blackboard.attack_goal))
            cte = 90
            speed = (raio * cte) ** 0.5 + 10

        status = TaskStatus.RUNNING

        return status, (OpCodes.NORMAL, angle, speed, distance_to_ball)


class GoToPositionUsingUnivector(UnivectorTask):

    def __init__(self, name="Go to position", max_speed: int = 75, acceptance_radius: float = 10.0, speed_prediction: bool = False, position=None):
        super().__init__(name, max_speed, acceptance_radius, speed_prediction)
        self.position = position

    def set_position(self, new_pos):
        self.position = new_pos

    def run(self, blackboard: BlackBoard) -> (TaskStatus, (OpCodes, float, int, float)):
        return self.go_to_objective(blackboard, self.position)


class GoToBallUsingUnivector(UnivectorTask):

    def __init__(self, name, max_speed: int = 250, acceptance_radius: float = 10.0, speed_prediction: bool = True):
        super().__init__(name, max_speed, acceptance_radius, speed_prediction)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        return self.go_to_objective(blackboard, blackboard.ball_position)


class GoToAttackGoalUsingUnivector(UnivectorTask):
    def __init__(self, name, max_speed: int = 250, acceptance_radius: float = 10.0, speed_prediction: bool = True):
        super().__init__(name, max_speed, acceptance_radius, speed_prediction)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        return self.go_to_objective(blackboard, blackboard.attack_goal_pos)


class ChargeWithBall:

    def __init__(self, name='ChargeWithBall', max_speed: int = 255):
        self.name = name
        self.max_speed = max_speed

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        goal_vector = blackboard.attack_goal_pos - blackboard.position

        angle = angle_between(
            [np.cos(blackboard.orientation), np.sin(blackboard.orientation)],
            goal_vector
        )

        distance_to_goal = np.linalg.norm(goal_vector)

        return TaskStatus.RUNNING, (OpCodes.NORMAL, angle, self.max_speed, distance_to_goal)


class MarkBallOnAxis(TreeNode):
    def __init__(self, name: str = "AlignWithYAxis",
                 max_speed: int = 0, 
                 axis: np.ndarray = np.array([.0,1.0])):
        super().__init__(name)
        self.max_speed = max_speed
        self.angle_to_correct = angle_between(np.array([1.0,0.0]), axis)
    
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        direction = clamp(blackboard.ball_position) - blackboard.position
        distance = np.linalg.norm(direction)

        if distance < self.acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.NORMAL, 0, 0, distance)
    

class AlignWithAxis(TreeNode):
    def __init__(self, name: str = "AlignWithYAxis",
                 max_speed: int = 0, 
                 axis: np.ndarray = np.array([.0,1.0]),
                 acceptance_radius: float = 0.0872665):
        super().__init__(name)
        self.max_speed = max_speed
        self.acceptance_radius = acceptance_radius
        self.angle_to_correct = angle_between(np.array([1.0,0.0]), axis)
    
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if abs(self.angle_to_correct - abs(blackboard.orientation)) <= self.acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.INVALID, .0, 0, .0)
        else:
            return TaskStatus.RUNNING, (OpCodes.NORMAL, self.angle_to_correct, self.max_speed, .0)


class GoToGoalCenter(TreeNode):
    def __init__(self, name: str = "GoToGoalCenter", 
                       max_speed: int = 255, 
                       acceptance_radius: float = 10.0):
        super().__init__(name)
        self.acceptance_radius = acceptance_radius
        self.max_speed = max_speed
    
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        direction = blackboard.home_goal_pos - blackboard.position
        distance = np.linalg.norm(direction)

        if distance < self.acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.NORMAL, 0, 0, distance)

        theta = math.atan2(direction[1], direction[0])
        return TaskStatus.RUNNING, (OpCodes.NORMAL, theta, self.max_speed, distance)


class GoToPosition(TreeNode):

    def __init__(self, name: str = 'Straight Line Movement',
                 max_speed: int = 80,
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
        path = self.target_pos - blackboard.position
        distance = np.linalg.norm(path)
        theta = math.atan2(path[1], path[0])

        if distance <= self.acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.NORMAL, 0, 0, .0)

        return TaskStatus.RUNNING, (OpCodes.NORMAL, theta, self.max_speed, distance)


class PushToAttack(TreeNode):
    def __init__(self, name: str = 'Push ball to the other side of field',
                 max_speed: int = 255):
        self.name = name
        self.max_speed = max_speed

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:

        if ball_on_attack_side(blackboard):
            return TaskStatus.FAILURE, (OpCodes.INVALID, .0, 0, .0)
        else:

            if robot_behind_ball(blackboard):
                path = blackboard.ball_position - blackboard.position
                distance = np.linalg.norm(path)
                theta = math.atan2(path[1], path[0])
                return TaskStatus.RUNNING, (OpCodes.NORMAL, theta, self.max_speed, distance)

            else:
                return TaskStatus.FAILURE, (OpCodes.INVALID, .0, 0, .0)


class GoToBallUsingMove2Point(TreeNode):
    def __init__(self, name: str = "GoToBallUsingMove2Point",
                 max_speed = 200):
        self.max_speed = max_speed

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        direction = blackboard.position - blackboard.ball_position
        distance = np.linalg.norm(direction)
        theta = math.atan2(direction[1], direction[0])

        return TaskStatus.RUNNING, (OpCodes.NORMAL, theta, self.max_speed, distance)