import math
import time
from abc import ABC, abstractmethod
from typing import Iterable
from typing import Tuple
import rospy
import numpy as np

from robot_module.movement.definitions import OpCodes
from robot_module.movement.univector.un_field import UnivectorField
from strategy.arena_utils import LEFT_AREA_CENTER_X, RIGHT_AREA_CENTER_X, ROBOT_SIZE, y_axis_section, RIGHT, \
    HALF_ARENA_WIDTH
from strategy.behaviour import ACTION, TreeNode
from strategy.behaviour import TaskStatus, BlackBoard, NO_ACTION
from strategy.strategy_utils import spin_direction
from utils.json_handler import JsonHandler
from utils.math_utils import predict_speed, angle_between, clamp
from utils.profiling_tools import log_warn
from strategy.arena_utils import univector_pos_section, ArenaSections, HALF_ARENA_HEIGHT


class StopAction(TreeNode):

    def __init__(self, name: str = 'Stop Task'):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        return TaskStatus.RUNNING, (OpCodes.STOP, .0, 0, .0)


class SpinTask(TreeNode):
    def __init__(self, name='Spin Task', invert=False):
        super().__init__(name)
        self.invert = invert

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        return TaskStatus.RUNNING, (spin_direction(blackboard.ball.position, blackboard.robot.position,
                                                   team_side=blackboard.home_goal.side, invert=self.invert), 0.0, 255,
                                    .0)


class UnivectorTask(ABC):
    def __init__(self, name: str,
                 max_speed: int = 250,
                 acceptance_radius: float = 10.0,
                 speed_prediction: bool = True):
        self.name = name
        self.speed = max_speed
        self.speed_prediction = speed_prediction
        self.acceptance_radius = acceptance_radius

        self.univector_field = UnivectorField()
        self.univector_field.update_constants()

    @abstractmethod
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        raise Exception("subclass must override run method")

    def go_to_objective(self, blackboard: BlackBoard, objective_position):
        distance_to_ball = np.linalg.norm(blackboard.robot.position - objective_position)

        if distance_to_ball < self.acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.STOP, 0, 0, 0)

        self.univector_field.update_obstacles(blackboard.enemy_team.positions,
                                              [[0, 0]] * 5)  # blackboard.enemies_speed)
        angle = self.univector_field.get_angle_with_ball(blackboard.robot.position, np.array([0, 0]),
                                                         # blackboard.speed,
                                                         objective_position, _attack_goal=blackboard.enemy_goal.side)
        speed = self.speed
        if self.speed_prediction:
            raio = predict_speed(blackboard.robot.position,
                                 [np.cos(blackboard.robot.orientation), np.sin(blackboard.robot.orientation)],
                                 objective_position,
                                 self.univector_field.get_attack_goal_axis(blackboard.enemy_goal.side))
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


class RecoverBallUsingUnivector(UnivectorTask):
    def __init__(self, name: str = "Recover Ball Using Univector", max_speed: int = 250, acceptance_radius: float = 7.0,
                 speed_prediction: bool = True):
        super().__init__(name, max_speed, acceptance_radius, speed_prediction)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        distance_to_ball = np.linalg.norm(blackboard.robot.position - blackboard.ball.position)

        if distance_to_ball < self.acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.STOP, 0, 0, 0)

        self.univector_field.update_obstacles(blackboard.enemy_team.positions,
                                              [[0, 0]] * 5)  # blackboard.enemies_speed)

        axis = (0.0, 1.0) if blackboard.ball.position[1] > HALF_ARENA_HEIGHT else (0.0, -1.0)
        angle = self.univector_field.get_angle_vec(blackboard.robot.position, blackboard.robot.orientation,
                                                   blackboard.ball.position, axis)

        status = TaskStatus.RUNNING

        return status, (OpCodes.SMOOTH, angle, self.speed, distance_to_ball)


class GoToBallUsingUnivector(UnivectorTask):

    def __init__(self, name: str = "Follow Ball", max_speed: int = 250, acceptance_radius: float = 10.0,
                 speed_prediction: bool = True):
        super().__init__(name, max_speed, acceptance_radius, speed_prediction)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        status, action = self.go_to_objective(blackboard, blackboard.ball.position)
        rospy.logfatal(action[1])
        rospy.logfatal(blackboard.robot.orientation)
        rospy.logfatal("----------------------------------")
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
        goal_vector = blackboard.enemy_goal.position - blackboard.robot.position

        angle = angle_between(
            self.x_vector,
            goal_vector,
            abs=False
        )

        distance_to_goal = np.linalg.norm(goal_vector)

        return TaskStatus.RUNNING, (OpCodes.SMOOTH, angle, self.max_speed, distance_to_goal)


class MarkBallOnAxis(TreeNode):
    def __init__(self, name: str = "MarkBallOnAxis",
                 max_speed: int = 255,
                 axis: np.ndarray = np.array([.0, 1.0]),
                 acceptance_radius: float = 5,
                 clamp_min: float = None,
                 clamp_max: float = None,
                 predict_ball: bool = False
                 ):
        super().__init__(name)
        self._acceptance_radius = acceptance_radius
        self._max_speed = max_speed
        self._angle_to_correct = angle_between(np.array([1.0, 0.0]), axis)
        self.turn_off_clamp = clamp_min is None and clamp_max is None
        self._clamp_min = clamp_min
        self._clamp_max = clamp_max
        self._predict_ball = predict_ball

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:

        if self._predict_ball:
            t = blackboard.ball.get_time_on_axis(axis=0, value=blackboard.robot.position[0])
            predicted_position = blackboard.ball.get_predicted_position_over_seconds(t)
            if abs(predicted_position[1] - blackboard.robot.position[1]) > self._acceptance_radius:
                target_position = predicted_position
            else:
                target_position = blackboard.ball.position
        else:
            target_position = blackboard.ball.position

        if self.turn_off_clamp:
            direction = target_position[1] - blackboard.robot.position[1]
        else:
            direction = clamp(target_position[1], self._clamp_min, self._clamp_max) - \
                        blackboard.robot.position[1]

        distance = abs(direction)

        if distance < self._acceptance_radius:
            return TaskStatus.RUNNING, (OpCodes.NORMAL,
                                        -self._angle_to_correct if direction < 0 else self._angle_to_correct, 0,
                                        distance)

        return TaskStatus.RUNNING, (OpCodes.NORMAL,
                                    -self._angle_to_correct if direction < 0 else self._angle_to_correct,
                                    self._max_speed,
                                    .0)


class MarkBallOnYAxis(TreeNode):
    def __init__(self, clamp_min: Iterable,
                 clamp_max: Iterable,
                 max_speed: int = 255,
                 name: str = "MarkBallOnYAxis",
                 acceptance_radius: float = 2):
        super().__init__(name)
        self._acceptance_radius = acceptance_radius
        self._max_speed = max_speed

        self._clamp_min = np.array(clamp_min)
        self._clamp_max = np.array(clamp_max)

    def set_clamps(self, clamp_min: Iterable, clamp_max: Iterable) -> None:
        self._clamp_min = clamp_min
        self._clamp_max = clamp_max

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        norm_distance = abs(blackboard.ball.position[0] - HALF_ARENA_WIDTH) / HALF_ARENA_WIDTH

        if norm_distance > 0.6:
            ball_y = blackboard.ball.position[1]
        else:
            scalar = 1 - norm_distance
            t = blackboard.ball.get_time_on_axis(axis=0, value=blackboard.robot.position[0])
            ball_y = blackboard.ball.get_predicted_position_over_seconds(t)[1]

        y = clamp(ball_y, self._clamp_min[1], self._clamp_max[1])

        target_pos = np.array([self._clamp_min[0], y])
        direction = target_pos - blackboard.robot.position
        distance = np.linalg.norm(direction)

        if distance < self._acceptance_radius:
            return TaskStatus.SUCCESS, NO_ACTION

        direction /= distance

        def gaussian(m, v):
            return math.exp(-(m ** 2) / (2 * (v ** 2)))

        alpha = gaussian(distance - self._acceptance_radius, 4.5)

        y_sign = -1 if direction[1] < 0 else 1
        direction_on_target = np.array([0, y_sign])

        final_direction = alpha * direction_on_target + (1 - alpha) * direction
        theta = math.atan2(final_direction[1], final_direction[0])

        return TaskStatus.RUNNING, (OpCodes.NORMAL, theta, self._max_speed, distance)


class AlignWithAxis(TreeNode):
    def __init__(self, name: str = "AlignWithAxis",
                 max_speed: int = 0,
                 axis: np.ndarray = np.array([.0, 1.0]),
                 acceptance_radius: float = 0.0872665, align_with_ball: bool = False):

        super().__init__(name)
        self.max_speed = max_speed
        self.acceptance_radius = acceptance_radius
        self.angle_to_correct = angle_between(np.array([1.0, 0.0]), axis)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if abs(self.angle_to_correct - abs(blackboard.robot.orientation)) <= self.acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.INVALID, .0, 0, .0)
        else:
            return TaskStatus.RUNNING, (OpCodes.SMOOTH, self.angle_to_correct, self.max_speed, .0)


class GetOutOfGoal(TreeNode):

    def __init__(self, name: str = "Go to goal area center",
                 max_speed: int = 255,
                 acceptance_radius: float = 10.0,
                 target_pos: list = None):
        super().__init__(name)
        self.acceptance_radius = acceptance_radius
        self.max_speed = max_speed
        self.target_pos = target_pos

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        team_goal_side = blackboard.home_goal.side
        robot_pos = blackboard.robot.position
        y_section = y_axis_section(robot_pos)
        new_y_pos = 0

        if self.target_pos is None:
            if y_section:
                shift = -ROBOT_SIZE
            else:
                shift = ROBOT_SIZE

            new_y_pos = robot_pos[1] + shift
            if team_goal_side:
                self.target_pos = (RIGHT_AREA_CENTER_X, new_y_pos)
            else:
                self.target_pos = (LEFT_AREA_CENTER_X, new_y_pos)
        path = self.target_pos - robot_pos

        distance = np.linalg.norm(path)
        theta = math.atan2(path[1], path[0])

        if distance <= self.acceptance_radius:
            self.target_pos = None

            return TaskStatus.SUCCESS, (OpCodes.STOP, 0, 0, .0)

        return TaskStatus.RUNNING, (OpCodes.SMOOTH, theta, self.max_speed, distance)


class MoveBackward(TreeNode):

    def __init__(self, name: str = "MoveBackward",
                 max_speed: int = 255,
                 acceptance_radius: float = 10.0,
                 distance: float = 0.0):
        super().__init__(name)
        self.acceptance_radius = acceptance_radius
        self.max_speed = max_speed
        self.distance = distance

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        orientation = blackboard.robot.orientation

        return TaskStatus.RUNNING, (OpCodes.SMOOTH, -orientation, self.max_speed, self.distance)


class GoToGoalCenter(TreeNode):

    def __init__(self, name: str = "GoToGoalCenter",
                 max_speed: int = 255,
                 acceptance_radius: float = 10.0):
        super().__init__(name)
        self.acceptance_radius = acceptance_radius
        self.max_speed = max_speed

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        home_goal_pos = blackboard.home_goal.position
        if blackboard.home_goal.side == RIGHT:
            home_goal_pos[0] = RIGHT_AREA_CENTER_X
        else:
            home_goal_pos[0] = LEFT_AREA_CENTER_X

        direction = home_goal_pos - blackboard.robot.position

        distance = np.linalg.norm(direction)

        if distance < self.acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.SMOOTH, 0, 0, distance)

        theta = math.atan2(direction[1], direction[0])
        return TaskStatus.RUNNING, (OpCodes.SMOOTH, theta, self.max_speed, distance)


class GoToPosition(TreeNode):

    def __init__(self, name: str = "Straight Line Movement",
                 max_speed: int = 255,
                 acceptance_radius: float = 10.0,
                 target_pos: list = None):
        super().__init__(name)
        self.acceptance_radius = acceptance_radius
        self.max_speed = max_speed
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


class GoToBallUsingMove2Point(TreeNode):
    def __init__(self, name: str = "GoToBallUsingMove2Point", speed=100, acceptance_radius: float = 6):
        super().__init__(name)
        self.speed = speed
        self.acceptance_radius = acceptance_radius

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        direction = blackboard.ball.position - blackboard.robot.position
        distance = np.linalg.norm(direction)
        theta = math.atan2(direction[1], direction[0])
        if distance < self.acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.NORMAL, 0.0, 0, 0)

        return TaskStatus.RUNNING, (OpCodes.NORMAL, theta, self.speed, distance)


class GoBack(TreeNode):
    def __init__(self, name: str = 'GoBack',
                 max_speed: int = 80,
                 acceptance_radius: float = 10.0):
        super().__init__(name)
        self.acceptance_radius = acceptance_radius
        self.max_speed = max_speed

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        target_position = [75., blackboard.robot.position[1]]
        path = target_position - blackboard.robot.position
        distance = np.linalg.norm(path)
        theta = math.atan2(path[1], path[0])

        if distance <= self.acceptance_radius:
            return TaskStatus.SUCCESS, (OpCodes.NORMAL, 0, 0, .0)

        return TaskStatus.RUNNING, (OpCodes.NORMAL, theta, self.max_speed, distance)



