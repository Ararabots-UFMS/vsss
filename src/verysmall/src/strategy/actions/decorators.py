import time
import numpy as np
from abc import abstractmethod
from typing import Tuple
from strategy.behaviour import TaskStatus, BlackBoard, ACTION
from robot_module.movement.definitions import OpCodes
from strategy.arena_utils import ArenaSections, section, y_axis_section, MAX_H_SIZE, ROBOT_SIZE
from strategy.strategy_utils import is_parallel_border, is_robot_parallel, is_robot_stucked_on_border
import rospy


class Decorator:
    def __init__(self, name: str):
        self.name = name
        self.child = None

    def add_child(self, child):
        self.child = child

    @abstractmethod
    def run(self, blackboard: BlackBoard):
        pass


class IgnoreFailure(Decorator):
    def __init__(self, name: str = 'IgnoreFailure'):
        super().__init__(name)

    def run(self, blackboard: BlackBoard):
        status, action = self.child.run(blackboard)
        if status == TaskStatus.RUNNING:
            return TaskStatus.RUNNING, action

        return TaskStatus.SUCCESS, action


class InvertOutput(Decorator):
    def __init__(self, name: str = 'Not'):
        super().__init__(name)

    def run(self, blackboard: BlackBoard):
        if self.child is None:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
        else:
            status, action = self.child.run(blackboard)

            if status != TaskStatus.RUNNING:
                if status == TaskStatus.FAILURE:
                    return TaskStatus.SUCCESS, action
                else:
                    return TaskStatus.FAILURE, action

            return status, action


class Timer(Decorator):
    def __init__(self, name: str = 'Timeout', exec_time: float = 0):
        super().__init__(name)
        self.exec_time = exec_time
        self.initial_time = time.time()
        self.current_time = None

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if self.child is None:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
        else:
            self.current_time = time.time()
            if self.current_time - self.initial_time < self.exec_time:
                return self.child.run(blackboard)
            else:
                self.initial_time = time.time()
                return TaskStatus.SUCCESS, (OpCodes.INVALID, 0, 0, 0)


class IgnoreSmoothing(Decorator):
    def __init__(self, name: str = "IgnoreSmoothing"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if self.child is None:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
        else:
            status, action = self.child.run(blackboard)
            if action[0] == OpCodes.SMOOTH:
                action = (OpCodes.NORMAL, action[1], action[2], action[3])
            return status, action


class DoNTimes(Decorator):
    def __init__(self, name: str = "Do N times", n: int = 1):
        super().__init__(name)
        self.n = n

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if self.child is None:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
        else:
            if self.n > 0:
                status, action = self.child.run(blackboard)
                if status != TaskStatus.RUNNING:
                    self.n -= 1
                return status, action
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)


class KeepRunning(Decorator):
    def __init__(self, name: str = "KeepRunningDecorator"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard):
        _, action = self.child.run(blackboard)
        return TaskStatus.RUNNING, action


class StatusChanged(Decorator):
    def __init__(self, name: str = "StatusChanged", function=None):
        super().__init__(name)
        self.last_status = TaskStatus.SUCCESS
        self._function = function

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        status, action = self.child.run(blackboard)

        if status != self.last_status:
            self._function()

        self.last_status = status

        return status, action


class SmoothBorderSpeed(Decorator):
    def __init__(self, name: str = "Smooth Border Speed"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if self.child is None:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
        else:
            status, action = self.child.run(blackboard)
            ball_pos = blackboard.ball.position
            robot_pos = blackboard.robot.position
            ball_sec = section(ball_pos)
            orientation = blackboard.robot.orientation
            min_speed = 50
            speed = action[2]
            distance = abs(robot_pos[1] - ball_pos[1])

            if is_robot_stucked_on_border(robot_pos, orientation):
                return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)

            if ball_sec == ArenaSections.UP_BORDER or ball_sec == ArenaSections.DOWN_BORDER:
                if not is_parallel_border(orientation,
                                          blackboard.home_goal.side,
                                          max_angle=15):
                    if distance < 20:
                        speed = min_speed

            action = (action[0], action[1], speed, action[3])
            return status, action


class ToggleFrontOnBorder(Decorator):
    def __init__(self, name: str = "Toggle Front on Border"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if self.child is None:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
        else:
            status, action = self.child.run(blackboard)
            robot_pos = blackboard.robot.position
            orientation = blackboard.robot.orientation

            if is_robot_stucked_on_border(robot_pos, orientation):
                action = (action[0] + OpCodes.TOGGLE_FRONT, action[1], action[2], action[3])
            return status, action
