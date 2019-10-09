import time
from abc import abstractmethod
from typing import Tuple
from strategy.behaviour import TaskStatus, BlackBoard, ACTION
from robot_module.movement.definitions import OpCodes
from strategy.arena_utils import univector_pos_section, ArenaSections, HALF_ARENA_HEIGHT
from utils.math_utils import range_convert
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
            ball_sec = univector_pos_section(ball_pos)
            robot_sec = univector_pos_section(robot_pos)

            if ball_sec == ArenaSections.UP_BORDER or ball_sec == ArenaSections.DOWN_BORDER:
                if robot_sec != ball_sec:
                    action = (action[0], action[1], action[2], action[3])
            return status, action


class CurveSmoothing(Decorator):
    def __init__(self, name: str = "Curve Smoothing", n_prev_angles: int = 5):
        super().__init__(name)
        self.n_prev_angles = n_prev_angles
        self.prev_angles = []

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if self.child is None:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
        else:
            status, action = self.child.run(blackboard)
            angle = action[1]
            if len(self.prev_angles) < self.n_prev_angles:
                self.prev_angles.append(angle)
            else:
                if any(abs(abs(x) - abs(angle)) > .8 for x in self.prev_angles):
                    action = (action[0], action[1], 50, action[3])
                self.prev_angles.append(angle)
                self.prev_angles.pop(0)

            return status, action
