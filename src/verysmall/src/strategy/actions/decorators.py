import time
from abc import abstractmethod
from typing import Tuple
from strategy.behaviour import TaskStatus, BlackBoard, ACTION
from robot_module.movement.definitions import OpCodes

class Decorator:
    def __init__(self, name):
        self.name = name
        self.child = None

    def add_child(self, child):
        self.child = child

    @abstractmethod
    def run(self, blackboard: BlackBoard):
        pass


class InvertOutput(Decorator):
    def __init__(self, name: str='Not'):
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


class Timer(Decorator):
    def __init__(self, name: str='Timeout', exec_time: float=0):
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
                return TaskStatus.SUCCESS, (OpCodes.INVALID, 0, 0, 0)
