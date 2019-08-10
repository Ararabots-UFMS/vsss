import time
from abc import abstractmethod
from typing import Tuple
from strategy.behaviour import TaskStatus, BlackBoard, ACTION
from robot_module.movement.definitions import OpCodes
import rospy

class Decorator:
    def __init__(self, name):
        self.name = name
        self.children = []

    def add_child(self, child):
        self.children.append(child)

    @abstractmethod
    def run(self, blackboard: BlackBoard):
        pass

class IgnoreFailure(Decorator):
    def __init__(self, name = 'IgnoreFailure'):
        super().__init__(name)
    
    def run(self, blackboard: BlackBoard):
        for c in self.children:
            status, action = c.run(blackboard)
            if status == TaskStatus.RUNNING:
                return TaskStatus.RUNNING, action

        return TaskStatus.SUCCESS, action


class InvertOutput(Decorator):
    def __init__(self, name: str='Not'):
        super().__init__(name)

    def run(self, blackboard: BlackBoard):
        if self.children is None:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
        else:
            for c in self.children:                
                status, action = c.run(blackboard)

                if status != TaskStatus.RUNNING:
                    if status == TaskStatus.FAILURE:
                        return TaskStatus.SUCCESS, action
                    else:
                        return TaskStatus.FAILURE, action

                return TaskStatus.SUCCESS, (OpCodes.INVALID, 0, 0, 0)
            

class Timer(Decorator):
    def __init__(self, name: str='Timeout', exec_time: float=0):
        super().__init__(name)
        self.exec_time = exec_time
        self.initial_time = time.time()
        self.current_time = None

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if self.children is None:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
        else:
            for c in self.children:
                self.current_time = time.time()
                if self.current_time - self.initial_time < self.exec_time:
                    return c.run(blackboard)
                else:
                    self.initial_time = time.time()
                    return TaskStatus.SUCCESS, (OpCodes.INVALID, 0, 0, 0)
