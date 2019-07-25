from strategy.behaviour import Selector, BlackBoard, ACTION, TaskStatus
from strategy.base_trees import Penalty, FreeBall

class GoalKeeper(Selector):
    def __init__(self, name: str = "behave"):
        super().__init__(name)
        self.children.append(Penalty())
        self.children.append(FreeBall())