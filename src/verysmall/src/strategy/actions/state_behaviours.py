from strategy.behaviour import TaskStatus, Sequence, BlackBoard
from strategy.strategy_utils import GameStates


class InState:
    def __init__(self, name, _desired_state):
        self.name = name
        self.desired_state = _desired_state

    def run(self, blackboard):
        if self.desired_state == blackboard.game.state:
            return TaskStatus.SUCCESS, None
        return TaskStatus.FAILURE, None


class ChangeState:
    def __init__(self, name, _target_state):
        self.name = name
        self.target_state = _target_state

    def run(self, blackboard):
        blackboard.game.state = self.target_state
        return TaskStatus.FAILURE, None
