from strategy.behaviour import TaskStatus, Sequence, BlackBoard
from strategy.strategy_utils import GameStates


class InState:
    """ 
        A selector runs each task in order until one succeeds,
        at which point it returns SUCCESS. If all tasks fail, a FAILURE
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """

    def __init__(self, name, _desired_state):
        self.name = name
        self.desired_state = _desired_state

    def run(self, blackboard):
        if self.desired_state == blackboard.game_state:
            return TaskStatus.SUCCESS, None

        return TaskStatus.FAILURE, None


class PenaltyTask(Sequence):
    def __init__(self, name="Penalty"):
        super().__init__(name)
        self.children.append()

    def run(self, blackboard: BlackBoard):
        if blackboard.game_state != GameStates.PENALTY:
            return TaskStatus.SUCCESS, None

        status, action = super().run(blackboard)

        return status, action


class ChangeState:
    def __init__(self, name, _target_state):
        self.name = name
        self.target_state = _target_state

    def run(self, blackboard):
        blackboard.game_state = self.target_state
        return TaskStatus.FAILURE, None
