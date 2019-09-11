from strategy.behaviour import *
from strategy.actions.state_behaviours import InState
from strategy.actions.movement_behaviours import GoToGoalCenter, StopAction, AlignWithAxis, GoToPosition, GetOutOfGoal, \
    MarkBallOnAxis
from strategy.base_trees import Penalty, FreeBall, BaseTree
from strategy.strategy_utils import GameStates
from strategy.actions.game_behaviours import IsInsideGoal
from strategy.actions.decorators import IgnoreSmoothing


class GoalKeeper(BaseTree):
    def __init__(self, name: str = "behave"):
        super().__init__(name)

        normal = Sequence("Normal")
        normal.add_child(InState("CheckNormalState", GameStates.NORMAL))
        normal.add_child(OutOfGoalAction())
        normal.add_child(AlignWithAxis())
        normal.add_child(MarkBallOnAxis())

        self.add_child(normal)


class OutOfGoalAction(Selector):
    def __init__(self, name: str = "Get Out Of Goal"):
        super().__init__(name)
        self.add_child(IsInsideGoal("IsGoalkeeperInsideGoal"))
        self.add_child(GetOutOfGoal(max_speed=200, acceptance_radius=1.0))

    def run(self, blackboard: BlackBoard):
        status, action = super().run(blackboard)
        if status == TaskStatus.SUCCESS:
            self.children[1].target_pos = None
        return status, action
