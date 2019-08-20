from strategy.behaviour import *
from strategy.actions.state_behaviours import InState
from strategy.actions.movement_behaviours import GoToGoalCenter, StopAction, AlignWithAxis, MarkBallOnAxis
from strategy.base_trees import Penalty, FreeBall, BaseTree
from strategy.strategy_utils import GameStates


class GoalKeeper(BaseTree):
    def __init__(self, name: str = "behave"):
        super().__init__(name)

        normal = Sequence("Normal")
        normal.add_child(InState("CheckNormalState", GameStates.NORMAL))
        #normal.add_child(GoToGoalCenter(max_speed=120, acceptance_radius=3))
        normal.add_child(AlignWithAxis())
        normal.add_child(MarkBallOnAxis(max_speed=200, acceptance_radius=5, clamp_min=35.0, clamp_max=90.0))
        self.add_child(normal)
