from strategy.behaviour import *
from strategy.actions.state_behaviours import InState
from strategy.actions.movement_behaviours import GoToGoalCenter, StopAction
from strategy.base_trees import Penalty, FreeBall, BaseTree
from strategy.strategy_utils import GameStates


class GoalKeeper(BaseTree):
    def __init__(self, name: str = "behave"):
        super().__init__(name)

        normal = Sequence("Normal")
        normal.add_child(InState("CheckNormalState", GameStates.NORMAL))
        normal.add_child(GoToGoalCenter(max_speed=40, acceptance_radius=3))
        self.add_child(normal)
