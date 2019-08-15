from typing import Iterable
from strategy.behaviour import *
from strategy.actions.state_behaviours import InState
from strategy.actions.movement_behaviours import GoToGoalCenter, StopAction, AlignWithAxis, MarkBallOnYAxis
from strategy.base_trees import Penalty, FreeBall, BaseTree
from strategy.strategy_utils import GameStates


class GoalKeeper(BaseTree):
    def __init__(self, name: str = "behave"):
        super().__init__(name)

        normal = Sequence("Normal")
        self.add_child(normal)

        normal.add_child(InState("CheckNormalState", GameStates.NORMAL))
        normal.add_child(GoToGoalCenter(max_speed=120, acceptance_radius=3))

        self.markBallOnY = MarkBallOnYAxis([10, 30], [10, 90], acceptance_radius=10) 
        normal.add_child(self.markBallOnY)
    
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        self.setYAxis(blackboard)
        super().run(blackboard)
    
    def setYAxis(self, blackboard: BlackBoard) -> None:
        self.markBallOnY.set_clamps(*self.get_clamps(blackboard))
    
    def get_clamps(self, blackboard: BlackBoard) -> Tuple[Iterable, Iterable]:
        a = 75 - blackboard.home_goal_pos[0]
        s = 1 if a > 0 else -1

        x = blackboard.home_goal_pos + s*10
        return [x, 30], [x, 90]


