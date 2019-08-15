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

        self.markBallOnY = MarkBallOnYAxis() 
        normal.add_child(self.markBallOnY)
        #normal.add_child(AlignWithAxis())
        self.add_child(normal)
    
    
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        pass

