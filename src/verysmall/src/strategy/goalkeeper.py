from strategy.behaviour import *
from strategy.actions.state_behaviours import InState
from strategy.actions.movement_behaviours import GoToGoalCenter, StopAction, AlignWithAxis, GoToPosition, GoToGoalAreaCenter
from strategy.base_trees import Penalty, FreeBall, BaseTree
from strategy.strategy_utils import GameStates
from strategy.actions.game_behaviours import IsInsideGoal


class GoalKeeper(BaseTree):
    def __init__(self, name: str = "behave"):
        super().__init__(name)

        normal = Sequence("Normal")
        normal.add_child(InState("CheckNormalState", GameStates.NORMAL))
        get_out_of_goal = Selector("Get Out Of Goal")
        get_out_of_goal.add_child(IsInsideGoal("IsGoalkeeperInsideGoal"))
        move = Sequence("Move")
        move.add_child(GoToGoalCenter(acceptance_radius=5.0))
        move.add_child(GoToGoalAreaCenter(acceptance_radius=5.0))
        get_out_of_goal.add_child(move)
        normal.add_child(get_out_of_goal)
        #normal.add_child(GoToGoalCenter(max_speed=120, acceptance_radius=3))
        normal.add_child(AlignWithAxis())
        self.add_child(normal)