from typing import Iterable
import rospy

from strategy.behaviour import *
from strategy.actions.state_behaviours import InState
from strategy.actions.movement_behaviours import GoToGoalCenter, StopAction, AlignWithAxis, MarkBallOnYAxis
from strategy.actions.game_behaviours import IsInAttackSide
from strategy.actions.decorators import InvertOutput
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
        self.add_child(normal)

        normal.add_child(InState("CheckNormalState", GameStates.NORMAL))
        normal_actions = Selector("NormalActions")
        normal.add_child(normal_actions)

        normal_actions.add_child(OutOfGoalAction())
        # invert = InvertOutput()
        # invert.add_child(AlignWithAxis())
        # normal_actions.add_child(invert)
        # normal_actions.add_child(MarkBallOnAxis())

        normal_actions.add_child(self._ball_on_attack_side_tree())
        self.markBallOnY = None
        normal_actions.add_child(self._ball_on_defense_side_tree())

        normal_actions.add_child(AlignWithAxis())


    def _ball_on_attack_side_tree(self) -> TreeNode:
        tree = Sequence("BallInAttackSide")
        tree.add_child(IsInAttackSide("VerifyBallInAttack", lambda b: b.ball.position))
        tree.add_child(GoToGoalCenter(max_speed=40, acceptance_radius=5))
        tree.add_child(AlignWithAxis())
        return tree

    def _ball_on_defense_side_tree(self) -> TreeNode:
        tree = Sequence("BallInDeffenseSide")

        inverter = InvertOutput()
        tree.add_child(inverter)

        inverter.add_child(IsInAttackSide("VerifyBallInAttack", lambda b: b.ball.get_predicted_position_over_seconds()))

        self.markBallOnY = MarkBallOnYAxis([10, 30], [10, 90],
                                           max_speed=120,
                                           acceptance_radius=5)
        tree.add_child(self.markBallOnY)

        return tree

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        self.setYAxis(blackboard)
        return super().run(blackboard)

    def setYAxis(self, blackboard: BlackBoard) -> None:
        a = self.get_clamps(blackboard)
        self.markBallOnY.set_clamps(*a)

    def get_clamps(self, blackboard: BlackBoard) -> Tuple[Iterable, Iterable]:
        a = 75 - blackboard.home_goal.position[0]
        s = 1 if a > 0 else -1

        x = blackboard.home_goal.position[0] + s * 8
        return ([x, 40], [x, 90])


class OutOfGoalAction(Sequence):
    def __init__(self, name: str = "Get Out Of Goal"):
        super().__init__(name)
        self.add_child(IsInsideGoal("IsGoalkeeperInsideGoal"))
        self.add_child(GetOutOfGoal(max_speed=200, acceptance_radius=1.0))

    def run(self, blackboard: BlackBoard):
        status, action = super().run(blackboard)
        if status == TaskStatus.SUCCESS:
            self.children[1].target_pos = None
        return status, action
