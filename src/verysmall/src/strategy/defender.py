from strategy.actions.game_behaviours import IsBallInRangeOfDefense, IsBallInBorder, AmIInDefenseField, IsNearBall, \
    IsBallInCriticalPosition, CanRobotUseMove2PointToRecoverBall, IsRobotInRangeOfDefense
from strategy.actions.movement_behaviours import MarkBallOnAxis, GoToBallUsingUnivector, SpinTask, \
    GoToBallUsingMove2Point, ChargeWithBall, GoBack, RecoverBallUsingUnivector, GoToDefenseRange
from strategy.actions.state_behaviours import InState
from strategy.actions.decorators import InvertOutput
from strategy.base_trees import BaseTree
from strategy.behaviour import *
from strategy.strategy_utils import GameStates


class Defender(BaseTree):

    def __init__(self, name="behave"):
        super().__init__(name)

        normal = Sequence("Normal")
        normal.add_child(InState('CheckNormalState', GameStates.NORMAL))

        defend = Selector("Defend")

        border = Sequence("Border")
        border.add_child(IsBallInRangeOfDefense("RangeOfDefense"))
        border.add_child(IsBallInBorder("BallInBorder"))
        border.add_child(GoToBallUsingMove2Point("Move2Point", speed=120, acceptance_radius=7))
        border.add_child(SpinTask("Spin"))
        defend.add_child(border)

        middle = Sequence("Middle")
        middle.add_child(IsBallInRangeOfDefense("InRangeOfDefense"))
        middle.add_child(GoToBallUsingUnivector("UsingUnivector", acceptance_radius=5, max_speed=130,
                                                speed_prediction=False))
        middle.add_child(ChargeWithBall("ChargeWithBall"))

        defend.add_child(middle)

        recover = Sequence("Recover")
        recover.add_child(IsBallInCriticalPosition())
        recover.add_child(self.reposition_before_recovery())
        method = Selector("SelectMove2PointOrUnivector")
        recover.add_child(method)

        ball_near_goal_check = Sequence("CanRobotUseMove2PointToRecoverBall?")
        ball_near_goal_check.add_child(CanRobotUseMove2PointToRecoverBall())
        ball_near_goal_check.add_child(GoToBallUsingMove2Point("Move2Point", speed=120, acceptance_radius=4))

        method.add_child(ball_near_goal_check)
        method.add_child(RecoverBallUsingUnivector())
        recover.add_child(SpinTask("Spin"))
        defend.add_child(recover)

        mark = Sequence("Mark")

        reposition = Selector("reposition")
        kick = Sequence("Kick")
        kick.add_child(IsNearBall("nearBall"))
        kick.add_child(SpinTask("Spin"))

        reposition.add_child(kick)
        reposition.add_child(AmIInDefenseField("AmIInAttackField"))
        reposition.add_child(GoBack("GoBack", acceptance_radius=15))

        mark.add_child(reposition)

        mark.add_child(self.go_to_defender_area_then_mark())

        defend.add_child(mark)
        normal.add_child(defend)
        self.add_child(normal)

    def reposition_before_recovery(self) -> TreeNode:
        tree = Selector("RepositionBeforeRecover")
        tree.add_child(GoToDefenseRange(blackboard_key='ball', speed=120))
        tree.add_child(self.go_to_defender_area())
        return tree

    def go_to_defender_area(self) -> TreeNode:
        tree = Sequence("CheckIfRobotIsInRangeOfDefense")
        tree.add_child(InvertOutput(child=IsRobotInRangeOfDefense()))
        tree.add_child(GoToDefenseRange(blackboard_key='robot', speed=120))
        return tree

    def go_to_defender_area_then_mark(self):
        tree = Sequence("GoToDefenderAreaThenMark")
        tree.add_child(GoToDefenseRange(blackboard_key='robot', speed=120))
        tree.add_child(MarkBallOnAxis("MarkBallOnAxis", predict_ball=False))
        return tree
