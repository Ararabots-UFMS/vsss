import rospy

from strategy.behaviour import *
from strategy.actions.movement_behaviours import MarkBallOnAxis, StopAction, PushToAttack, GoToBallUsingUnivector,\
    SpinTask, GoToBallUsingMove2Point
from strategy.actions.state_behaviours import InState
from strategy.actions.game_behaviours import AmIAttacking, IsBallInRangeOfDefense, IsBallInBorder, IsNearBall
from strategy.strategy_utils import GameStates
from strategy.base_trees import BaseTree


class Defender(BaseTree):

    def __init__(self, name="behave"):
        super().__init__(name)

        normal = Sequence("Normal")
        normal.add_child(InState('CheckNormalState', GameStates.NORMAL))

        defend = Selector("Defend")

        border = Sequence("Border")
        border.add_child(IsBallInBorder("BallInBorder"))
        border.add_child(GoToBallUsingMove2Point("Move2Point"))
        border.add_child(SpinTask("Spin"))
        defend.add_child(border)

        middle = Sequence("Middle")
        middle.add_child(IsBallInRangeOfDefense("InRangeOfDefense"))
        middle.add_child(GoToBallUsingUnivector("UsingUnivector"))

        defend.add_child(middle)
        normal.add_child(defend)

        mark = Sequence("Mark")
        mark.add_child(MarkBallOnAxis("MarkBall"))

        normal.add_child(mark)
        self.add_child(normal)
