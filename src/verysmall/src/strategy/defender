from strategy.behaviour import *
from strategy.actions.movement_behaviours import MarkBallOnAxis, StopAction, PushToAttack, GoToBallUsingUnivector,\
    SpinTask, GoToBallUsingMove2Point
from strategy.actions.state_behaviours import InState
from strategy.actions.game_behaviours import AmIAttacking, IsBallInRangeOfDefense, IsBallInBorder, IsNearBall
from strategy.strategy_utils import GameStates
from strategy.base_trees import BaseTree
from robot_module.movement.definitions import OpCodes
import rospy


class Defender(BaseTree):

    def __init__(self, name="behave"):
        super().__init__(name)
        normal = Sequence('Normal')
        normal.add_child(InState('CheckNormalState', GameStates.NORMAL))

        defend = Selector("Defend")

        prepare_for_ball = Sequence("PrepareForBall")
        prepare_for_ball.add_child(AmIAttacking("AmIAttacking"))
        prepare_for_ball.add_child(MarkBallOnAxis("MarkBallOnAxis"))

        defend.add_child(prepare_for_ball)#3

        go_for_ball = Sequence("GoForBall")
        go_for_ball.add_child(IsBallInRangeOfDefense("IsBallInRangeOfDefense"))

        defend_border = Sequence("DefendBorder")
        defend_border.add_child(IsBallInBorder("IsBallInBorder"))

        do_spin = Sequence("DoSpin")
        do_spin.add_child(IsNearBall("IsNearBall"))
        do_spin.add_child(SpinTask("SpinTask"))
        defend_border.add_child(do_spin)

        defend_border.add_child(GoToBallUsingMove2Point("GoToBallUsingMove2Point"))
        go_for_ball.add_child(defend_border)

        go_for_ball.add_child(GoToBallUsingUnivector)

        defend.add_child(go_for_ball)#2

        wait_for_ball = Sequence("WaitForBall")
        wait_for_ball.add_child(IsBallInRangeOfDefense("IsBallInRangeOfDefense"))
        wait_for_ball.add_child(MarkBallOnAxis("MarkBallOnAxis"))

        defend.add_child(wait_for_ball)#1

        normal.add_child(defend)
        normal.add_child(StopAction("StopAction"))

