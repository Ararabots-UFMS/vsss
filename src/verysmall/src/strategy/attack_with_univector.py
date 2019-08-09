from strategy.behaviour import *
from strategy.actions.movement_behaviours import StopAction, GoToBallUsingUnivector, SpinTask
from strategy.actions.state_behaviours import InState
from strategy.strategy_utils import GameStates
from strategy.base_trees import BaseTree, FreeWayAttack
from robot_module.movement.definitions import OpCodes
from strategy.actions.decorators import IgnoreFailure
import rospy


class AttackerWithUnivectorBT(BaseTree):
    """docstring for ClassName"""

    def __init__(self, name='behave'):
        super().__init__(name)

        normal = Sequence('Normal')
        normal.add_child(InState('CheckNormalState', GameStates.NORMAL))
        ignore_failure = IgnoreFailure()
        ignore_failure.add_child(FreeWayAttack())
        normal.add_child(ignore_failure)
        normal.add_child(GoToBallUsingUnivector('FollowBall'))  # FollowBall
        normal.add_child(SpinTask('Spin'))  # Spin
        self.add_child(normal)