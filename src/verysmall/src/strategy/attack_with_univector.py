from strategy.behaviour import *
from strategy.actions.movement_behaviours import StopAction, GoToBallUsingUnivector, SpinTask
from strategy.actions.state_behaviours import InState
from strategy.strategy_utils import GameStates
from strategy.base_trees import BaseTree
from robot_module.movement.definitions import OpCodes
import rospy


class AttackerWithUnivectorBT(BaseTree):
    """docstring for ClassName"""

    def __init__(self, name='behave'):
        super().__init__(name)

        normal = Sequence('Normal')
        normal.add_child(InState('CheckNormalState', GameStates.NORMAL))
        normal.add_child(GoToBallUsingUnivector('FollowBall'))  # FollowBall
        normal.add_child(SpinTask('Spin'))  # Spin
        self.add_child(normal)