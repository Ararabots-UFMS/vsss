from strategy.behaviour import *
from strategy.actions.movement_behaviours import StopAction, GoToBallUsingUnivector, SpinTask
from strategy.actions.state_behaviours import InState
from strategy.strategy_utils import GameStates
from strategy.base_trees import FreeBall, Penalty
from robot_module.movement.definitions import OpCodes
import rospy


class AttackerWithUnivectorBT(Selector):
    """docstring for ClassName"""

    def __init__(self, name='behave'):
        super().__init__(name)

        stopped = Sequence('Stopped')
        stopped.add_child(InState('CheckStoppedState', GameStates.STOPPED))
        stopped.add_child(StopAction('Wait'))
        self.add_child(stopped)

        self.add_child(Penalty())
        self.add_child(FreeBall())

        normal = Sequence('Normal')
        normal.add_child(InState('CheckNormalState', GameStates.NORMAL))
        normal.add_child(GoToBallUsingUnivector('FollowBall'))  # FollowBall
        normal.add_child(SpinTask('Spin'))  # Spin
        self.add_child(normal)