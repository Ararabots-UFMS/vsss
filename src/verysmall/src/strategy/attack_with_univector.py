from strategy.behaviour import *
from strategy.actions.movement_behaviours import StopAction, GoToBallUsingUnivector, SpinTask
from strategy.actions.state_behaviours import InState
from strategy.strategy_utils import GameStates
import rospy


class AttackerWithUnivectorBT(Selector):
    """docstring for ClassName"""

    def __init__(self, name='behave'):
        super().__init__(name)

        stopped = Sequence('Stopped')
        stopped.children.append(InState('CheckStoppedState', GameStates.STOPPED))
        stopped.children.append(StopAction('Wait'))
        self.children.append(stopped)

        normal = Sequence('Normal')
        normal.children.append(InState('CheckNormalState', GameStates.NORMAL))
        normal.children.append(GoToBallUsingUnivector('FollowBall'))  # FollowBall
        normal.children.append(SpinTask('Spin'))  # Spin
        self.children.append(normal)

    def run(self, blackboard):

        result, action = super().run(blackboard)

        if result != TaskStatus.FAILURE and action is not None:
            return action
        else:
            return .0, .0, False

