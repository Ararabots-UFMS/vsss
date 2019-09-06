from strategy.behaviour import *
from strategy.base_trees import BaseTree, FreeWayAttack
from strategy.actions.state_behaviours import InState
from strategy.actions.game_behaviours import IsBallInsideCentralArea
from strategy.strategy_utils import GameStates
from strategy.actions.movement_behaviours import StopAction, GoToBallUsingUnivector, SpinTask, ChargeWithBall
from robot_module.movement.definitions import OpCodes
from strategy.actions.decorators import IgnoreFailure, IgnoreSmoothing


import rospy


class Attacker(BaseTree):

    def __init__(self, name='behave'):
        super().__init__(name)

        normal = Sequence('Normal')
        normal.add_child(InState('CheckNormalState', GameStates.NORMAL))
        normal.add_child(self.naive_go_to_ball())
        
        #Free Way Attack
        """ ignore_failure = IgnoreFailure()
        ignore_failure.add_child(FreeWayAttack())
        normal.add_child(ignore_failure)
        ignore_smoothing = IgnoreSmoothing(name="Ignore smoothing free way attack")
        ignore_smoothing.add_child(GoToBallUsingUnivector(max_speed=0))
        #normal.add_child(ignore_smoothing)
        normal.add_child(GoToBallUsingUnivector())
        #normal.add_child(SpinTask('Spin')) """

        self.add_child(normal)
        
    def naive_go_to_ball(self) -> TreeNode:
        tree = Sequence("Go ball when ball in central area")
        tree.add_child(IsBallInsideCentralArea("Check ball"))
        go_to_ball = GoToBallUsingUnivector("AttackBallInTheMiddle", 
                                            max_speed=150,
                                            acceptance_radius=7,
                                            speed_prediction=False)
        tree.add_child(go_to_ball)

        return tree