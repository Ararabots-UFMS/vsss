from strategy.behaviour import *
from strategy.base_trees import BaseTree
from strategy.actions.state_behaviours import InState
from strategy.actions.game_behaviours import IsBallInsideCentralArea
from strategy.strategy_utils import GameStates
from strategy.actions.movement_behaviours import StopAction, GoToBallUsingUnivector, SpinTask
from robot_module.movement.definitions import OpCodes

import rospy


class Attacker(BaseTree):

    def __init__(self, name='behave'):
        super().__init__(name)

        normal = Sequence('Normal')
        normal.add_child(InState('CheckNormalState', GameStates.NORMAL))
        
        normal.add_child(GoToBallUsingUnivector('FollowBall'))  # FollowBall
        normal.add_child(SpinTask('Spin'))  # Spin
        self.add_child(normal)
    
    def naive_go_to_ball(self) -> TreeNode:
        tree = Sequence("Go ball when ball in central area")
        tree.add_child(IsBallInsideCentralArea("Check ball"))
        go_to_ball = GoToBallUsingUnivector("AttackBallInTheMiddle", 
                                            max_speed=180,
                                            acceptance_radius=2,
                                            speed_prediction=False)
        tree.add_child(GoToBallUsingUnivector(go_to_ball)

        return tree