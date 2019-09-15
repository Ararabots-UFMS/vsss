from strategy.behaviour import *
from strategy.base_trees import BaseTree, FreeWayAttack
from strategy.actions.state_behaviours import InState
from strategy.actions.game_behaviours import IsBallInsideCentralArea, IsBehindBall, IsRobotInsideEnemyGoalLine
from strategy.strategy_utils import GameStates, behind_ball
from strategy.actions.movement_behaviours import StopAction, GoToBallUsingUnivector, GoToAttackGoalUsingUnivector, SpinTask, ChargeWithBall, AlignWithAxis
from robot_module.movement.definitions import OpCodes
from strategy.actions.decorators import IgnoreFailure
from strategy.arena_utils import inside_rectangle, RIGHT, LEFT

import numpy as np
import rospy

class Attacker(BaseTree):

    def __init__(self, name='behave'):
        super().__init__(name)

        normal = Sequence('Normal')
        self.add_child(normal)

        normal.add_child(InState('CheckNormalState', GameStates.NORMAL))
        normal_actions = Selector('Normal Actions')
        normal.add_child(normal_actions)
        
        normal_actions.add_child(self.ball_and_robot_in_enemy_goalline())
        normal_actions.add_child(FreeWayAttack('FreewayAttack'))
        normal_actions.add_child(self.naive_go_to_ball())
    
    def naive_go_to_ball(self) -> TreeNode:
        tree = Sequence("Go ball when ball in central area")
        #tree.add_child(IsBallInsideCentralArea("Check ball"))
        go_to_ball = GoToBallUsingUnivector("AttackBallInTheMiddle", 
                                            max_speed=150,
                                            acceptance_radius=7,
                                            speed_prediction=False)
        tree.add_child(go_to_ball)
        tree.add_child(SpinTask('Spin'))  # Spin 

        return tree
    
    def ball_and_robot_in_enemy_goalline(self) -> TreeNode:
        tree = Sequence('BallAndRobotInAttackGoalLine')
        tree.add_child(IsRobotInsideEnemyGoalLine("EnemyGoalLine"))
        tree.add_child(IsBehindBall('IsBehindBall', 20))
        #manter distância alta para não ocorrer do robô perder a bola enquanto
        # acelera
        #tree.add_child(GoToAttackGoalUsingUnivector('Attack'))       
        tree.add_child(ChargeWithBall('Attack', 200))

        return tree


