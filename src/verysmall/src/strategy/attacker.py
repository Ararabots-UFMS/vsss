from strategy.actions.game_behaviours import IsBehindBall, IsRobotInsideEnemyGoalLine, IsBallInsideAreas, IsNearBall
from strategy.actions.movement_behaviours import GoToBallUsingUnivector, SpinTask, ChargeWithBall
from strategy.actions.state_behaviours import InState
from strategy.base_trees import BaseTree, FreeWayAttack
from strategy.behaviour import *
from strategy.strategy_utils import GameStates
from strategy.arena_utils import ArenaSections
from strategy.actions.decorators import SmoothBorderSpeed, CurveSmoothing

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
        # tree.add_child(IsBallInsideCentralArea("Check ball"))
        go_to_ball = GoToBallUsingUnivector("AttackBallInTheMiddle",
                                            max_speed=250,
                                            acceptance_radius=7,
                                            speed_prediction=False)
        curve_smoothing = CurveSmoothing(n_prev_angles=20)
        curve_smoothing.add_child(go_to_ball)
        tree.add_child(curve_smoothing)
        tree.add_child(SpinTask('Spin'))  # Spin

        return tree

    def ball_and_robot_in_enemy_goalline(self) -> TreeNode:
        tree = Sequence('BallAndRobotInAttackGoalLine')
        tree.add_child(IsRobotInsideEnemyGoalLine("EnemyGoalLine"))
        spin_or_dash = Selector("SpinOrDash")
        tree.add_child(spin_or_dash)

        spin_sequence = Sequence("SpinSequence")
        spin_or_dash.add_child(spin_sequence)

        dash_sequence = Sequence("DashSequence")
        spin_or_dash.add_child(dash_sequence)

        spin_sequence.add_child(
            IsBallInsideAreas(areas=[ArenaSections.LEFT_DOWN_CORNER, ArenaSections.LEFT_UP_CORNER,
                                     ArenaSections.RIGHT_DOWN_CORNER, ArenaSections.RIGHT_UP_CORNER]))
        spin_sequence.add_child(IsNearBall())
        spin_sequence.add_child(SpinTask())

        dash_sequence.add_child(IsBehindBall('IsBehindBall', 20))
        # manter distância alta para não ocorrer do robô perder a bola enquanto
        # acelera
        dash_sequence.add_child(ChargeWithBall('Attack', 200))

        return tree
