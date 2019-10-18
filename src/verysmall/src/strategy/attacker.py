from strategy.actions.game_behaviours import IsBehindBall, IsRobotInsideEnemyGoalLine, IsBallInsideAreas, IsNearBall, \
    IsBallInBorder
from strategy.actions.movement_behaviours import GoToBallUsingUnivector, SpinTask, ChargeWithBall, \
    GoToBallUsingMove2Point
from strategy.actions.state_behaviours import InState
from strategy.base_trees import BaseTree, FreeWayAttack
from strategy.behaviour import *
from strategy.strategy_utils import GameStates
from strategy.arena_utils import ArenaSections


class Attacker(BaseTree):

    def __init__(self, name='behave'):
        super().__init__(name)

        normal = Sequence('Normal')
        self.add_child(normal)

        normal.add_child(InState('CheckNormalState', GameStates.NORMAL))
        normal_actions = Selector('Normal Actions')
        normal.add_child(normal_actions)

        normal_actions.add_child(self.ball_and_robot_in_enemy_goalline())
        normal_actions.add_child(self.naive_go_to_ball())
        normal_actions.add_child(FreeWayAttack('FreewayAttack'))

    def naive_go_to_ball(self) -> TreeNode:
        tree = Selector("Go ball when ball in central area")
        # tree.add_child(IsBallInsideCentralArea("Check ball"))

        border = Sequence("Ball on border")
        move_to_point_movement = GoToBallUsingMove2Point("GotoBallMove2point", acceptance_radius=7)
        border.add_child(IsBallInBorder())
        border.add_child(move_to_point_movement)
        border.add_child(SpinTask('Spin'))

        middle = Sequence("Ball out of border")
        univector_movement = GoToBallUsingUnivector("AttackBallInTheMiddle",
                                            max_speed=150,
                                            acceptance_radius=7,
                                            speed_prediction=False)

        middle.add_child(univector_movement)
        middle.add_child(SpinTask('Spin'))  # Spin

        tree.add_child(border)
        tree.add_child(middle)

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
