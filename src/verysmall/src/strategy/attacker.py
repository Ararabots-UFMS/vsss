import math as mth
import rospy

from strategy.actions.game_behaviours import IsBehindBall,\
 IsRobotInsideEnemyGoalLine, IsBallInsideSections, IsNearBall,\
 CanRobotUseMove2PointToRecoverBall

from strategy.actions.movement_behaviours import GoToBallUsingUnivector,\
     SpinTask, ChargeWithBall, GoToBallUsingMove2Point
from strategy.actions.state_behaviours import InState
from strategy.base_trees import BaseTree, FreeWayAttack
from strategy.behaviour import *
from strategy.strategy_utils import GameStates
from strategy.arena_utils import ArenaSections, section, univector_pos_section
from strategy.actions.decorators import SafeHeadOnBorder


from utils.math_utils import FORWARD, BACKWARDS, DEG2RAD
from robot_module.movement.definitions import OpCodes

class Attacker(BaseTree):

    def __init__(self, name='behave'):
        super().__init__(name)

        normal = SafeHeadOnBorder(child=Sequence('Normal'))
        self.add_child(normal)

        normal.add_child(InState('CheckNormalState', GameStates.NORMAL))
        normal_actions = Selector('Normal Actions')
        normal.add_child(normal_actions)

        # normal_actions.add_child(self.ball_on_border_tree())
        normal_actions.add_child(self.ball_and_robot_in_enemy_goalline())
        normal_actions.add_child(FreeWayAttack('FreewayAttack'))
        normal_actions.add_child(self.naive_go_to_ball())

    def naive_go_to_ball(self) -> TreeNode:
        tree = Sequence("Go ball when ball in central area")
        # tree.add_child(IsBallInsideCentralArea("Check ball"))
        go_to_ball = GoToBallUsingUnivector("AttackBallInTheMiddle",
                                            max_speed=120,
                                            acceptance_radius=7,
                                            speed_prediction=False)
        tree.add_child(go_to_ball)
        tree.add_child(SpinTask('Spin'))  # Spin

        return tree

    def ball_and_robot_in_enemy_goalline(self) -> TreeNode:
        tree = Sequence("BallAndRobotInAttackGoalLine")
        tree.add_child(IsRobotInsideEnemyGoalLine("EnemyGoalLine"))
        spin_or_dash = Selector("SpinOrDash")
        tree.add_child(spin_or_dash)

        spin_sequence = Sequence("SpinSequence")
        spin_or_dash.add_child(spin_sequence)

        dash_sequence = Sequence("DashSequence")
        spin_or_dash.add_child(dash_sequence)

        spin_sequence.add_child(
            IsBallInsideSections(sections=[ArenaSections.LEFT_DOWN_CORNER, 
                                           ArenaSections.LEFT_UP_CORNER,
                                           ArenaSections.RIGHT_DOWN_CORNER,
                                           ArenaSections.RIGHT_UP_CORNER]))
        spin_sequence.add_child(IsNearBall())
        spin_sequence.add_child(SpinTask())

        dash_sequence.add_child(IsBehindBall("IsBehindBall", 20))
        dash_sequence.add_child(ChargeWithBall("Attack", 200))

        return tree
    
    # def ball_on_border_tree(self) -> TreeNode:
    #     tree = Sequence("BallOnBorder")

    #     s = [ArenaSections.UP_BORDER, ArenaSections.DOWN_BORDER]
    #     tree.add_child(IsBallInsideSections(sections=s))

    #     on_boarder_behaviours = Selector("BallOnBorderBehaviours")
    #     tree.add_child(on_boarder_behaviours)

    #     on_boarder_behaviours.add_child(Sequence("GetBallUsingMove2Point",
    #         [CanRobotUseMove2PointToRecoverBall(), 
    #          GoToBallUsingMove2Point(speed=150, acceptance_radius=4)]))

    #     return tree

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        status, action = super().run(blackboard)

        if univector_pos_section(blackboard.robot.position) == ArenaSections.CENTER:
            action = list(action)
            action[0] += OpCodes.USE_FORWARD_HEAD if blackboard.current_orientation else OpCodes.USE_BACKWARD_HEAD

        return status, action
