import math as mth
import rospy

from strategy.actions.game_behaviours import IsBehindBall,\
 IsRobotInsideEnemyGoalLine, IsBallInsideSections, IsNearBall,\
 CanAttackerUseMoveToPointToGuideBall, IsBallInBorder, IsBallInCriticalArea, IsRobotInsideSections

from strategy.actions.movement_behaviours import GoToBallUsingUnivector,\
     SpinTask, ChargeWithBall, GoToBallUsingMove2Point, CanUseMoveToPointSafely,\
     GoToPositionUsingUnivector, GoToPosition

from strategy.actions.state_behaviours import InState
from strategy.base_trees import BaseTree, FreeWayAttack
from strategy.behaviour import *
from strategy.strategy_utils import GameStates
from strategy.arena_utils import ArenaSections, univector_pos_section
from strategy.actions.decorators import SafeHeadOnBorder


from utils.math_utils import FORWARD, BACKWARDS, DEG2RAD
from robot_module.movement.definitions import OpCodes

class Attacker(BaseTree):

    def __init__(self, name='behave'):
        super().__init__(name)
        self._critical_position_task = None
        self._move_to_point_task = None

        normal = SafeHeadOnBorder(child=Sequence('Normal'))
        self.add_child(normal)

        normal.add_child(InState('CheckNormalState', GameStates.NORMAL))
        normal_actions = Selector('Normal Actions')
        normal.add_child(normal_actions)

        normal_actions.add_child(self._robot_inside_enemy_goal())
        normal_actions.add_child(FreeWayAttack('FreewayAttack'))
        # normal_actions.add_child(self._ball_on_border_tree())
        normal_actions.add_child(self._ball_on_critical_area_tree())
        normal_actions.add_child(self.ball_and_robot_in_enemy_goalline())
        normal_actions.add_child(self.naive_go_to_ball())
        

    def naive_go_to_ball(self) -> TreeNode:
        tree = Selector("Go ball")

        middle = Sequence("Ball out of border")
        univector_movement = GoToBallUsingUnivector("AttackBallInTheMiddle",
                                            max_speed=150,
                                            acceptance_radius=4,
                                            speed_prediction=False)

        middle.add_child(univector_movement)
        middle.add_child(SpinTask('Spin'))  # Spin

        tree.add_child(middle)

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
    
    def _ball_on_border_tree(self) -> TreeNode:
        tree = Sequence("Ball on border")
        tree.add_child(IsBallInBorder())
        tree.add_child(CanUseMoveToPointSafely())        
        tree.add_child(GoToBallUsingMove2Point("GotoBallMove2point", speed=175,acceptance_radius=4))
        tree.add_child(SpinTask('Spin'))

        return tree
    
    def _ball_on_critical_area_tree(self) -> TreeNode:
        tree = Sequence("Ball on critial area")
        tree.add_child(IsBallInCriticalArea())
        self._critical_position_task = GoToPositionUsingUnivector(max_speed=80,
                                        acceptance_radius=5)
        tree.add_child(self._critical_position_task)
        return tree


    def _robot_inside_enemy_goal(self) -> TreeNode:
        tree = Sequence("RobotInsideEnemyGoal")
        tree.add_child(IsRobotInsideSections(sections=[ArenaSections.LEFT_GOAL, 
                                                      ArenaSections.RIGHT_GOAL]))
        
        self._move_to_point_task = GoToPosition()
        tree.add_child(self._move_to_point_task)

        return tree

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        team_side = blackboard.home_goal.side
        
        shift = (-1 + 2*team_side) * 25
        self._critical_position_task.set_position(np.array([75+shift, 65]))
        
        x_pos = 15 if team_side == 0 else 135
        self._move_to_point_task.target_pos = np.array([x_pos, 65])
        
        status, action = super().run(blackboard)
        return status, action
