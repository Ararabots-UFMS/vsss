from typing import Iterable

from strategy.actions.decorators import InvertOutput, DoNTimes, StatusChanged, KeepRunning
from strategy.actions.game_behaviours import IsInAttackSide, IsBehindBall, IsInsideGoal, IsBallInsideAreas, \
    IsEnemyInsideAreas, IsEnemyNearBall, IsEnemyInCriticalPosition
from strategy.actions.movement_behaviours import GoToGoalCenter, AlignWithAxis, GetOutOfGoal, GoToBallUsingMove2Point, \
    MarkBallOnYAxis, RemoveBallFromGoalArea, SpinTask
from strategy.actions.state_behaviours import InState
from strategy.arena_utils import ArenaSections
from strategy.base_trees import BaseTree
from strategy.behaviour import *
from strategy.strategy_utils import GameStates


class GoalKeeper(BaseTree):
    def __init__(self, name: str = "behave"):
        super().__init__(name)

        normal = Sequence("Normal")
        self.add_child(normal)

        normal.add_child(InState("CheckNormalState", GameStates.NORMAL))
        normal_actions = Selector("NormalActions")
        normal.add_child(normal_actions)

        self.do_once = DoNTimes(n=1)
        self.do_once.add_child(AlignWithAxis())

        normal_actions.add_child(self.get_ball_out_of_def_area())

        status_changed = StatusChanged(function=self.reset_counter)
        status_changed.add_child(OutOfGoalAction())
        normal_actions.add_child(status_changed)

        normal_actions.add_child(self.do_once)
        normal_actions.add_child(self._ball_on_attack_side_tree())
        self.mark_ball_on_y = None
        normal_actions.add_child(self._ball_on_defense_side_tree())

    def reset_counter(self):
        self.do_once.n = 1

    def _ball_on_attack_side_tree(self) -> TreeNode:
        tree = Sequence("BallInAttackSide")
        tree.add_child(IsInAttackSide("VerifyBallInAttack", lambda b: b.ball.position))
        tree.add_child(GoToGoalCenter(max_speed=40, acceptance_radius=3))
        keep_align_action = KeepRunning()
        keep_align_action.add_child(AlignWithAxis())
        tree.add_child(keep_align_action)
        return tree

    def _ball_on_defense_side_tree(self) -> TreeNode:
        tree = Sequence("BallInDefenseSide")

        inverter = InvertOutput()
        tree.add_child(inverter)

        inverter.add_child(IsInAttackSide("VerifyBallInAttack", lambda b: b.ball.get_predicted_position_over_seconds(
            b.ball.get_time_on_axis(axis=0, value=b.robot.position[0]))))

        self.mark_ball_on_y = MarkBallOnYAxis([5, 40], [5, 80],
                                              max_speed=120,
                                              acceptance_radius=4)
        tree.add_child(self.mark_ball_on_y)
        keep_align_action = KeepRunning()
        keep_align_action.add_child(AlignWithAxis())
        tree.add_child(keep_align_action)

        return tree

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        self.set_y_axis(blackboard)
        return super().run(blackboard)

    def set_y_axis(self, blackboard: BlackBoard) -> None:
        a = self.get_clamps(blackboard)
        self.mark_ball_on_y.set_clamps(*a)

    def get_clamps(self, blackboard: BlackBoard) -> Tuple[Iterable, Iterable]:
        s = -1 if blackboard.home_goal.side else 1

        x = blackboard.home_goal.position[0] + s * 5
        return [x, 40], [x, 80]

    def get_ball_out_of_def_area(self) -> TreeNode:
        tree = Sequence("GetBallOutOfDefenseArea")

        inverter = InvertOutput()
        tree.add_child(inverter)
        inverter.add_child(IsInAttackSide("VerifyBallInAttack", lambda b: b.ball.position))

        is_ball_or_enemy_in_critical_position = Selector("IsBallOrEnemyInCritalPosition")
        is_ball_inside_defense_area = IsBallInsideAreas(name="IsBallInsideDefenseArea",
                                                        areas=[ArenaSections.LEFT_GOAL_AREA,
                                                               ArenaSections.RIGHT_GOAL_AREA])

        enemy_near_ball_and_in_critical_position = Sequence('EnemyNearBallAndInCriticalPos')
        enemy_near_ball_and_in_critical_position.add_child(IsEnemyInCriticalPosition())
        enemy_near_ball_and_in_critical_position.add_child(IsEnemyNearBall())

        is_ball_or_enemy_in_critical_position.add_child(is_ball_inside_defense_area)
        is_ball_or_enemy_in_critical_position.add_child(enemy_near_ball_and_in_critical_position)
        tree.add_child(is_ball_or_enemy_in_critical_position)
        tree.add_child(GoToBallUsingMove2Point(acceptance_radius=5))

        push_or_spin = Selector("PushOrSpin")
        push_action = Sequence("PushAction")
        push_action.add_child(IsBehindBall("IsBehindBall", 10))
        push_action.add_child(RemoveBallFromGoalArea())

        push_or_spin.add_child(push_action)
        push_or_spin.add_child(SpinTask())

        tree.add_child(push_or_spin)

        return tree


class OutOfGoalAction(Sequence):
    def __init__(self, name: str = "Get Out Of Goal"):
        super().__init__(name)
        self.add_child(IsInsideGoal("IsGoalkeeperInsideGoal"))
        self.add_child(GetOutOfGoal(max_speed=200, acceptance_radius=1.0))

    def run(self, blackboard: BlackBoard):
        status, action = super().run(blackboard)
        if status == TaskStatus.SUCCESS:
            self.children[1].target_pos = None
        return status, action
