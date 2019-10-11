from typing import Iterable

from strategy.actions.decorators import InvertOutput, DoNTimes, StatusChanged, Kee
from strategy.actions.game_behaviours import IsInAttackSide, IsInsideDefenseGoal 
from strategy.actions.game_behaviours import IsInDefenseBottomLine
from strategy.actions.movement_behaviours import GoToGoalCenter, AlignWithAxis, GetOutOfGoal
from strategy.actions.movement_behaviours import MarkBallOnYAxis
from strategy.actions.state_behaviours import InState
from strategy.base_trees import BaseTree
from strategy.behaviour import *
from strategy.strategy_utils import GameStates
from strategy.arena_utils import LEFT


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
        
        status_changed = StatusChanged(function=self.reset_counter)
        status_changed.add_child(OutOfGoalAction())
        normal_actions.add_child(status_changed)

        normal_actions.add_child(self.do_once)

        self.mark_ball_on_y = None
        self.mark_ball_on_bottom_line = None
        normal_actions.add_child(self._ball_on_defense_side_tree())
        normal_actions.add_child(self._ball_on_attack_side_tree())

    def reset_counter(self):
        self.do_once.n = 1

    def _ball_on_attack_side_tree(self) -> TreeNode:
        tree = Sequence("BallInAttackSide")
        tree.add_child(IsInAttackSide("VerifyBallInAttack", lambda b: b.ball.position))
        tree.add_child(GoToGoalCenter(max_speed=40, acceptance_radius=3))
        tree.add_child(AlignWithAxis())
        return tree

    def _ball_on_defense_side_tree(self) -> TreeNode:
        tree = Sequence("BallInDefenseSide")

        inverter = InvertOutput()
        tree.add_child(inverter)
        inverter.add_child(IsInAttackSide("VerifyBallInAttack", 
        lambda b: b.ball.get_predicted_position_over_seconds(b.ball.get_time_on_axis(axis=0,value=b.robot.position[0]))))
        
        tree.add_child(self._ball_on_bottom_tree())
        
        self.mark_ball_on_y = MarkBallOnYAxis([10, 30], [10, 90],
                                              max_speed=120,
                                              acceptance_radius=2)
        tree.add_child(self.mark_ball_on_y)

        return tree

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        self.set_y_axis(blackboard)
        self._set_bottom_line_axis(blackboard)
        return super().run(blackboard)

    def set_y_axis(self, blackboard: BlackBoard) -> None:
        a = self.get_clamps(blackboard)
        self.mark_ball_on_y.set_clamps(*a)

    def get_clamps(self, blackboard: BlackBoard) -> Tuple[Iterable, Iterable]:
        s = -1 if blackboard.home_goal.side else 1

        x = blackboard.home_goal.position[0] + s * 4
        return [x, 40], [x, 90]

    def _ball_on_bottom_tree(self) -> TreeNode:
        tree = Sequence("BallInBotttomLine")
        tree.add_child(IsInDefenseBottomLine("IsBallInBottomLine", 
                                             lambda b : b.ball.position))

        self.mark_ball_on_bottom_line = MarkBallOnYAxis([10, 30], [10, 90],
                                                        max_speed=120,
                                                        acceptance_radius=2)
        tree.add_child(self.mark_ball_on_bottom_line)
        return tree
    
    def _set_bottom_line_axis(self, blackboard: BlackBoard) -> None:
        team_side = blackboard.home_goal.side
        x = 4 if team_side == LEFT else 146
        self.mark_ball_on_bottom_line._clamp_min[0] = x
        self.mark_ball_on_bottom_line._clamp_max[0] = x


class OutOfGoalAction(Sequence):
    def __init__(self, name: str = "Get Out Of Goal"):
        super().__init__(name)
        self.add_child(IsInsideDefenseGoal("IsGoalkeeperInsideGoal", 
                                           lambda b : b.ball.position))
        self.add_child(GetOutOfGoal(max_speed=200, acceptance_radius=1.0))

    def run(self, blackboard: BlackBoard):
        status, action = super().run(blackboard)
        if status == TaskStatus.SUCCESS:
            self.children[1].target_pos = None
        return status, action
