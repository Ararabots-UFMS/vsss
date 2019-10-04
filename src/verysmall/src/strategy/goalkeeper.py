from typing import Iterable

from strategy.actions.decorators import InvertOutput, DoNTimes, StatusChanged
from strategy.actions.game_behaviours import IsInAttackSide, IsBehindBall
from strategy.actions.game_behaviours import IsInsideGoal, IsBallInsideDefenseArea, BlackBoard
from strategy.actions.movement_behaviours import GoToGoalCenter, AlignWithAxis, GetOutOfGoal
from strategy.actions.movement_behaviours import MarkBallOnYAxis, ChargeWithBall, GoToBallUsingUnivector
from strategy.actions.state_behaviours import InState
from strategy.base_trees import BaseTree
from strategy.behaviour import *
from strategy.strategy_utils import GameStates
from strategy.arena_utils import goal_position

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

        normal_actions.add_child(self._ball_on_attack_side_tree())
        self.mark_ball_on_y = None
        normal_actions.add_child(self._ball_on_defense_side_tree())
        normal_actions.add_child(self.get_ball_out_of_def_area())

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

        inverter.add_child(IsInAttackSide("VerifyBallInAttack", lambda b: b.ball.get_predicted_position_over_seconds(b.ball.get_time_on_axis(axis=0,value=b.robot.position[0]))))

        self.mark_ball_on_y = MarkBallOnYAxis([10, 30], [10, 90],
                                              max_speed=120,
                                              acceptance_radius=2)
        tree.add_child(self.mark_ball_on_y)

        return tree

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        self.set_y_axis(blackboard)
        return super().run(blackboard)

    def set_y_axis(self, blackboard: BlackBoard) -> None:
        a = self.get_clamps(blackboard)
        self.mark_ball_on_y.set_clamps(*a)

    def get_clamps(self, blackboard: BlackBoard) -> Tuple[Iterable, Iterable]:
        s = -1 if blackboard.home_goal.side else 1

        x = blackboard.home_goal.position[0] + s * 4
        return [x, 40], [x, 90]
    
    def get_ball_out_of_def_area(self) -> TreeNode:
        tree = Sequence("GetBallOutOfDefenseArea")
        
        tree.add_child(IsBallInsideDefenseArea())
        tree.add_child(GoToBallUsingUnivector())
        tree.add_child(IsBehindBall("IsBehindBall", 5))
        tree.add_child(ChargeWithBall())
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
