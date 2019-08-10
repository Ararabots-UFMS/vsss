from strategy.behaviour import *
from strategy.strategy_utils import behind_ball, ball_on_critical_position, ball_on_attack_side, ball_on_border,\
    near_ball


class IsBehindBall:
    def __init__(self, name, distance):
        self.name = name
        self.distance = distance

    def run(self, blackboard: BlackBoard):
        if behind_ball(blackboard.ball_position, blackboard.true_pos, blackboard.team_side, self.distance):
            return TaskStatus.SUCCESS, None
        else:
            return TaskStatus.FAILURE, None


class AmIAttacking(TreeNode):
    def __init__(self, name: str = "IsEnemyAttacking"):
        self.name = name

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if ball_on_attack_side(blackboard):
            return TaskStatus.SUCCESS, None

        return TaskStatus.FAILURE, None


class IsBallInRangeOfDefense(TreeNode):
    def __init__(self, name: str = "IsBallInRangeOfDefense"):
        self.name = name

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if ball_on_critical_position(blackboard):
            return TaskStatus.SUCCESS, None

        return TaskStatus.FAILURE, None


class IsBallInBorder(TreeNode):
    def __init__(self, name: str = "IsballInBorder"):
        self.name = name

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if ball_on_border(blackboard):
            return TaskStatus.SUCCESS, None

        return TaskStatus.FAILURE, None


class IsNearBall(TreeNode):
    def __init__(self, name: str = "IsNearBall"):
        self.name = name

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if near_ball(blackboard.position, blackboard.ball_position):
            return TaskStatus.SUCCESS, None

        return TaskStatus.FAILURE, None