from strategy.behaviour import *
from strategy.strategy_utils import behind_ball


class IsBehindBall:
    def __init__(self, name, distance):
        self.name = name
        self.distance = distance

    def run(self, blackboard: BlackBoard):
        if behind_ball(blackboard.ball_position, blackboard.true_pos, blackboard.team_side, self.distance):
            return TaskStatus.SUCCESS, None
        else:
            return TaskStatus.FAILURE, None

