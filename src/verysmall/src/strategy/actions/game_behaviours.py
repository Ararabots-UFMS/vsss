from strategy.behaviour import *
from strategy.strategy_utils import behind_ball
from strategy.base_trees import ACTION, NO_ACTION, TreeNode, TaskStatus
import numpy as np
from typing import Tuple

class IsBehindBall:
    def __init__(self, name, distance):
        self.name = name
        self.distance = distance

    def run(self, blackboard: BlackBoard):
        if behind_ball(blackboard.ball_position, blackboard.true_pos, blackboard.team_side, self.distance):
            return TaskStatus.SUCCESS, None
        else:
            return TaskStatus.FAILURE, None


class IsBallInsideCentralArea(TreeNode):
    def __init__(self, name: str, width:int = 110, height: int = 90):
        super().__init__(name)
        self._width = width
        self._height = height
        self._center = np.array(150, 130) / 2
        self._tl = self._br = None
        self.update_corners()
    
    def update_corners(self) -> None:
        vec = np.array(self._width, -self._height) / 2
        self._tl = self._center - vec
        self._br = self._center + vec
    
    def run(self, ball_pos: np.ndarray) -> Tuple[TaskStatus, ACTION]:
        if self._tl[0] <= ball_pos[0] < self._br[0] and \
           self._br[1] <= ball_pos[1] < self._tl[1]:
           return TaskStatus.SUCCESS, NO_ACTION
        else:
            return TaskStatus.FAILURE, NO_ACTION
