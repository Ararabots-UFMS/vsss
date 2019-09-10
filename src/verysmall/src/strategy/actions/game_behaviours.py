from typing import Tuple, Callable
from strategy.behaviour import *
from strategy.strategy_utils import behind_ball, distance_point
from strategy import arena_utils
from utils.math_utils import angle_between
import numpy as np
from math import sin
import rospy

from strategy.behaviour import BlackBoard, OpCodes, TaskStatus, Goal, EnemyTeam, HomeTeam, FriendlyRobot, MovingBody
from strategy.strategy_utils import behind_ball
from strategy.behaviour import ACTION, NO_ACTION, TreeNode
from utils.math_utils import angle_between

# TODO: extend tree node
class IsBehindBall:
    def __init__(self, name: str, distance: int):
        self.name = name
        self.distance = distance

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if behind_ball(blackboard.ball.position,
        blackboard.robot.last_know_location, blackboard.home_goal.side, 
        self.distance):
            return TaskStatus.SUCCESS, (OpCodes.INVALID, 0, 0, 0)
        else:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)


class IsTheWayFree:
    def __init__(self, name: str, free_way_distance: int):
        self.name = name
        self.free_way_distance = free_way_distance

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        enemy_goal_pos = blackboard.enemy_goal.position
        ball_pos = blackboard.ball.position
        v_ball_enemy_goal = enemy_goal_pos - ball_pos

        task_result = TaskStatus.SUCCESS, (OpCodes.INVALID, 0, 0, 0)

        for enemy_position in blackboard.enemy_team.positions:
            enemy_goal = blackboard.enemy_goal.side
            v_ball_enemy = enemy_position - blackboard.ball.position
            theta = angle_between(v_ball_enemy_goal, v_ball_enemy, abs=False)
            enemy_to_path_distance = np.linalg.norm(v_ball_enemy) * sin(theta)
            
            if arena_utils.section(enemy_position).value != enemy_goal:
                # É utilizado o x da bola, pois caso o adversário esteja entre 
                # o robô e a bola, o univector trata a situação
                ball_x = blackboard.ball.position[0]
                enemy_x = enemy_position[0]

                # Evita que o caminho seja considerado obstruído por 
                # adversários atrás do robô
                if enemy_goal:
                    is_enemy_in_way = enemy_x > ball_x
                else:
                    is_enemy_in_way = enemy_x < ball_x


                if abs(enemy_to_path_distance) <= self.free_way_distance and \
                    is_enemy_in_way :
                    task_result = TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
                    break # Interrompe o loop para o primeiro robô no caminho.
        return task_result


class IsBallInsideCentralArea(TreeNode):
    def __init__(self, name: str, width: int = 110, height: int = 90):
        super().__init__(name)
        self._width = width
        self._height = height
        self._center = np.array([150, 130]) / 2
        self._tl = self._br = None
        self.update_corners()

    def update_corners(self) -> None:
        vec = np.array([self._width, -self._height]) / 2
        self._tl = self._center - vec
        self._br = self._center + vec

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        ball_pos = blackboard.ball.position
        if self._tl[0] <= ball_pos[0] < self._br[0] and \
                self._br[1] <= ball_pos[1] < self._tl[1]:
            return TaskStatus.SUCCESS, NO_ACTION
        else:
            return TaskStatus.FAILURE, NO_ACTION


class IsInAttackSide(TreeNode):
    def __init__(self, name: str, get_pos: Callable[[BlackBoard], np.ndarray]):
        super().__init__(name)
        self._get_pos = get_pos
    
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        side = blackboard.home_goal.side
        x_left = 75*(1-side)
        x_right = 75*(2 - side)

        x_obj = self._get_pos(blackboard)[0]
        
        if x_left < x_obj < x_right:
            status = TaskStatus.SUCCESS
        else:
            status = TaskStatus.FAILURE
        
        return status, NO_ACTION

class InsideMetaRange(TreeNode):
    def __init__(self, name: str, distance: int = 25):
        super().__init__(name)
        self.distance = distance
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if distance_point(blackboard.robot.position,
        blackboard.home_goal.position) < self.distance:
            return TaskStatus.SUCCESS, (OpCodes.INVALID, 0, 0, 0)
        else:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
