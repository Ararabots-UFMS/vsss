from strategy.behaviour import *
from strategy.strategy_utils import behind_ball
from utils.math_utils import angle_between
import numpy as np
from math import sin
import rospy


class IsBehindBall:
    def __init__(self, name: str, distance: int):
        self.name = name
        self.distance = distance

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if behind_ball(blackboard.ball_position, blackboard.true_pos, blackboard.team_side, self.distance):
            return TaskStatus.SUCCESS, (OpCodes.INVALID, 0, 0, 0)
        else:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)


class IsTheWayFree:
    def __init__(self, name: str, free_way_distance: int):
        self.name = name
        self.free_way_distance = free_way_distance

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        v_ball_enemy_goal = blackboard.attack_goal_pos - blackboard.ball_position

        task_result = TaskStatus.SUCCESS, (OpCodes.INVALID, 0, 0, 0)

        for enemy_position in blackboard.enemies_position:
            enemy_side = blackboard.attack_goal
            v_ball_enemy = enemy_position - blackboard.ball_position
            theta = angle_between(v_ball_enemy_goal, v_ball_enemy, abs=False)
            enemy_path_distance = np.linalg.norm(v_ball_enemy) * sin(theta)
            
            if enemy_side:
                if abs(enemy_path_distance) <= self.free_way_distance and enemy_position[0] > blackboard.ball_position[0]:
                    task_result = TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
                    break
            else:
                if abs(enemy_path_distance) <= self.free_way_distance and enemy_position[0] < blackboard.ball_position[0]:
                    task_result = TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
                    break

        return task_result


