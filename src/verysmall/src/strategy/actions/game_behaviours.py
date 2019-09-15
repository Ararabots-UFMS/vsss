from typing import Tuple, Callable
import numpy as np
from math import sin
import rospy


from strategy.arena_utils import on_attack_side
from strategy.behaviour import *
from strategy.strategy_utils import behind_ball, distance_point
from strategy import arena_utils
from strategy.arena_utils import RIGHT, LEFT
from strategy.strategy_utils import behind_ball, near_ball, ball_on_border, ball_on_critical_position, ball_on_attack_side
from utils.math_utils import angle_between
from strategy.behaviour import BlackBoard, OpCodes, TaskStatus, Goal, EnemyTeam, HomeTeam, FriendlyRobot, MovingBody
from strategy.behaviour import ACTION, NO_ACTION, TreeNode


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
                    break  # Interrompe o loop para o primeiro robô no caminho.
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
        x_obj = self._get_pos(blackboard)[0]
        
        if (x_obj > 75 and side == RIGHT) or (x_obj < 75 and side == LEFT):
            status = TaskStatus.SUCCESS
        else:
            status = TaskStatus.FAILURE
        
        return status, NO_ACTION

class AmIAttacking(TreeNode):
    def __init__(self, name: str = "AmIAttacking"):
        self.name = name

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:

        if ball_on_attack_side(blackboard.ball.position, blackboard.home_goal.side):
            return TaskStatus.SUCCESS, (OpCodes.INVALID, 0, 0, 0)
        return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)


class IsBallInRangeOfDefense(TreeNode):
    def __init__(self, name: str = "IsBallInRangeOfDefense"):
        self.name = name

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if not ball_on_attack_side(blackboard.ball.position, blackboard.home_goal.side) and not \
                ball_on_critical_position(blackboard.ball.position):
            return TaskStatus.SUCCESS, (OpCodes.INVALID, 0, 0, 0)
        return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)


class IsBallInBorder(TreeNode):
    def __init__(self, name: str = "IsBallInBorder"):
        self.name = name

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:

        if ball_on_border(blackboard.ball.position, blackboard.home_goal.side):
            return TaskStatus.SUCCESS, (OpCodes.INVALID, 0, 0, 0)
        return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)


class AmIInDefenseField(TreeNode):
    def __init__(self, name: str = "AmIInDefenseField"):
        self.name = name

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if not on_attack_side(blackboard.robot.position, blackboard.home_goal.side):
            return TaskStatus.SUCCESS, (OpCodes.INVALID, 0, 0, 0)

        return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)


class IsNearBall:
    def __init__(self, name: str = "IsNearBall", distance=6.):
        self.name = name
        self.distance = distance

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if near_ball(blackboard.ball.position, blackboard.robot.position, self.distance):
            return TaskStatus.SUCCESS, (OpCodes.INVALID, 0, 0, 0)
        else:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)


class IsInsideMetaRange(TreeNode):
    def __init__(self, name: str, distance: int = 25):
        super().__init__(name)
        self.distance = distance
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        if distance_point(blackboard.robot.position,
                          blackboard.home_goal.position) < self.distance:
            return TaskStatus.SUCCESS, (OpCodes.INVALID, 0, 0, 0)
        else:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)


class IsInsideGoal(TreeNode):
    def __init__(self, name: str = "IsInsideGoal"):
        super().__init__(name)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        robot_pos = blackboard.robot.position
        team_goal_side = blackboard.home_goal.side
        # Sinal do shift: 1 se o team_goal_side é LEFT e -1 se é RIGHT
        sign = 1 if team_goal_side else -1

        shift = sign * 3
        # Posição do robô deslocada no eixo X em direção ao gol
        shifted_robot_pos = np.array([robot_pos[0] + shift, robot_pos[1]])
        section = arena_utils.section(shifted_robot_pos).value
        """
        Os enums LEFT e RIGHT tem valores 0 e 1. Como os enums LEFT_GOAL e 
        RIGHT_GOAL tem 2 e 3, respectivamente, a subtração do gol da seção do 
        campo na qual o robô se encontra garante que este está dentro do gol 
        aliado
        """
        if section - team_goal_side == 2: 
            return TaskStatus.SUCCESS, (OpCodes.INVALID, 0, 0, 0)
        else:
            return TaskStatus.FAILURE, (OpCodes.INVALID, 0, 0, 0)
