from strategy.behaviour import TaskStatus, BlackBoard
from robot_module.movement.univector.un_field import univectorField
from robot_module.movement.definitions import OpCodes
from strategy.strategy_utils import spin_direction
from strategy.behaviour import ACTION
from utils.json_handler import JsonHandler
from utils.math_utils import predict_speed, angle_between
from abc import ABC, abstractmethod
from typing import List, Tuple
import numpy as np
from rospy import logfatal


class StopAction:

    def __init__(self, name):
        self.name = name

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        return TaskStatus.RUNNING, (OpCodes.STOP, .0, 0, .0)


class SpinTask:
    def __init__(self, name):
        self.name = name

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        return TaskStatus.RUNNING, (spin_direction(blackboard.ball_position,blackboard.position,
                                                   team_side=blackboard.team_side), 0.0, 255, .0)


class UnivectorTask(ABC):
    def __init__(self, name: str, 
                       max_speed: int = 250, 
                       acceptance_radius: float = 10.0, 
                       speed_prediction: bool = True):
        self.name = name
        self.speed = max_speed
        self.speed_prediction = speed_prediction
        self.acceptance_radius = acceptance_radius

        univector_list = JsonHandler().read("parameters/univector_constants.json")

        # univector
        RADIUS = univector_list['RADIUS']
        KR = univector_list['KR']
        K0 = univector_list['K0']
        DMIN = univector_list['DMIN']
        LDELTA = univector_list['LDELTA']

        self.univector_field = univectorField()
        self.univector_field.updateConstants(RADIUS, KR, K0, DMIN, LDELTA)

    @abstractmethod
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        raise Exception("subclass must override run method")
        pass

    def go_to_objective(self, blackboard: BlackBoard, objective_position):
        distance_to_ball = np.linalg.norm(blackboard.position - objective_position)

        if distance_to_ball < self.acceptance_radius:
            return TaskStatus.SUCCESS, None

        self.univector_field.update_attack_side(blackboard.attack_goal)

        self.univector_field.updateObstacles(blackboard.enemies_position, [[0, 0]] * 5)  # blackboard.enemies_speed)
        angle = self.univector_field.get_angle_with_ball(blackboard.position, np.array([0, 0]),  # blackboard.speed,
                                                         objective_position)
        speed = self.speed
        if self.speed_prediction:
            raio = predict_speed(blackboard.position, [np.cos(blackboard.orientation), np.sin(blackboard.orientation)],
                                 objective_position, self.univector_field.get_attack_goal())
            cte = 90
            speed = (raio * cte) ** 0.5 + 10

        status = TaskStatus.RUNNING

        return status, (OpCodes.NORMAL, angle, speed, distance_to_ball)


class GoToBallUsingUnivector(UnivectorTask):

    def __init__(self, name, max_speed: int = 250, acceptance_radius: float = 10.0, speed_prediction: bool = True):
        super().__init__(name, max_speed, acceptance_radius, speed_prediction)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        return self.go_to_objective(blackboard, blackboard.ball_position)


class GoToAttackGoalUsingUnivector(UnivectorTask):
    def __init__(self, name, max_speed: int = 250, acceptance_radius: float = 10.0, speed_prediction: bool = True):
        super().__init__(name, max_speed, acceptance_radius, speed_prediction)

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        return self.go_to_objective(blackboard, blackboard.attack_goal_pos)


class ChargeWithBall:

    def __init__(self, name='ChargeWithBall', max_speed: int = 255):
        self.name = name
        self.max_speed = max_speed

    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        goal_vector = blackboard.attack_goal_pos - blackboard.position

        angle = angle_between(
            [np.cos(blackboard.orientation), np.sin(blackboard.orientation)],
            goal_vector
        )

        distance_to_goal = np.linalg.norm(goal_vector)
        logfatal(distance_to_goal)
        return TaskStatus.RUNNING, (OpCodes.NORMAL, angle, self.max_speed, distance_to_goal)


class GoToGoalCenter(UnivectorTask):
    def __init__(self, name: str = "GoToGoalCenter", 
                       max_speed: int = 120, 
                       acceptance_radius: float = 5.0, 
                       speed_prediction: bool = False):
        super().__init__(name, max_speed, acceptance_radius, speed_prediction)
    
    def run(self, blackboard: BlackBoard) -> Tuple[TaskStatus, ACTION]:
        ret = self.go_to_objective(blackboard, blackboard.home_goal_pos)
        import rospy
        rospy.logfatal(ret)
        return ret
