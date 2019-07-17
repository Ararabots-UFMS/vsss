from strategy.behaviour import TaskStatus, BlackBoard
from robot_module.movement.univector.un_field import univectorField
from robot_module.movement.functions.movement import Movement
from utils.json_handler import JsonHandler
from utils.math_utils import predict_speed
import numpy as np


class StopAction:

    def __init__(self, name):
        self.name = name

    def run(self, blackboard: BlackBoard):
        return TaskStatus.SUCCESS, (.0, .0, False)


class SpinTask:
    def __init__(self, name):
        self.name = name

    def run(self, blackboard: BlackBoard):
        return TaskStatus.RUNNING, (360, 255, False)


class GoToBallUsingUnivector:

    def __init__(self, name, max_speed=250, acceptance_radius=10.0, speed_prediction=True):
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

    def run(self, blackboard: BlackBoard):
        if np.linalg.norm(blackboard.position - blackboard.ball_position) < self.acceptance_radius:
            return TaskStatus.SUCCESS, None

        self.univector_field.updateObstacles(blackboard.enemies_position, [[0, 0]] * 5)  # blackboard.enemies_speed)
        vector = self.univector_field.getVecWithBall(blackboard.position, np.array([0, 0]),  # blackboard.speed,
                                                     blackboard.ball_position)
        speed = self.speed
        if self.speed_prediction:
            raio = predict_speed(blackboard.position, [np.cos(blackboard.orientation), np.sin(blackboard.orientation)],
                                 blackboard.ball_position, self.univector_field.get_attack_goal())
            cte = 90
            speed = (raio * cte) ** 0.5 + 10

        status = TaskStatus.RUNNING

        return status, (vector, speed, False)


class ChargeWithBall:

    def __init__(self, name='ChargeWithBall'):
        self.name = name

    def run(self, blackboard:BlackBoard):

        param_1, param_2, _ = self.state_machine.movement.move_to_point(
            220, np.array(self.position),
            [np.cos(self.orientation), np.sin(self.orientation)],
            np.array([(not self.team_side) * 150, 65]))

        return param_1, param_2, SOFTWARE  # 0.0, 250, HARDWARE
