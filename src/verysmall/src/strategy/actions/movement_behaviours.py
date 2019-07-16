from strategy.behaviour import TaskStatus, BlackBoard
from robot_module.movement.univector.un_field import univectorField
from robot_module.movement.functions.movement import Movement
from utils.json_handler import JsonHandler
from utils.math_utils import predict_speed
import numpy as np

class StopAction:
    """ 
        A selector runs each task in order until one succeeds,
        at which point it returns SUCCESS. If all tasks fail, a FAILURE
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """
    def __init__(self, name):
        self.name = name

    def run(self, blackboard : BlackBoard):
        return TaskStatus.SUCCESS, (.0, .0, False)


class SpinTask:
    def __init__(self, name):
        self.name = name

    def run(self, blackboard : BlackBoard):
        return TaskStatus.SUCCESS, (360, 255, False)


class GoToBallUsingUnivector:
    """docstring for GoToBallUsingUnivector"""
    def __init__(self, name, max_speed=250, acceptance_radius = 5.0, speed_prediction=True):
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

    def run(self, blackboard : BlackBoard):
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


