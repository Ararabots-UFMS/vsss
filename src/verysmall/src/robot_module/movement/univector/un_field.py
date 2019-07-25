import sys
import numpy as np

sys.path.append('../')
import math
from math import pi
from math import cos, sin, atan2

sys.path.append('../../../')
from utils.math_utils import gaussian
from strategy.strategy_utils import section, CENTER
from typing import Tuple, List

LEFT = 0
RIGHT = 1


def wrap2pi(theta: float) -> float:
    if theta > pi:
        return theta - 2 * pi
    if theta < -pi:
        return 2 * pi + theta
    else:
        return theta


class HyperbolicSpiral:

    def __init__(self, _Kr, _radius):
        self.Kr = _Kr
        self.radius = _radius

    def update_params(self, _KR: float, _RADIUS: float) -> None:
        self.Kr = _KR
        self.radius = _RADIUS

    def fi_h(self, _p: np.ndarray, radius: float = None, cw: bool = True) -> float:

        if radius is None:
            r = self.radius
        else:
            r = radius

        p = np.array(_p)
        theta = atan2(p[1], p[0])
        ro = np.linalg.norm(p)

        if ro > r:
            a = (pi / 2.0) * (2.0 - (r + self.Kr) / (ro + self.Kr))
        else:
            a = (pi / 2.0) * math.sqrt(ro / r)

        if cw:
            _theta = wrap2pi(theta + a)
        else:
            _theta = wrap2pi(theta - a)

        return atan2(sin(_theta), cos(_theta))

    def n_h(self, _p: np.ndarray, _radius: float = None, cw: bool = True) -> np.ndarray:
        p = np.array(_p)
        if _radius is None:
            radius = self.radius
        else:
            radius = _radius

        fi = self.fi_h(p, radius, cw)
        return np.array([cos(fi), sin(fi)])


class Repulsive:

    def __init__(self):
        self.origin = np.array([None, None])

    def update_origin(self, newOrigin: np.ndarray) -> None:
        self.origin = np.copy(newOrigin)

    def fi_r(self, _p, _origin: np.ndarray = None, _theta: bool = True):
        if np.all(_origin != None):
            self.update_origin(_origin)

        p = np.array(_p) - self.origin

        if _theta:
            return atan2(p[1], p[0])
        else:
            return p


class Move2Goal:

    def __init__(self, _Kr: float, _radius: float):
        self.Kr = _Kr
        self.radius = _radius
        self.hyperSpiral = HyperbolicSpiral(self.Kr, self.radius)
        self.origin = np.array([None, None])

        self.u = np.array([None, None])
        self.v = np.array([None, None])

        self.toUnivectorMatrix = None
        self.toCanonicalMatrix = None

    def update_params(self, _KR: float, _RADIUS: float) -> None:
        self.Kr = _KR
        self.radius = _RADIUS
        self.hyperSpiral.update_params(self.Kr, self.radius)

    def update_axis(self, new_origin: np.ndarray, new_u_axis: np.ndarray) -> None:
        self.origin = np.array(new_origin)
        self.u = new_u_axis
        self.build_axis()

    def build_axis(self) -> None:
        self.u /= -np.linalg.norm(self.u)
        theta = math.atan2(self.u[1], self.u[0])
        self.v = np.array([-sin(theta), cos(theta)])

        self.toCanonicalMatrix = np.array([self.u, self.v]).T
        self.toUnivectorMatrix = np.linalg.inv(self.toCanonicalMatrix)

    def fi_tuf(self, _p: np.ndarray) -> float:
        n_h = self.hyperSpiral.n_h

        p = np.array(_p) - self.origin
        r = self.radius

        p = np.dot(self.toUnivectorMatrix, p).reshape(2, )

        x, y = p
        yl = y + r
        yr = y - r

        # Parece que houve algum erro de digitacao no artigo
        # Pois quando pl e pr sao definidos dessa maneira o campo gerado
        # se parece mais com o resultado obtido no artigo
        pl = np.array([x, yr])
        pr = np.array([x, yl])

        # Este caso eh para quando o robo esta dentro do "circulo" da bola
        if -r <= y < r:
            nh_pl = n_h(pl, cw=False)
            nh_pr = n_h(pr, cw=True)

            # Apesar de no artigo nao ser utilizado o modulo, quando utilizado
            # na implementacao o resultado foi mais condizente com o artigo
            vec = (abs(yl) * nh_pl + abs(yr) * nh_pr) / (2.0 * r)
            vec = np.dot(self.toCanonicalMatrix, vec).reshape(2, )
        else:
            if y < -r:
                theta = self.hyperSpiral.fi_h(pl, cw=True)
            else:  # y >= r
                theta = self.hyperSpiral.fi_h(pr, cw=False)

            vec = np.array([cos(theta), sin(theta)])
            vec = np.dot(self.toCanonicalMatrix, vec).reshape(2, )

        return atan2(vec[1], vec[0])


class AvoidObstacle:
    def __init__(self, _pObs: np.ndarray, _vObs: np.ndarray, _pRobot: np.ndarray, _vRobot: np.ndarray, _K0: float):
        self.pObs = np.array(_pObs)
        self.vObs = np.array(_vObs)
        self.pRobot = np.array(_pRobot)
        self.vRobot = np.array(_vRobot)
        self.K0 = _K0
        self.repField = Repulsive()

    def get_s(self) -> np.ndarray:
        return self.K0 * (self.vObs - self.vRobot)

    def get_virtual_pos(self) -> np.ndarray:
        s = self.get_s()
        s_norm = np.linalg.norm(s)
        d = np.linalg.norm(self.pObs - self.pRobot)
        if d >= s_norm:
            v_pos = self.pObs + s
        else:
            v_pos = self.pObs + (d / s_norm) * s
        return v_pos

    def fi_auf(self, _robotPos: np.ndarray, _vPos: np.ndarray = (None, None), _theta: bool = True) -> np.ndarray:
        if np.all(_vPos == None):
            v_pos = self.get_virtual_pos()
        else:
            v_pos = _vPos
        vec = self.repField.fi_r(_robotPos, _origin=v_pos, _theta=_theta)
        return vec

    def update_param(self, _K0: float) -> None:
        self.K0 = _K0

    def update_obstacle(self, _pObs: np.ndarray, _vObs: np.ndarray) -> None:
        self.pObs = np.array(_pObs)
        self.vObs = np.array(_vObs)

    def update_robot(self, _pRobot: np.ndarray, _vRobot: np.ndarray) -> None:
        self.pRobot = np.array(_pRobot)
        self.vRobot = np.array(_vRobot)


class UnivectorField:
    def __init__(self):
        self.obstacles = np.array([[None, None]])
        self.obstaclesSpeed = np.array([[None, None]])
        self.ballPos = np.array([None, None])
        self.robotPos = np.array([None, None])
        self.vRobot = np.array([None, None])
        # Field constants
        self.RADIUS = None
        self.KR = None
        self.K0 = None
        self.DMIN = None
        self.LDELTA = None

        # Subfields
        self.avdObsField = AvoidObstacle([None, None], [None, None],
                                         [None, None], [None, None], self.K0)

        self.mv2Goal = Move2Goal(self.KR, self.RADIUS)

    @staticmethod
    def get_attack_goal_axis(attack_goal: bool) -> np.ndarray:
        if attack_goal == LEFT:
            return np.array([-1.0, 0.0])
        else:
            return np.array([1.0, 0.0])

    @staticmethod
    def get_attack_goal_position(attack_goal: bool) -> np.ndarray:
        """
        Return the position of the goal, given attacking side  and section of the object
        :param team_side: int
        :return: np.array([x,y])
        """
        return np.array([attack_goal * 150, 65])

    def update_obstacles(self, _obstacles: np.ndarray, _obsSpeeds: np.ndarray) -> None:
        self.obstacles = np.array(_obstacles)
        self.obstaclesSpeed = np.array(_obsSpeeds)

    def update_robot(self, _robotPos: np.ndarray, _vRobot: np.ndarray) -> None:
        self.robotPos = np.array(_robotPos)
        self.vRobot = np.array(_vRobot)
        self.avdObsField.update_robot(self.robotPos, self.vRobot)

    def update_constants(self, _RADIUS: float, _KR: float, _K0: float, _DMIN: float, _LDELTA: float) -> np.ndarray:
        self.RADIUS = _RADIUS
        self.KR = _KR
        self.K0 = _K0
        self.DMIN = _DMIN
        self.LDELTA = _LDELTA

        self.avdObsField.update_param(self.K0)
        self.mv2Goal.update_params(self.KR, self.RADIUS)

    def get_angle_vec(self, _robotPos: np.ndarray = None, _vRobot: np.ndarray = None,
                      _goal_pos: np.ndarray = None, _goal_axis=None) -> float:

        if _robotPos is not None and _vRobot is not None:
            # Just in case the user send lists
            robot_pos = np.array(_robotPos)
            v_robot = np.array(_vRobot)
            self.update_robot(robot_pos, v_robot)

        if _goal_pos is not None and _goal_axis is not None:
            goal_position = np.array(_goal_pos)
            goal_axis = np.array(_goal_axis)
            self.mv2Goal.update_axis(goal_position, goal_axis)

        closest_center = np.array([None, None])  # array to store the closest center
        centers = []
        min_distance = self.DMIN + 1

        if self.obstacles.size:
            # get the Repulsive field centers
            for i in range(self.obstacles.shape[0]):
                self.avdObsField.update_obstacle(self.obstacles[i], self.obstaclesSpeed[i])
                center = self.avdObsField.get_virtual_pos()
                centers.append(center)

            centers = np.asarray(centers)
            dist_vec = np.linalg.norm(np.subtract(centers, self.robotPos), axis=1)
            index = np.argmin(dist_vec)  # index of closest center
            closest_center = centers[index]
            min_distance = dist_vec[index]

            fi_auf = self.avdObsField.fi_auf(self.robotPos, _vPos=closest_center, _theta=True)

        # the first case when the robot is to close from an obstacle
        if min_distance <= self.DMIN:
            return fi_auf
        else:
            fi_tuf = self.mv2Goal.fi_tuf(self.robotPos)
            # Checks if at least one obstacle exist
            if self.obstacles.size:
                g = gaussian(min_distance - self.DMIN, self.LDELTA)
                # a + jb
                # c + jd
                # a*c + jad + jcb -b*d
                # a*c - b*d, j(ad+cb)
                # fi_auf *= g
                # fi_tuf *= (1.0-g)
                # v1 = np.array([cos(fi_auf), sin(fi_auf)])
                # v2 = np.array([cos(fi_tuf), sin(fi_tuf)])
                # result = np.array([v1[0]*v2[0]-v1[1]*v2[1], v1[0]*v2[1]+v2[0]*v1[1]])
                # return atan2(result[1], result[0])
                diff = wrap2pi(fi_auf - fi_tuf)
                return wrap2pi(g * diff + fi_tuf)
            else:  # if there is no obstacles
                return fi_tuf

    def get_vec_with_ball(self, _robotPos: np.ndarray = None,
                          _vRobot: np.ndarray = None,
                          _ball: np.ndarray = None,
                          _attack_goal: bool = RIGHT) -> np.ndarray:
        angle = self.get_angle_with_ball(_robotPos, _vRobot, _ball, _attack_goal)
        return np.asarray([np.cos(angle), np.sin(angle)])

    def get_angle_with_ball(self, _robotPos: np.ndarray = None,
                            _vRobot: np.ndarray = None,
                            _ball: np.ndarray = None,
                            _attack_goal: bool = RIGHT) -> float:
        return self.get_angle_vec(_robotPos, _vRobot, _ball,
                                  np.array(self.get_attack_goal_position(_attack_goal) - _ball, dtype=np.float32)
                                  )
