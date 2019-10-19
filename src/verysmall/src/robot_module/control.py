import math
from bisect import bisect_left
from time import time
from typing import Tuple, List
import numpy as np
from rospy import logfatal
import utils.math_utils as mth
from robot_module.PID import PIDController
from robot_module.hardware import RobotHardware
from robot_module.movement.definitions import OpCodes
from strategy.behaviour import BlackBoard
from utils.math_utils import RAD2DEG, DEG2RAD, FORWARD, BACKWARDS
from strategy.arena_utils import ArenaSections, univector_pos_section

Constants = Tuple[int, float, float, float]


class Control:
    def __init__(self, hardware: RobotHardware, blackboard: BlackBoard,
                 constants: List[Constants],
                 max_fine_movement_speed,
                 error_tolerance=3*DEG2RAD) -> None:
        self._hardware = hardware
        self._blackboard = blackboard
        self._max_fine_movement_speed = max_fine_movement_speed
        self._alpha = 10  # centimeters

        self._head = FORWARD
        self._hysteresis_angle_window = 20 * DEG2RAD
        self._upper_angle_tol = math.pi / 2.0 + self._hysteresis_angle_window
        self._lower_angle_tol = math.pi / 2.0 - self._hysteresis_angle_window
        self._beta = 0.5
        self._t = 1
        self._current_orientation = 0
        self._ma_orientation_vec = np.array([0, 0])
        self._ma_orientation = 0

        self._pid_constants_set = sorted(constants)
        self._speed_keys = [s[0] for s in self._pid_constants_set]  # TODO: get a better name
        self._pidController = PIDController(tolerance=error_tolerance)
        self._pid_last_use = time()
        self._pid_reset_time = 0.032  # 2 frames

    def __setattr__(self, key, value):
        if key == '_head':
            self._blackboard.current_orientation = value

        super().__setattr__(key, value)

    def get_wheels_speeds(self, opcode: OpCodes,
                          angle: float,
                          speed: int,
                          distance: float) -> Tuple[float, float]:

        if opcode & OpCodes.USE_FORWARD_HEAD:
            self._head = FORWARD
        elif opcode & OpCodes.USE_BACKWARD_HEAD:
            self._head = BACKWARDS
        else:
            self.set_head(angle)
        
        if opcode & OpCodes.ORIENTATION_AVERAGE:
            self._current_orientation = self._ma_orientation
        else:
            self._current_orientation = self._blackboard.robot.orientation

        if opcode & OpCodes.SMOOTH:
            return self._follow_vector(speed, angle, distance)
        elif opcode & OpCodes.NORMAL:
            return self._follow_vector(speed, angle, distance, optimal_speed=False)
        elif opcode & OpCodes.SPIN_CCW:
            return -255, 255
        elif opcode & OpCodes.SPIN_CW:
            return 255, -255
        else:
            return 0, 0

    def _follow_vector(self, speed: int,
                       angle: float,
                       distance: float,
                       optimal_speed: bool = True) -> Tuple[float, float]:

        diff_angle = self.get_diff_angle(angle)

        if optimal_speed:
            speed = self.get_optimal_speed(speed, diff_angle, distance)
            speed = min(speed, self._hardware.get_next_speed())

        speed = min(speed, self._hardware.get_next_speed())

        t = time()
        if t - self._pid_last_use > self._pid_reset_time:
            self._pidController.reset()
        self._pid_last_use = t

        constants = self.interpolate_constants(speed)

        self._pidController.set_constants(*constants)
        correction = self._pidController.update(diff_angle)

        if self._head == FORWARD:
            return speed + correction, speed - correction
        else:
            return -speed + correction, -speed - correction

    def set_head(self, angle: float) -> np.array:
        abs_diff = abs(mth.min_angle(self._current_orientation, angle))
        if abs_diff > self._upper_angle_tol:
            self._head = BACKWARDS
        elif abs_diff < self._lower_angle_tol:
            self._head = FORWARD

    def get_diff_angle(self, target_angle: float) -> float:
        if self._head == FORWARD:
            orientation = self._current_orientation
        else:
            orientation = mth.wrap2pi(self._current_orientation + math.pi)

        return mth.min_angle(orientation, target_angle)

    def interpolate_constants(self, speed: float) -> Tuple[float, float, float]:
        i = bisect_left(self._speed_keys, speed)

        if i == len(self._pid_constants_set):
            i -= 1
            return self._pid_constants_set[i][1:]

        if self._pid_constants_set[i][0] == speed:
            return self._pid_constants_set[i][1:]

        if 0 < i < len(self._pid_constants_set):
            w1 = speed - self._pid_constants_set[i - 1][0]
            set1 = np.array(self._pid_constants_set[i - 1][1:])

            w2 = self._pid_constants_set[i][0] - speed
            set2 = np.array(self._pid_constants_set[i][1:])

            norm = self._pid_constants_set[i][0] - self._pid_constants_set[i - 1][0]
            return (w1 * set1 + w2 * set2) / norm

    def get_optimal_speed(self, target_speed: float,
                          diff_angle: float,
                          target_distance: float) -> float:

        if target_speed < self._max_fine_movement_speed:
            return target_speed
        elif diff_angle * RAD2DEG < 10 and target_distance > 2 * self._alpha:
            return target_speed
        else:
            optimal_speed = self.sigmoid(target_speed, target_distance)
            return optimal_speed

    def sigmoid(self, target_speed: float,
                distance: float) -> float:
        scale = target_speed - self._max_fine_movement_speed

        s = scale / (1 + math.exp(0.5 * (-distance + self._alpha)))
        return s + self._max_fine_movement_speed

    def update_orientation(self, orientation: float) -> None:
        vec = np.array([math.cos(orientation), math.sin(orientation)])
        v = self._beta * self._ma_orientation_vec + \
            (1 - self._beta) * vec
        v /= (1 - self._beta ** self._t)

        self._ma_orientation_vec = v
        self._ma_orientation = math.atan2(v[1], v[0])
        self._t += 1
