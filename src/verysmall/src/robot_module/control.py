from typing import Tuple, List
import numpy as np
import math
from bisect import bisect_left
from time import time
#import robot_module.robot as robot
from robot_module.PID import PIDController
from robot_module.movement.definitions import OpCodes

from utils.math_utils import DEG2RAD, FORWARD, BACKWARDS
import utils.math_utils as mth 


Constants = Tuple[int, float, float, float]

class Control:
    def __init__(self, myrobot,
                       constants: List[Constants]) -> None:
        self._myrobot = myrobot

        self._head = FORWARD
        self._hysteresis_angle_window = 10 * DEG2RAD
        self._upper_angle_tol = math.pi/2.0 + self._hysteresis_angle_window
        self._lower_angle_tol = math.pi/2.0 - self._hysteresis_angle_window

        self._pid_constants_set = sorted(constants)
        self._speed_keys = [s[0] for s in self._pid_constants_set] # TODO: get a better name
        self._pidController = PIDController()
        self._pid_last_use = time()
        self._pid_reset_time = 0.032 # 2 frames

    def get_wheels_speeds(self, opcode: OpCodes, 
                                speed: np.array,
                                angle: float,
                                distance: float) -> Tuple[float, float]:

        if opcode == OpCodes.NORMAL:
            return self._follow_vector(speed, angle, distance)
        elif opcode == OpCodes.SPIN_CCW:
            return -255, 255
        elif opcode == OpCodes.SPIN_CW:
            return 255, -255
        else:
            return 0, 0
        
    
    def _follow_vector(self, speed, angle, distance) -> Tuple[float, float]:
        self.set_head(angle)
        diff_angle = self.get_diff_angle(angle)

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
            return -speed - correction, -speed + correction
        

    def set_head(self, angle: float) -> np.array:
        diff = mth.min_angle(self._myrobot.orientation, angle)

        if diff > self._upper_angle_tol:
            self._head = BACKWARDS
        elif diff < self._lower_angle_tol:
            self._head = FORWARD
    
    def get_diff_angle(self, target_angle: float) -> float:
        if self._head == FORWARD:
            orientation = self._myrobot.orientation
        else:
            orientation = mth.wrap2pi(self._myrobot.orientation + math.pi)
        
        return mth.min_angle(orientation, target_angle)
    
    def interpolate_constants(self, speed: float) -> Tuple[float, float, float]:
        i = bisect_left(self._speed_keys, speed)
        
        if self._pid_constants_set[i][0] == speed:
            return self._pid_constants_set[i][1:]
        elif 0 < i < len(self._pid_constants_set):
            w1 = self._pid_constants_set[i-1][0] - speed
            set1 = np.array(self._pid_constants_set[i-1][1:])
            
            w2 = speed - self._pid_constants_set[i][0]
            set2 = np.array(self._pid_constants_set[i][1:])
            
            norm = self._pid_constants_set[i][0] - self._pid_constants_set[i-1][0]
            return (w1*set1 + w2*set2) / norm
        else:
            if i == len(self._pid_constants_set):
                i -= 1
            
            return self._pid_constants_set[i][1:]


