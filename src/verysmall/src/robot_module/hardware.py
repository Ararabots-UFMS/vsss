from typing import Union, Tuple, List
from enum import Enum
from time import time
from collections import namedtuple
from robot_module.comunication.sender import STDMsg, SelfControlMsg


class RobotHardware:
    def __init__(self, max_speed:int = 255):
        self._hardware_max_speed = max_speed
        self._max_enable_speed = 70
        self._allowed_speed = self._max_enable_speed
        self._last_step_time = time()
        self._step_time = 0.05
        self._speed_step = 15

        self.LEFTFORWARD_RIGHTFORWARD   =   0x00 # 0000 0000
        self.LEFTFORWARD_RIGHTBACKWARD  =   0x01 # 0000 0001
        self.LEFTBACKWARD_RIGHTFORWARD  =   0x02 # 0000 0010
        self.LEFTBACKWARD_RIGHTBACKWARD =   0x03 # 0000 0011
        self.SET_MOTOR_CODE = 0
    
    def encode(self, msg: Union[STDMsg, SelfControlMsg]) -> List:
        if isinstance(msg, STDMsg):
            message = self.encodeSTDMsg(msg) 
        elif isinstance(msg, SelfControlMsg):
            message = None
        
        return message
    
    def encodeSTDMsg(self, msg: STDMsg) -> List[int]:
        message = []
        left, right = msg.left_speed, msg.right_speed

        if msg.left_speed >= 0 and msg.right_speed >= 0:
            message.append(self.SET_MOTOR_CODE | self.LEFTFORWARD_RIGHTFORWARD)
        elif msg.left_speed >= 0 and msg.right_speed <= 0:
            message.append(self.SET_MOTOR_CODE | self.LEFTFORWARD_RIGHTBACKWARD)
        elif msg.left_speed <= 0 and msg.right_speed >= 0:
            message.append(self.SET_MOTOR_CODE | self.LEFTBACKWARD_RIGHTFORWARD)
        else:
            message.append(self.SET_MOTOR_CODE | self.LEFTBACKWARD_RIGHTBACKWARD)
        
        message.append(abs(left))
        message.append(abs(right))

        return message

    def normalize_speeds(self, msg: STDMsg) -> STDMsg:
        max_abs_speed = max(abs(msg.left_speed), abs(msg.right_speed))
        self.update_allowed_speed(max_abs_speed)
        left, right = self.limitOutput(msg.left_speed, msg.right_speed)
        return STDMsg(left, right)
        
    
    def update_allowed_speed(self, target_abs_speed: float) -> None:
        t = time()
        
        if target_abs_speed < self._allowed_speed:
            self._allowed_speed = target_abs_speed
            self._last_step_time = t
        else:
            if self._allowed_speed < self._max_enable_speed:
                self._allowed_speed = min(self._max_enable_speed, target_abs_speed)
                self._last_step_time = t
            elif t - self._last_step_time > self._step_time:
                self._allowed_speed = min(target_abs_speed, 
                                          self._allowed_speed+self._speed_step)
                self._last_step_time = t
    
    def limitOutput(self, left_speed:float, right_speed:float) -> Tuple[int, int]:
        if abs(left_speed) > self._allowed_speed or \
        abs(right_speed) > self._allowed_speed:
            max_ = max(abs(left_speed), abs(right_speed))
            inv = 1.0/max_
            
            left_speed = inv*left_speed * self._allowed_speed
            right_speed = inv*right_speed * self._allowed_speed
            
            return int(left_speed), int(right_speed)
        else:
            return int(left_speed), int(right_speed)


