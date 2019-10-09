from time import time
from typing import Union, Tuple, List

from robot_module.comunication.sender import STDMsg, SelfControlMsg


class RobotHardware:
    def __init__(self, max_speed: int = 255):
        self._hardware_max_speed = max_speed
        self._max_enable_speed = 70
        self._allowed_speed = self._max_enable_speed
        self._last_step_time = time()
        self._step_time = 0.05
        self._speed_step = 15

        self.LEFTFORWARD_RIGHTFORWARD = 0x00  # 0000 0000
        self.LEFTFORWARD_RIGHTBACKWARD = 0x01  # 0000 0001
        self.LEFTBACKWARD_RIGHTFORWARD = 0x02  # 0000 0010
        self.LEFTBACKWARD_RIGHTBACKWARD = 0x03  # 0000 0011

        self._current_motors_direction = self.LEFTFORWARD_RIGHTFORWARD

    def encode(self, msg: Union[STDMsg, SelfControlMsg]) -> List:
        if isinstance(msg, STDMsg):
            message = self.encode_std_msg(msg)
        elif isinstance(msg, SelfControlMsg):
            message = None

        return message

    def encode_std_msg(self, msg: STDMsg) -> List[int]:
        message = []
        
        self.set_current_direction(msg.left_speed, msg.right_speed)
        message.append(self._current_motors_direction)
        
        left, right = int(msg.left_speed), int(msg.right_speed)
        message.append(abs(left))
        message.append(abs(right))

        return message
    
    def set_current_direction(self, left_speed: float, right_speed: float) -> None:
        if left_speed >= 0 and right_speed >= 0:
            self._current_motors_direction = self.LEFTFORWARD_RIGHTFORWARD
        elif left_speed >= 0 and right_speed <= 0:
            self._current_motors_direction = self.LEFTFORWARD_RIGHTBACKWARD
        elif left_speed <= 0 and right_speed >= 0:
            self._current_motors_direction = self.LEFTBACKWARD_RIGHTFORWARD
        else:
            self._current_motors_direction = self.LEFTBACKWARD_RIGHTBACKWARD

    def normalize_speeds(self, msg: STDMsg) -> STDMsg:
        old_direction = self._current_motors_direction
        self.set_current_direction(msg.left_speed, msg.right_speed)
        if old_direction != self._current_motors_direction:
            self._allowed_speed = self._max_enable_speed

        max_abs_speed = max(abs(msg.left_speed), abs(msg.right_speed))
        self.update_allowed_speed(max_abs_speed)
        
        left, right = self.limit_output(msg.left_speed, msg.right_speed)
        return STDMsg(left, right)

    def update_allowed_speed(self, target_abs_speed: float) -> None:
        t = time()

        if target_abs_speed > self._hardware_max_speed:
            target_abs_speed = self._hardware_max_speed

        if target_abs_speed < self._allowed_speed:
            self._allowed_speed = target_abs_speed
            self._last_step_time = t
        else:
            if self._allowed_speed < self._max_enable_speed:
                self._allowed_speed = min(self._max_enable_speed, target_abs_speed)
                self._last_step_time = t
            elif t - self._last_step_time > self._step_time:
                self._allowed_speed = min(target_abs_speed,
                                          self._allowed_speed + self._speed_step)
                self._last_step_time = t

    def get_next_speed(self):
        return (self._allowed_speed + self._speed_step) % 256

    def limit_output(self, left_speed: float, right_speed: float) -> Tuple[int, int]:
        if abs(left_speed) > self._allowed_speed or \
                abs(right_speed) > self._allowed_speed:
            max_ = max(abs(left_speed), abs(right_speed))
            inv = 1.0 / max_

            left_speed = inv * left_speed * self._allowed_speed
            right_speed = inv * right_speed * self._allowed_speed

            return int(left_speed), int(right_speed)
        else:
            return int(left_speed), int(right_speed)
