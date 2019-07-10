from typing import Union, Tuple, List
from enum import Enum
from collections import namedtuple
from robot_module.comunication.sender import STDMsg, SelfControlMsg


class RobotHardware:
    def __init__(self, max_speed:int = 255):
        self.max_speed = max_speed

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
        left, right = self.limitOutput(msg.left_speed, msg.right_speed)

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
    
    
    def limitOutput(self, left_speed:float, right_speed:float) -> Tuple[int, int]:
        if abs(left_speed) > self.max_speed or \
           abs(right_speed) > self.max_speed:
            max_ = max(abs(left_speed), abs(right_speed))
            inv = 1.0/max_
            
            left_speed = inv*left_speed * self.max_speed
            right_speed = inv*right_speed * self.max_speed
            
            return int(left_speed), int(right_speed)
        else:
            return int(left_speed), int(right_speed)