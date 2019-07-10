import rospy
from typing import Union, List
from ROS.sender_publisher import SenderPublisher
from collections import namedtuple

STDMsg = namedtuple("STDMsg", ["left_speed", "right_speed"])
SelfControlMsg = namedtuple("SelfControlMsg", ["speed", "delta_theta"])

class Sender:
    def __init__(self, socket_id: int):
        self._socket_id = socket_id
        self.publisher = SenderPublisher()
    
    def send(self, priority: int, 
                   msg: List) -> None:
        self.publisher.publish(priority, self._socket_id, msg)
    
    