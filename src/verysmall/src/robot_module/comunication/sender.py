import rospy
from typing import Union, List
from ROS.sender_publisher import SenderPublisher
from collections import namedtuple

STDMsg = namedtuple("STDMsg", ["left_speed", "right_speed"])
SelfControlMsg = namedtuple("SelfControlMsg", ["direction", "speed", "delta_theta"])


class Sender:
    def __init__(self, socket_id: int, owner_name: str = None):
        self._socket_id = socket_id
        self.publisher = SenderPublisher(owner_name)

    def send(self, priority: int,
             msg: List) -> None:
        self.publisher.publish(priority, self._socket_id, msg)
