from typing import Callable, List
import rospy
from verysmall.msg import connection_status_topic

Callback = Callable[[List], None]

class MainWindowControllerSubscriber:
    def __init__(self, callback: Callback):
        self._callback = callback

        rospy.Subscriber('connection_status_topic',
                          connection_status_topic,
                          self._read_topic,
                          queue_size=5)

    def _read_topic(self, data: connection_status_topic) -> None:
        self._callback(data.sockets_status)