import rospy

from verysmall.msg import message_server_topic
from message_server_module.message_server import MessageServer, Message

class MessageServerSubscriber:
    def __init__(self, my_server: MessageServer):
        self._my_server = my_server
        rospy.Subscriber('message_server_topic',
                          message_server_topic,
                          self._read_topic,
                          queue_size=5)

    def _read_topic(self, data:message_server_topic) -> None:
        m = Message(data.priority, data.socket_id, data.payload)
        self._my_server.putItemInBuffer(m)