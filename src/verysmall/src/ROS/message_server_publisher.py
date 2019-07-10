from typing import List
import rospy
import numpy as np

from verysmall.msg import connection_status_topic


class MessageServerPublisher:
    def __init__(self):
        self.TAG = "MESSAGE SERVER PUBLISHER"
        self.publisher = rospy.Publisher('connection_status_topic', 
                                          connection_status_topic,
                                          queue_size=10)
    
    
    def publish(self, sockets_status: List) -> None:
        msg = sockets_status
        try:
            self.publisher.publish(msg)
        except rospy.ROSException as e:
            rospy.logfatal(self.TAG+": UNABLE TO PUBLISH. "+repr(e))
        
