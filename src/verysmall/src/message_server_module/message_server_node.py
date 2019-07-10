#!/usr/bin/python3
import rospy
from threading import Thread
from message_server_module.message_server import MessageServer


if __name__ == "__main__":
    rospy.init_node('message_server', anonymous=True)
    
    server = MessageServer()
    
    server_thread = Thread(target=server.loop, args=())
    server_thread.daemon = True
    server_thread.start()

    rospy.spin()
