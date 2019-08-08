#!/usr/bin/python3
import rospy
from threading import Thread
from message_server_module.message_server import MessageServer
from sys import argv
from random import randint

if __name__ == "__main__":
    rospy.init_node('message_server', anonymous=True)

    try:
        owner_id = argv[1]
    except ValueError:
        owner_id = 'Player_' + str(randint(0, 99999))
    
    server = MessageServer(owner_id=owner_id)
    
    server_thread = Thread(target=server.loop, args=())
    server_thread.daemon = True
    server_thread.start()

    rospy.spin()
