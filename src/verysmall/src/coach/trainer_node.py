#!/usr/bin/python3
import rospy
from random import randint
from sys import argv

from coach.Trainer import Trainer

if __name__ == "__main__":
    rospy.init_node('trainer', anonymous=True)

    try:
        owner_id = argv[1]
    except ValueError:
        owner_id = 'Player_' + str(randint(0, 99999))

    
    trainer = Trainer(owner_id)

    rospy.spin()
