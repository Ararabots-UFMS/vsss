#!/usr/bin/python
import rospy
import sys
from robot import Robot


if __name__ == '__main__':
    # robot_id body_id node_name
    rospy.init_node(sys.argv[1])
    rospy.logfatal(sys.argv[1]+" - "+ sys.argv[2] +" Online")
    robot = Robot(sys.argv[1], sys.argv[2], sys.argv[3])
    rospy.on_shutdown(robot.bluetooth_detach())
    rospy.spin()
