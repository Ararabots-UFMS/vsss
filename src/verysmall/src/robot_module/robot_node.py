#!/usr/bin/python3
from logging import debug
import rospy
import sys
from argparse import ArgumentParser
from robot_module.robot import Robot
import os
from utils.debug_profile import debug_profiler


def makeArgParser() -> ArgumentParser:
    parser = ArgumentParser()
    parser.add_argument("id", type=int, help="robot id")
    parser.add_argument("tag", type=int, help="tag id")
    parser.add_argument("body", type=str, help="robot body name")
    parser.add_argument("team_side", type=int, help="robot team side: 0->---, 1->---")
    parser.add_argument("team_color", type=int, help="robot team color: 0->blue, 1->yellow")
    parser.add_argument("robot_role", type=int, help="robot state machine id")
    parser.add_argument("owner_name", type=str, help="owner of this game topic and robots")
    parser.add_argument("socket_id", type=int, help="robot socket id")
    parser.add_argument("should_debug", type=int, default=0, help="robot debug flag")
    parser.add_argument("ros_args", nargs='*', help="additional ros parameters")
    return parser


if __name__ == '__main__':
    parser = makeArgParser()
    args = parser.parse_args()

    robot_name = "ROBOT " + str(args.id)
    rospy.init_node(robot_name, anonymous=True)
    rospy.logfatal(robot_name + " TAG: " + str(args.tag) + " Online")

    robot = Robot(args.id, args.tag, args.body, args.team_side, args.team_color,
                  args.robot_role, args.owner_name, args.socket_id, args.should_debug)

    rospy.spin()
    
    # debug_profiler.dump_stats(os.environ["ROS_ARARA_ROOT"] + "debug_logs.bin") # TODO: pensar em uma forma boa de habilitar isso
