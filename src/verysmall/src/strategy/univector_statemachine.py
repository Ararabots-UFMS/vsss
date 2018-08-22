from statemachine import StateMachine, State
import sys
import os
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT']+"src/robot/"
from movement.functions.movement import Movement
import rospy
import numpy as np

movement = Movement([115, 0.005, 10, 0.05])

class AttackerWithUnivector(StateMachine):
    """Simple attacker with univector field"""

    # Robot States
    univector = State('Univector', initial=True)

    action = univector.to(univector)

    def on_action(self, speed, robot_position, robot_vector, robot_speed,  obstacles_position, obstacles_speed,  ball_position):
        rospy.logfatal(str(movement.pid.error))
        return movement.follow_vector([np.cos(robot_vector), np.sin(robot_vector)], [-1, 0], 130)
        # return movement.do_univector(speed, robot_position, [np.cos(robot_vector), np.sin(robot_vector)], [0,0] , obstacles_position, [[0, 0]]*len(obstacles_position), ball_position)