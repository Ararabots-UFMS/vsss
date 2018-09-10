from statemachine import StateMachine, State
import sys
import os
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT']+"src/robot/"
from movement.functions.movement import Movement
import rospy
import numpy as np

movement = Movement([115, 0.005, 10, 0], 5)

class AttackerWithUnivector(StateMachine):
    """Simple attacker with univector field"""

    # Robot States
    univector = State('Univector', initial=True)

    action = univector.to(univector)

    def on_action(self, speed, robot_position, robot_vector, robot_speed,  obstacles_position, obstacles_speed,  ball_position):
        #return movement.follow_vector(200, [np.cos(robot_vector), np.sin(robot_vector)], [1, 0])
        return movement.do_univector(200, robot_position, [np.cos(robot_vector), np.sin(robot_vector)], [0, 0], np.resize(obstacles_position, (5, 2)), [[0,0]]*5, ball_position)