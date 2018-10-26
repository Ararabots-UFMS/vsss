#!/usr/bin/python
import sys
import os
import numpy as np
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT'] + "src/"
path = sys.path[0] + 'parameters/univector_constants.json'
from utils.math_utils import angleBetween, distancePoints, unitVector
from utils.json_handler import JsonHandler
sys.path[0]+="robot/movement/"
from control.PID import PID
from univector.un_field import univectorField
from rospy import logfatal

jsonHandler = JsonHandler()
univector_list = jsonHandler.read(path)

# univector
RADIUS = univector_list['RADIUS']
KR = univector_list['KR']
K0 = univector_list['K0']
DMIN = univector_list['DMIN']
LDELTA = univector_list['LDELTA']

RIGHT = 1
LEFT = 0

SOFTWARE = 0
HARDWARE = 1

class Movement():
    """Movement class return leftWheelSpeed(int), rightWheelSpeed(int), done(boolean)"""

    def __init__(self, PID_list, error=10, attack_goal=RIGHT, _pid_type=SOFTWARE, _debug_topic = None):
        self.pid = PID(kp=PID_list[0], ki=PID_list[1], kd=PID_list[2])
        self.last_pos = np.array([0, 0])
        self.error_margin = error
        self.attack_goal = attack_goal
        if type(attack_goal) is int:
            self.univet_field = univectorField(attack_goal=self.attack_goal)
        else:
            self.univet_field = univectorField(attack_goal=attack_goal, _rotation = True)

        self.univet_field.updateConstants(RADIUS, KR, K0, DMIN, LDELTA)
        self.pid_type = _pid_type
        self.debug_topic = _debug_topic

    def set_pid_type(self, _pid_type):
        """
            Define if the pid correction is done on SOFTWARE or HARDWARE (0 or 1 respectively)
        """
        self.pid_type = _pid_type

    def update_pid(self, PID_list):
        """
        Update pid paramters
        :param PID_list: [float, float, float]
        :return:
        """
        self.pid.set_constants(PID_list[0], PID_list[1], PID_list[2])

    def do_univector(self, speed, robot_position, robot_vector, robot_speed, obstacle_position, obstacle_speed, ball_position):
        """Recive players positions and speed and return the speed to follow univector
         :param speed : int
         :param robot_position : np.array([float, float])
         :param robot_vector : np.array([float, float])
         :param robot_speed : np.array([float, float])
         :param obstacle_position : np.array([float, float])
         :param obstacle_speed : np.array([float, float])
         :param ball_position : np.array([float, float])

        :return: returns nothing
        """
        self.univet_field.updateObstacles(np.array(obstacle_position), np.array(obstacle_speed))
        vec = self.univet_field.getVec(np.array(robot_position), np.array(robot_speed), np.array(ball_position))
        return self.follow_vector(speed, np.array(robot_vector), np.array(vec))

    def in_goal_position(self, robot_position, goal_position):
        """Verify if the robot is in goal position and return a boolean of the result
         :param robot_position : np.array([float, float])
         :param goal_position : np.array([float, float])

        :return: returns : boolean
        """
        if distancePoints(robot_position, goal_position) <= self.error_margin:
            return True
        return False

    def in_goal_vector(self, robot_vector, goal_vector):
        """Verify if the robot is in goal vector and return a boolean of the result
         :param robot_vector : np.array([float, float])
         :param goal_vector : np.array([float, float])

         :return: returns : boolean
        """
        if abs(angleBetween(robot_vector, goal_vector, ccw=False)) <= 0.087266463: #5 degrees error
            return True
        return False

    def move_to_point(self, speed, robot_position, robot_vector, goal_position):
        """Recives robot position, robot direction vector, goal position and a speed.
        Return the speed os the wheel to follow the vector (goal - robot)
         :param speed : int
         :param robot_position : np.array([float, float])
         :param robot_vector : np.array([float, float])
         :param goal_position : np.array([float, float])

        :return: returns int, int, boolean
        """
        if self.in_goal_position(robot_position, goal_position):
            return 0, 0, True
        direction_vector = unitVector(goal_position - robot_position)

        # logfatal(str(direction_vector))
        return self.follow_vector(speed, robot_vector, direction_vector)

    def follow_vector(self,  speed, robot_vector, goal_vector):
        """Recives the robot vector, goal vector and a speed and return the speed
        of the wheels to follow the goal vector
         :param speed : int
         :param robot_vector : np.array([float, float])
         :param goal_vector : np.array([float, float])

        :return: returns int, int, boolean
        """
        if self.debug_topic is not None:
            self.debug_topic.debug_publish(goal_vector.tolist())

        diff_angle = angleBetween(robot_vector, goal_vector, ccw=True)
        # Return the speed and angle if the PID is in hardware, otherwise
        # returns both wheels speed and its correction
        if self.pid_type == HARDWARE:
            return (diff_angle, speed)

        correction = self.pid.update(diff_angle)
        return self.return_speed(speed, correction)

    def spin(self, speed, ccw=True):
        """Recives a speed and a boolean counterclockwise and return the left wheel speed,
           right wheel speed and a boolean. Spin the robot
         :param speed : int
         :param ccw : boolean

         :return: returns int, int, boolean
        """
        if ccw:
            return int(-speed), int(speed), False
        return int(speed), int(-speed), False

    def head_to(self, robot_vector, goal_vector):
        """Recives robot direction vector, goal vector and a speed. Return the left wheels speed,
        right wheel speed and done. Robot vector and goal vector will be parallels vectors.
        :param robot_vector : np.array([float, float])
        :param goal_vector : np.array([float, float])

        :return: returns int, int, boolean
        """
        diff_angle = angleBetween(robot_vector, goal_vector, ccw=False)
        #logfatal(str(diff_angle))

        if self.in_goal_vector(robot_vector, goal_vector):
            return 0, 0, True

        if self.pid_type == HARDWARE:
            return diff_angle, 0, False

        correction = self.pid.update(diff_angle)
        return self.normalize(int(correction)), self.normalize(int(-correction)), False

    def return_speed(self, speed, correction):
        """Recives the robot speed and the PID correction, and return each wheel speed.
        :param speed : int
        :param correction : int

        :return: returns int, int, boolean
        """
        if speed < 0: #backwards
            # TODO: Change here when use backwards
            if correction > 0:
                return int(speed), int(speed - correction), False
            else:
                return int(speed + correction), int(speed), False
        else: #forward
            return self.normalize(int(speed + correction)), self.normalize(int(speed - correction)), False

    def normalize(self, speed):
        """Normalize robot speed
            :param speed: int
            :return: return int
        """
        if abs(speed) > 255:
            return 255*speed/abs(speed)
        return speed
