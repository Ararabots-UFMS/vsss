#!/usr/bin/python
import sys
import os
import numpy as np
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT'] + "src/"
from utils.math_utils import angleBetween, distancePoints
from utils.json_handler import JsonHandler
sys.path[0]+="robot/movement/"
from control.PID import PID
from univector.un_field import univectorField
from rospy import logfatal

path = '../../../parameters/univector_constants.json'
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

class Movement():
    """Movement class return leftWheelSpeed(int), rightWheelSpeed(int), done(boolean)"""

    def __init__(self, PID_list, error=10, attack_side=RIGHT):
        self.pid = PID(kp=PID_list[0], ki=PID_list[1], kd=PID_list[2])
        self.last_pos = np.array([0, 0])
        self.error_margin = error
        self.attack_side = attack_side
        self.univet_field = univectorField(atack_goal=RIGHT)
        self.univet_field.updateConstants(RADIUS, KR, K0, DMIN, LDELTA)

    def do_univector(self, speed, robot_position, robot_vector, robot_speed, obstacle_position, obstacle_speed, ball_position):
        """Recive players positions and speed and return the speed to follow univector"""
        self.univet_field.updateObstacles(np.array(obstacle_position), np.array(obstacle_speed))
        vec = self.univet_field.getVec(np.array(robot_position), np.array(robot_speed), np.array(ball_position))
        logfatal(str(vec))
        logfatal(str(robot_vector))
        return self.follow_vector(np.array(robot_vector), np.array(vec), speed)

    def in_goal_position(self, robot_position, goal_position):
        """Verify if the robot is in goal position and return a boolean of the result"""
        if distancePoints(robot_position, goal_position) <= self.error_margin:
            return True
        return False

    def in_goal_vector(self, robot_vector, goal_vector):
        """Verify if the robot is in goal vector and return a boolean of the result"""
        if abs(angleBetween(robot_vector, goal_vector, ccw=False)) <= 0.0349066: #2 degrees error
            return True
        return False

    def move_to_point(self, robot_position, robot_vector, goal_position, speed):
        """Recives robot position, robot direction vector, goal position and a speed.
        Return the speed os the wheel to follow the vector (goal - robot)
        """
        if self.in_goal_position(robot_position, goal_position):
            return 0, 0, True
        if any(self.last_pos != goal_position):
            self.pid.reset()
        direction_vector = goal_position - robot_position
        return self.follow_vector(robot_vector, direction_vector, speed) 

    def follow_vector(self, robot_vector, goal_vector, speed):
        """Recives the robot vector, goal vector and a speed and return the speed
        of the wheels to follow the goal vector"""
        diff_angle = angleBetween(robot_vector, goal_vector, ccw=False) 
        correction = self.pid.update(diff_angle)
        return self.return_speed(speed, correction)

    def spin(self, speed, ccw=True):
        """Recives a speed and a boolean counterclockwise and return the left wheel speed,
        right wheel speed and a boolean. Spin the robot"""
        if ccw:
            return int(-speed), int(speed), False
        return int(speed), int(-speed), False

    def head_to(self, robot_vector, goal_vector, speed):
        """Recives robot direction vector, goal vector and a speed. Return the left wheels speed,
        right wheel speed and done. Robot vector and goal vector will be parallels vectors."""
        diff_angle = angleBetween(robot_vector, goal_vector, ccw=False)
        if self.in_goal_vector(robot_vector, goal_vector):
            return 0, 0, True
        correction = self.pid.update(diff_angle)
        if correction < 0:
            return self.normalize(int(speed+correction)), 0, False
        return 0, self.normalize(int(speed-correction)), False

    def return_speed(self, speed, correction):
        """Recives the robot speed and the PID correction, and return each wheel speed."""
        if speed < 0: #backwards
            if correction > 0:
                return int(speed), int(speed - correction), False
            else:
                return int(speed + correction), int(speed), False
        else: #forward
            if correction > 0:
                return self.normalize(int(speed - correction)), int(speed), False
            else:
                return int(speed), self.normalize(int(speed + correction)), False

    def normalize(self, speed):
        """Normalize robot speed
            speed: int
            return: int
        """
        if abs(speed) > 255:
            return 255*speed/abs(speed)
        return speed