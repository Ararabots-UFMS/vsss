import rospy
import sys
from verysmall.msg import things_position, motor_speed,game_topic
from comunication.sender import Sender
try:
    from utils.json_handler import JsonHandler
except ImportError:
    sys.path[0] = sys.path[0] + '/../'
    from utils.json_handler import JsonHandler

class Robot():
    """docstring for Robot"""

    def __init__(self, robot_id, bluetooth_id, robot_body, isAdversary=False):
        # Parameters
        self.robot_id = robot_id
        self.robot_id_integer = int(self.robot_id.split("_")[1]) - 1
        self.bluetooth_id = bluetooth_id
        self.robot_body = robot_body

        # Receive from vision
        self.position = None
        self.vector = None
        self.ball_position = None
        self.team_vector = None
        self.team_position = None
        self.enemies_vector = None
        self.enemies_position = None

        # Calculated inside robot
        self.motor_speed = None
        self.PID = None

        # Receive from game topic
        self.behaviour_type = None
        self.game_state = 0
        self.role = None
        self.penalty_robot = None
        self.freeball_robot = None
        self.meta_robot = None

        # Open bluetooth socket
        self.bluetooth_sender = Sender(self.robot_id_integer, self.bluetooth_id)
        self.bluetooth_sender.connect()

        rospy.Subscriber('things_position', things_position, self.read_topic)

        rospy.Subscriber('game_topic', game_topic, self.read_game_topic)

        self.changed_game_state = True
        self.game_state_string = ["Stopped",
                                  "Normal Play",
                                  "Freeball",
                                  "Penaly",
                                  "Meta"]

    def run(self):

        if self.game_state == 0:  # Stopped
            pass
        elif self.game_state == 1:  # Normal Play
            pass
        elif self.game_state == 2:  # Freeball
            pass
        elif self.game_state == 3:  # Penaly
            pass
        elif self.game_state == 4:  # Meta
            pass
        else:  # I really really really Dont Know
            print("wut")

        if self.changed_game_state:
            rospy.logfatal("Robo_" + self.robot_id + ": Run("+self.game_state_string[self.game_state]+")")
            self.changed_game_state = False

    def read_parameters(self):
        pass

    def read_game_topic(self, data):
        self.game_state = data.game_state
        self.penalty_robot = data.penalty_robot
        self.freeball_robot = data.freeball_robot
        self.meta_robot = data.meta_robot
        self.role = data.robot_roles[self.robot_id_integer]
        #rospy.logfatal(sys.argv[1] + " - " + str(self.game_state) + " - " + str(self.role))
        self.changed_game_state = True

    def read_topic(self, data):
        self.position = data.team_pos[self.robot_id_integer]
        self.vector = data.team_vector[self.robot_id_integer]
        self.ball_position = data.ball_pos
        self.team_vector = data.team_pos
        self.team_position = data.team_vector
        self.enemies_vector = data.enemies_vector
        self.enemies_position = data.enemies_pos
        self.run()

    def go_to(self):
        pass

    def follow_vector(self):
        pass

    def head_to(self):
        pass

    def spin(self):
        pass

    def compute_univector_field(self):
        pass

    def send_control(self):
        pass

    def debug(self):
        pass

    def bluetooth_detach(self):
        self.bluetooth_sender.closeSocket()
