import rospy
from verysmall.msg import things_position, motor_speed

class Robot():
    """docstring for Robot"""

    def __init__(self, robot_id, bluetooth_id, robot_body, isActive=True, isAdversary=False):
        print __file__
        # Parameters
        self.robot_id = robot_id
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
        self.game_state = None
        self.role = None

        self.pub = rospy.Publisher('robot_' + str(robot_id), things_position, queue_size=1)

        if isActive:
            rospy.Subscriber('things_position', things_position, self.read_topic)

    # rospy.init_node('robot_'+str(robot_id))

    def run(self):
        print("Robo_" + str(self.robot_id) + ": Rodei principal")

    def read_parameters(self):
        pass

    def read_topic(self, data):
        self.position = data.team_pos[self.robot_id]
        self.vector = data.team_vector[self.robot_id]
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
