from enum import Enum
import rospy


class TaskStatus(Enum):
    SUCCESS = 0
    FAILURE = 1
    RUNNING = 2


class BlackBoard:
    """docstring for BlackBoard"""

    def __init__(self):
        self.game_state = None
        self.team_side = None
        self.attack_goal = None

        self.freeball_robot_id = None
        self.meta_robot_id = None
        self.penalty_robot_id = None

        self.ball_position = None
        self.ball_speed = None

        self.my_id = None
        self.role = None
        self.position = None
        self.true_pos = None
        self.orientation = None
        self.speed = None

        self.team_color = None
        self.team_pos = None
        self.team_orientation = None
        self.team_speed = None

        self.enemies_position = None
        self.enemies_orientation = None
        self.enemies_speed = None

    def __repr__(self):
        return 'BlackBoard:\n' + \
               '\t--self.game_state: ' + str(self.game_state) + '\n' + \
               '\t--self.team_side: ' + str(self.team_side) + '\n' + \
               '\t--self.attack_goal: ' + str(self.attack_goal) + '\n' + \
               '\t--self.freeball_robot_id: ' + str(self.freeball_robot_id) + '\n' + \
               '\t--self.meta_robot_id: ' + str(self.meta_robot_id) + '\n' + \
               '\t--self.penalty_robot_id: ' + str(self.penalty_robot_id) + '\n' + \
               '\t--self.ball_position: ' + str(self.ball_position) + '\n' + \
               '\t--self.ball_speed: ' + str(self.ball_speed) + '\n' + \
               '\t--self.my_id: ' + str(self.my_id) + '\n' + \
               '\t--self.role: ' + str(self.role) + '\n' + \
               '\t--self.position: ' + str(self.position) + '\n' + \
               '\t--self.orientation: ' + str(self.orientation) + '\n' + \
               '\t--self.speed: ' + str(self.speed) + '\n' + \
               '\t--self.team_color: ' + str(self.team_color) + '\n' + \
               '\t--self.team_pos: ' + str(self.team_pos) + '\n' + \
               '\t--self.team_orientation: ' + str(self.team_orientation) + '\n' + \
               '\t--self.team_speed: ' + str(self.team_speed) + '\n' + \
               '\t--self.enemies_position: ' + str(self.enemies_position) + '\n' + \
               '\t--self.enemies_orientation: ' + str(self.enemies_orientation) + '\n' + \
               '\t--self.enemies_speed: ' + str(self.enemies_speed) + '\n'


class Sequence:
    """
        A sequence runs each task in order until one fails,
        at which point it returns FAILURE. If all tasks succeed, a SUCCESS
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """

    def __init__(self, name):
        self.name = name
        self.children = []

    def run(self, blackboard):

        for c in self.children:
            status, action = c.run(blackboard)

            if status != TaskStatus.SUCCESS:
                return status, action

        return TaskStatus.SUCCESS, None


class Selector:
    """
        A selector runs each task in order until one succeeds,
        at which point it returns SUCCESS. If all tasks fail, a FAILURE
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """

    def __init__(self, name):
        self.name = name
        self.children = []

    def run(self, blackboard):
        for c in self.children:
            status, action = c.run(blackboard)

            if status != TaskStatus.FAILURE:
                return status, action

        return TaskStatus.FAILURE, None
