from enum import Enum


class TaskStatus(Enum):
    SUCCESS = 0
    FAILURE = 1
    RUNNING = 2


class BlackBoard:
    """docstring for BlackBoard"""

    def __init__(self):
        self.game_state = None
        self.team_side = None

        self.freeball_robot_id = None
        self.meta_robot_id = None
        self.penalty_robot_id = None

        self.ball_position = None
        self.ball_speed = None

        self.my_id = None
        self.role = None
        self.position = None
        self.orientation = None
        self.speed = None

        self.team_color = None
        self.team_pos = None
        self.team_orientation = None
        self.team_speed = None

        self.enemies_position = None
        self.enemies_orientation = None
        self.enemies_speed = None


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
            status = c.run(blackboard)

            if status != TaskStatus.SUCCESS:
                return status

        return TaskStatus.SUCCESS


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
            status = c.run(blackboard)

            if status != TaskStatus.FAILURE:
                return status

        return TaskStatus.FAILURE
