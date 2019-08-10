from strategy.behaviour import TaskStatus, Sequence, BlackBoard
from strategy.strategy_utils import GameStates


class InState:
    def __init__(self, name, _desired_state):
        self.name = name
        self.desired_state = _desired_state

    def run(self, blackboard):
        import rospy
        rospy.logfatal("estado:" + repr(self.desired_state))
        if self.desired_state == blackboard.game_state:
            rospy.logfatal("estou no estado:" + repr(self.desired_state))
            return TaskStatus.SUCCESS, None
        rospy.logfatal("nao estou em:" + repr(self.desired_state))
        return TaskStatus.FAILURE, None

class ChangeState:
    def __init__(self, name, _target_state):
        self.name = name
        self.target_state = _target_state

    def run(self, blackboard):
        blackboard.game_state = self.target_state
        return TaskStatus.FAILURE, None
