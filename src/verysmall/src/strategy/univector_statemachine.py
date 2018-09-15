from statemachine import StateMachine, State
from statemachine.mixins import MachineMixin
from robot_state_machine import RobotStateMachine

class AttackerWithUnivector(RobotStateMachine):
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)
        super(AttackerWithUnivector, self).__init__()

    def __repr__(self):
        return "{}({!r})".format(type(self).__name__, self.__dict__)