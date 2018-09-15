from statemachine.mixins import MachineMixin
from statemachine import StateMachine, State

class AttackerWithUnivector(MachineMixin):
    state_machine_name = 'RobotStateMachine'

    univector = State('Univector')

    normal_to_univector = normal.to(univector)

    normal_game = stop_to_normal | normal_to_univector

    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)
        super(AttackerWithUnivector, self).__init__()

    def __repr__(self):
        return "{}({!r})".format(type(self).__name__, self.__dict__)

    def on_enter_univector(self):
        self.normal_to_univector()
        return 1, 1, False