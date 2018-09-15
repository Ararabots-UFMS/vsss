from statemachine.mixins import MachineMixin

class AttackerWithUnivector(MachineMixin):
    state_machine_name = 'RobotStateMachine'

    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)
        super(AttackerWithUnivector, self).__init__()

    def __repr__(self):
        return "{}({!r})".format(type(self).__name__, self.__dict__)