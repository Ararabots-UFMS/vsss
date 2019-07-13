from strategy.behaviour import *
from strategy.actions.movement_behaviours import StopAction


class AttackerWithUnivectorController:
    """docstring for ClassName"""

    def __init__(self, name='behave'):
        self.name = name
        self.children = []

        stopped = Sequence('Stopped')
        stopped.children.append(StopAction('Wait'))
        self.children.append(stopped)