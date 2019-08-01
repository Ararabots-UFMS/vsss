from strategy.behaviour import *
from strategy.actions.state_behaviours import InState
from strategy.actions.movement_behaviours import GoToGoalCenter, StopAction
from strategy.base_trees import Penalty, FreeBall
from strategy.strategy_utils import GameStates

class GoalKeeper(Selector):
    def __init__(self, name: str = "behave"):
        super().__init__(name)
        
        stopped = Sequence('Stopped')
        stopped.add_child(InState('CheckStoppedState', GameStates.STOPPED))
        stopped.add_child(StopAction('Wait'))
        self.add_child(stopped)
        
        self.add_child(Penalty())
        self.add_child(FreeBall())

        normal = Sequence("Normal")
        normal.add_child(InState("CheckNormalState", GameStates.NORMAL))
        normal.add_child(GoToGoalCenter())
        self.add_child(normal)
