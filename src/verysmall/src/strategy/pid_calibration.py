from strategy.behaviour import Selector, Sequence, BlackBoard, TaskStatus
from strategy.actions.movement_behaviours import StopAction, GoToPositionUsingUnivector,SpinTask
from strategy.actions.state_behaviours import InState
from strategy.strategy_utils import GameStates
from itertools import cycle
from rospy import logwarn
from robot_module.movement.definitions import OpCodes
from strategy.actions.decorators import Timer

class CalibrationTree(Selector):
    def __init__(self, name="behave"):
        super().__init__(name)
        self.waypoints_list = cycle([(50, 50), (100, 50), (100, 100), (50, 100)])
        stop_sequence = Sequence('Stop Sequence')
        stop_sequence.children.append(InState('Stopped Game?', GameStates.STOPPED))
        stop_sequence.children.append(StopAction('Wait'))
        self.children.append(stop_sequence)

        patrol = Sequence('Patrol')
        self.my_litte_univector = GoToPositionUsingUnivector(position=next(self.waypoints_list))
        patrol.children.append(self.my_litte_univector)

        spin_task = Timer(exec_time=3)
        spin_task.add_child(SpinTask())
        patrol.children.append(spin_task)

        self.children.append(patrol)

    def run(self, blackboard):
        status, action = super().run(blackboard)
        if status == TaskStatus.SUCCESS:
            logwarn("Uhull next waypoint")
            self.my_litte_univector.set_position(next(self.waypoints_list))
            action = (OpCodes.STOP, .0, 0, .0)
        
        return status, action

