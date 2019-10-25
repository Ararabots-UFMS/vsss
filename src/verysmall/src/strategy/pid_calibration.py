from functools import partial

from strategy.behaviour import Selector, Sequence, BlackBoard, TaskStatus
from strategy.actions.movement_behaviours import GoToPosition, StopAction, SpinTask
from strategy.actions.state_behaviours import InState
from strategy.strategy_utils import GameStates
import cv2
from itertools import cycle
from rospy import logwarn
from robot_module.movement.definitions import OpCodes
from strategy.actions.decorators import Timer, IgnoreSmoothing
import numpy as np


class CalibrationTree(Selector):
    def __init__(self, name="behave", robot=None):
        super().__init__(name)
        self.show_pid_tweaker = True
        self.waypoints_list = cycle([(37, 25), (117, 25), (117, 105), (37, 105)])
        stop_sequence = Sequence('Stop Sequence')
        stop_sequence.children.append(InState('Stopped Game?', GameStates.STOPPED))
        stop_sequence.children.append(StopAction('Wait'))
        self.children.append(stop_sequence)
        self._robot = robot
        self.current_orientation = True
        self.current_speed = 0
        patrol = Sequence('Patrol')
        self.straight_line_movement = GoToPosition(target_pos=next(self.waypoints_list),
                                                   max_speed=self.current_speed,
                                                   acceptance_radius=15.0)

        ignore_smoothing = IgnoreSmoothing(name="Ignore smoothing pid")
        ignore_smoothing.add_child(self.straight_line_movement)
        patrol.children.append(ignore_smoothing)
        spin_task = Timer(exec_time=3)
        spin_task.add_child(SpinTask())
        # patrol.children.append(spin_task)

        self.children.append(patrol)

    def update_pid(self, key, scale, value):
        pass

    def create_pid_tweaker(self):
        big_range = 10e5
        medium_range = 10e4
        small_range = 10e3
        scale = 10e1
        self.empty_image = np.zeros((1, 500, 3), np.uint8)
        cv2.namedWindow("PID Tweaker", 1)
        cv2.imshow("PID Tweaker", self.empty_image)

        constants = self._robot._controller.pid_constants_set[str(self.current_speed)]

        cv2.createTrackbar("Front:", "PID Tweaker", value=int(self.current_orientation), count=1,
                           onChange=partial(self.update_pid, "Front", 1))
        cv2.createTrackbar("Speed", "PID Tweaker", 0, 4,
                           partial(self.update_pid, "Speed", 50))

        cv2.createTrackbar("KP", "PID Tweaker", int(constants["KP"] * scale), big_range,
                           partial(self.update_pid, "KP", scale))
        cv2.createTrackbar("KD", "PID Tweaker", int(constants["KD"] * scale), medium_range,
                           partial(self.update_pid, "KD", scale))
        cv2.createTrackbar("KI", "PID Tweaker", int(constants["KI"] * scale), small_range,
                           partial(self.update_pid, "KI", scale))
        self.show_pid_tweaker = True

    def run(self, blackboard):
        status, action = super().run(blackboard)
        if status == TaskStatus.SUCCESS:
            self.straight_line_movement.set_new_target_pos(next(self.waypoints_list))
            action = (OpCodes.STOP, .0, 0, .0)

        opcode = action[0] + (OpCodes.USE_FORWARD_HEAD if self.current_orientation else OpCodes.USE_BACKWARD_HEAD)

        return status, (opcode,) + action[1:]
