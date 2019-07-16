from strategy.behaviour import TaskStatus


class InState:
    """ 
        A selector runs each task in order until one succeeds,
        at which point it returns SUCCESS. If all tasks fail, a FAILURE
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """
    def __init__(self, name, _desired_state):
        self.name = name
        self.desired_state = _desired_state

    def run(self, blackboard):
        if self.desired_state == blackboard.game_state:
                return TaskStatus.SUCCESS

        return TaskStatus.FAILURE
