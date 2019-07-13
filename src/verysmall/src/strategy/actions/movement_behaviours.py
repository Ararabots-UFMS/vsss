class StopAction(Task):
    """ 
        A selector runs each task in order until one succeeds,
        at which point it returns SUCCESS. If all tasks fail, a FAILURE
        status is returned.  If a subtask is still RUNNING, then a RUNNING
        status is returned and processing continues until either SUCCESS
        or FAILURE is returned from the subtask.
    """
    def __init__(self, name, _desired_state):
        super(StopAction, self).__init__(name, *args, **kwargs)

    def run(self, blackboard):
        return TaskStatus.SUCCESS, (.0, .0, False)

class GoToBallUsingUnivector(Task):
    """docstring for GoToBallUsingUnivector"""
    def __init__(self, name, _desired_state):
        super(GoToBallUsingUnivector, self).__init__(name, *args, **kwargs)


    def run(self, blackboard):

        


        