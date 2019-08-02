from strategy.behaviour import TaskStatus, BlackBoard


class InvertOutput:
    def __init__(self, name='Not'):
        self.name = name
        self.child = None

    def run(self, blackboard: BlackBoard):
        if self.child is None:
            return TaskStatus.FAILURE, None
        else:
            status, action = self.child.run(blackboard)

            if status != TaskStatus.RUNNING:
                if status == TaskStatus.FAILURE:
                    return TaskStatus.SUCCESS, action
                else:
                    return TaskStatus.FAILURE, action
