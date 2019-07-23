from strategy.behaviour import Selector, BlackBoard, ACTION

class GoalKeeper(Selector):
    def __init__(self, name: str = "behave"):
        super().__init__(name)
    
    
    def run(self, blackboard: BlackBoard) -> ACTION:
        result, action = super().run(blackboard)

        if result != # stop pro comidao
