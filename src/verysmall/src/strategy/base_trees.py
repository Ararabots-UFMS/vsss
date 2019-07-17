from strategy.behaviour import BlackBoard, Sequence, Selector, TaskStatus
from strategy.actions.state_behaviours import InState, ChangeState
from strategy.actions.game_behaviours import IsBehindBall
from strategy.actions.movement_behaviours import ChargeWithBall
from strategy.strategy_utils import GameStates


class Penalty(Sequence):
    def __init__(self, name='Penalty'):
        super().__init__(name)

        check_state = InState('CheckState', GameStates.PENALTY)
        self.children.append(check_state)

        check_if_behind_ball = Selector("CheckIfBehindBall")
        is_behind = IsBehindBall("Check", 15)
        change_state = ChangeState("ReturnToNormal", GameStates.NORMAL)

        check_if_behind_ball.children.append(is_behind)
        check_if_behind_ball.children.append(change_state)

        self.children.append(check_if_behind_ball)

        charge_with_ball = ChargeWithBall("ChargeWithBall")

        self.children.append(charge_with_ball)