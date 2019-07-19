from strategy.behaviour import BlackBoard, Sequence, Selector, TaskStatus
from strategy.actions.state_behaviours import InState, ChangeState
from strategy.actions.game_behaviours import IsBehindBall
from strategy.actions.movement_behaviours import ChargeWithBall, univectorField, GoToBallUsingUnivector, \
    GoToAttackGoalUsingUnivector
from strategy.strategy_utils import GameStates


class Penalty(Sequence):
    def __init__(self, name='Penalty'):
        super().__init__(name)

        check_state = InState('CheckPenaltyState', GameStates.PENALTY)
        self.children.append(check_state)

        check_if_behind_ball = Selector("CheckIfBehindBall")
        is_behind = IsBehindBall("Check", 25)
        change_state = ChangeState("ReturnToNormal", GameStates.NORMAL)

        check_if_behind_ball.children.append(is_behind)
        check_if_behind_ball.children.append(change_state)

        self.children.append(check_if_behind_ball)

        charge_with_ball = GoToAttackGoalUsingUnivector('FollowGoal', acceptance_radius=5, speed_prediction=False) #ChargeWithBall("ChargeWithBall", max_speed=220)

        self.children.append(charge_with_ball)


class FreeBall(Sequence):
    def __init__(self, name='FreeBall'):
        super().__init__(name)

        check_state = InState('CheckFreeBallState', GameStates.FREE_BALL)
        self.children.append(check_state)

        check_if_behind_ball = Selector("CheckIfBehindBall")
        is_behind = IsBehindBall("Check", 25)
        change_state = ChangeState("ReturnToNormal", GameStates.NORMAL)

        check_if_behind_ball.children.append(is_behind)
        check_if_behind_ball.children.append(change_state)

        self.children.append(check_if_behind_ball)
        self.children.append(GoToBallUsingUnivector('FollowBall'))  # FollowBall
        self.children.append(GoToAttackGoalUsingUnivector('FollowGoal'))  # FollowBall
