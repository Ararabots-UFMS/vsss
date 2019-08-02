from strategy.behaviour import BlackBoard, Sequence, Selector, TaskStatus
from strategy.actions.state_behaviours import InState, ChangeState
from strategy.actions.game_behaviours import IsBehindBall
from strategy.actions.movement_behaviours import ChargeWithBall, univectorField, GoToBallUsingUnivector, \
    GoToAttackGoalUsingUnivector, StopAction
from strategy.strategy_utils import GameStates


class Penalty(Sequence):
    def __init__(self, name='Penalty'):
        super().__init__(name)

        check_state = InState('CheckPenaltyState', GameStates.PENALTY)
        self.add_child(check_state)

        check_if_behind_ball = Selector("CheckIfBehindBall")
        is_behind = IsBehindBall("Check", 25)
        change_state = ChangeState("ReturnToNormal", GameStates.NORMAL)

        check_if_behind_ball.add_child(is_behind)
        check_if_behind_ball.add_child(change_state)

        self.add_child(check_if_behind_ball)

        charge_with_ball = GoToAttackGoalUsingUnivector('FollowGoal', acceptance_radius=5, speed_prediction=False) #ChargeWithBall("ChargeWithBall", max_speed=220)

        self.add_child(charge_with_ball)


class FreeBall(Sequence):
    def __init__(self, name='FreeBall'):
        super().__init__(name)

        check_state = InState('CheckFreeBallState', GameStates.FREE_BALL)
        self.add_child(check_state)

        check_if_behind_ball = Selector("CheckIfBehindBall")
        is_behind = IsBehindBall("Check", 25)
        change_state = ChangeState("ReturnToNormal", GameStates.NORMAL)

        check_if_behind_ball.add_child(is_behind)
        check_if_behind_ball.add_child(change_state)

        self.add_child(check_if_behind_ball)
        self.add_child(GoToBallUsingUnivector('FollowBall'))  # FollowBall
        self.add_child(GoToAttackGoalUsingUnivector('FollowGoal'))  # FollowBall


class Stopped(Sequence):
    def __init__(self, name: str = 'Stopped'):
        super().__init__(name)

        self.add_child(InState('CheckStoppedState', GameStates.STOPPED))
        self.add_child(StopAction('Wait'))


class BaseTree(Selector):
    def __init__(self, name: str = "BaseTree"):
        super().__init__(name)

        self.add_child(Stopped('Stopped'))
        self.add_child(Penalty('Penalty'))
        self.add_child(FreeBall('FreeBall'))
