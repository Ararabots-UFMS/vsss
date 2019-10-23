from strategy.behaviour import BlackBoard, Sequence, Selector, TaskStatus
from strategy.actions.state_behaviours import InState, ChangeState
from strategy.actions.game_behaviours import IsBehindBall, IsTheWayFree, IsInsideMetaRange
from strategy.actions.movement_behaviours import *
from strategy.strategy_utils import GameStates
from strategy.actions.decorators import UseFrontHead

class Penalty(Sequence):
    def __init__(self, name='Penalty'):
        super().__init__(name)

        check_state = InState('CheckPenaltyState', GameStates.PENALTY)
        self.add_child(check_state)

        check_if_behind_ball = Selector("CheckIfBehindBall")
        is_behind = IsBehindBall("Check", 30)
        change_state = ChangeState("ReturnToNormal", GameStates.NORMAL)

        check_if_behind_ball.add_child(is_behind)
        check_if_behind_ball.add_child(change_state)

        self.add_child(check_if_behind_ball)

        charge_with_ball = ChargeWithBall("ChargeWithFreeWay", 200)

        self.add_child(charge_with_ball)


class FreeBall(Sequence):
    def __init__(self, name='FreeBall'):
        super().__init__(name)

        check_state = InState('CheckFreeBallState', GameStates.FREE_BALL)
        self.add_child(check_state)

        check_if_behind_ball = Selector("CheckIfBehindBall")
        is_behind = IsBehindBall("Check", 30)
        change_state = ChangeState("ReturnToNormal", GameStates.NORMAL)

        check_if_behind_ball.add_child(is_behind)
        check_if_behind_ball.add_child(change_state)

        self.add_child(check_if_behind_ball)
        self.add_child(GoToBallUsingMove2Point(speed=150, acceptance_radius=4))
        charge_with_ball = ChargeWithBall("ChargeWithFreeWay", 200)
        self.add_child(charge_with_ball)


class Meta(Sequence):
    def __init__(self, name: str = "Meta"):
        super().__init__(name)

        check_state = InState("CheckMetaState", GameStates.META)
        self.add_child(check_state)

        meta = Selector("IsInsideMetaRange")
        in_range_and_behind_the_ball = Sequence("InRangeAndBehindTheBall")
        is_behind_the_ball = IsBehindBall("BehindTheBall", 25)
        inside_meta_range = IsInsideMetaRange('MetaDist', 20)
        in_range_and_behind_the_ball.add_child(is_behind_the_ball)
        in_range_and_behind_the_ball.add_child(inside_meta_range)
        meta.add_child(in_range_and_behind_the_ball)

        change_state = ChangeState("ReturnToNormal", GameStates.NORMAL)
        meta.add_child(change_state)

        self.add_child(meta)
        # charge_with_ball = GoToAttackGoalUsingUnivector("FollowGoal",
        #                                                 acceptance_radius=5, speed_prediction=False)
        # charge_with_ball = ChargeWithBall("Charge", 200)
        charge_with_ball = GoToBallUsingMove2Point(speed=200, acceptance_radius=1)
        self.add_child(charge_with_ball)


class Stopped(Sequence):
    def __init__(self, name: str = 'Stopped'):
        super().__init__(name)

        self.add_child(InState('CheckStoppedState', GameStates.STOPPED))
        self.add_child(StopAction('Wait'))


class FreeWayAttack(Sequence):
    def __init__(self, name: str = "FreeWayAttack"):
        super().__init__(name)

        self.add_child(IsBehindBall("CheckIfBehindTheBall", 10))
        # self.add_child(IsTheWayFree("CheckIfTheWayIsFree", 10))
        charge_with_ball = ChargeWithBall("ChargeWithFreeWay", 200)
        self.add_child(charge_with_ball)


class BaseTree(Selector):
    def __init__(self, name: str = "BaseTree"):
        super().__init__(name)

        self.add_child(Stopped("Stopped"))
        self.add_child(UseFrontHead(child=Penalty("Penalty")))
        self.add_child(UseFrontHead(child=FreeBall("FreeBall")))
        self.add_child(UseFrontHead(child=Meta("Meta")))
