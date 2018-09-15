from statemachine import StateMachine, State

class RobotStateMachine(StateMachine):
    """
    Class for creation of the state machine of robberies, 
    consisting of basic states of each robbery
    
    Extends:
        StateMachine
    
    Variables:
        stop                {[State]} -- [Robberies stopped in this state]
        freeball            {[State]} -- [Free ball state]
        normal              {[State]} -- [Game playing normally]
        penalt              {[State]} -- [Penalty status]
        stop                {[Transition]} -- [Status of all stopped robberies]
        game_freeball       {[Transition]} -- [Transition that starts to freeball]
        game_normal         {[Transition]} -- [Game transition that occurs normally]
        game_penalt         {[Transition]} -- [Game transition stop to penalty]
        freeball_to_normal  {[Transition]} -- [Transition freeball to normal]
        penalt_to_normal    {[Transition]} -- [Transition penalty to normal]
    """
    def __init__(self):
        # Base States
        self.stop      = State('Stop', initial=True, value=1)
        self.normal    = State('Normal', value=2)
        self.freeball  = State('FreeBall', value=3)
        self.penalty   = State('Penalty', value=4)

        # Base Transictions

        # Stop to freebal game
        self.stop_to_freeball = self.stop.to(self.freeball)
        # Stop to normal game
        self.stop_to_normal = self.stop.to(self.normal)
        # Stop to penalty
        self.stop_to_penalty = self.stop.to(self.penalty)
        # Freball to normal game
        self.freeball_to_normal = self.freeball.to(self.normal)
        # Penalty to normal game
        self.penalty_to_normal = self.penalty.to(self.normal)

        self.ee = self.stop_to_freeball | self.stop_to_normal | self.stop_to_penalty | self.freeball_to_normal | self.penalty_to_normal

    # Stop all robbery
    def on_enter_stop(self):
        print "Stop"

    # Callback of transition of the Stop Game to freeball
    def on_enter_normal(self):
        print "Normal"

    # Callback of transition of the Stop Game to Normal
    def on_enter_freeball(self):
        print "Freeball"

    # Callback of transition of the Stop Game to Penalt
    def on_enter_penalty(self):
        print "Penalty"


class ModelMachine(object):
    def __init__(self, state):
        self.state = state

    def stop(self):
        self.state = "Stop"