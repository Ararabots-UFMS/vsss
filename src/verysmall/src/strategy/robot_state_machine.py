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
    # def __init__(self):
    # Base States
    stop      = State('Stop', initial=True)
    normal    = State('Normal')
    freeball  = State('FreeBall')
    penalty   = State('Penalty')

    # Base Transictions

    # Stop to freebal game
    stop_to_freeball = stop.to(freeball)
    # Stop to normal game
    stop_to_normal = stop.to(normal)
    # Stop to penalty
    stop_to_penalty = stop.to(penalty)
    # Freball to normal game
    freeball_to_normal = freeball.to(normal)
    # Penalty to normal game
    penalty_to_normal = penalty.to(normal)

    go = stop_to_freeball | stop_to_normal | stop_to_penalty | freeball_to_normal | penalty_to_normal

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