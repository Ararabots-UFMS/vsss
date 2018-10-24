from statemachine import StateMachine, State

class RunningStriker(StateMachine):
    """
    Class for creation of the state machine of robberies,
    consisting of basic states of each robbery

    Extends:
        StateMachine

    Variables:
        stop                {[State]} -- [Robberies stopped in this state]
        freeball            {[State]} -- [Freeball state]
        normal              {[State]} -- [Normal Game state]
        penalt              {[State]} -- [Penalty state]
        meta                {[State]} -- [Meta state]
        stop                {[Transition]} -- [Status of all stopped robberies]
        game_freeball       {[Transition]} -- [Transition that starts to freeball]
        game_normal         {[Transition]} -- [Game transition that occurs normally]
        game_penalt         {[Transition]} -- [Game transition stop to penalty]
        freeball_to_normal  {[Transition]} -- [Transition freeball to normal]
        penalt_to_normal    {[Transition]} -- [Transition penalty to normal]
        meta_to_normal      {[Transition]} -- [Transition meta to normal]
        normal_to_univector {[Transition]} -- [Transition normal to univector]
        univector_to_univector {[Transition]} -- [univector normal to univector]
    """
    # Base States
    stop = State('stop', initial=True)
    normal = State('Normal')
    freeball = State('FreeBall')
    penalty = State('Penalty')
    univector = State('Univector')
    running = State('Running')
    border = State('Border')
    point = State('Point')
    meta = State('Meta')

    # Stop to freeball game
    stop_to_freeball = stop.to(freeball)
    # Stop to normal game
    stop_to_normal = stop.to(normal)
    # Stop to penalty
    stop_to_penalty = stop.to(penalty)
    #Stop to running
    stop_to_running = stop.to(running)
    #Stop to border
    stop_to_border = stop.to(border)
    #Stop to point
    stop_to_point = stop.to(point)
    # Stop to Meta
    stop_to_meta = stop.to(meta)

    # Freeball to normal game
    freeball_to_normal = freeball.to(normal)
    # Freeball to running
    freeball_to_running = freeball.to(running)
    # Freeball to border
    freeball_to_border = freeball.to(border)
    # Freeball to point
    freeball_to_point = freeball.to(point)
    # Freeball to univector
    freeball_to_univector = freeball.to(univector)

    # Meta to normal game
    meta_to_normal = meta.to(normal)

    # Penalty to normal game
    penalty_to_normal = penalty.to(normal)
    # Penalty to univector
    penalty_to_univector = penalty.to(univector)
    # Penalty to border
    penalty_to_border = penalty.to(border)
    # Penalty to point
    penalty_to_point = penalty.to(point)

    normal_to_univector = normal.to(univector)
    normal_to_point = normal.to(point)
    normal_to_border = normal.to(border)
    univector_to_univector = univector.to(univector)

    point_to_point = point.to(point)
    go = stop_to_freeball | stop_to_normal | stop_to_penalty | freeball_to_normal | penalty_to_normal


class Striker(object):
    def __init__(self, state):
        self.state = state
