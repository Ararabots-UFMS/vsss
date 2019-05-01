from statemachine import StateMachine, State


class AttackerWithUnivector(StateMachine):
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
    stop = State('Stop', initial=True)
    normal = State('Normal')
    freeball = State('FreeBall')
    penalty = State('Penalty')
    univector = State('Univector')
    meta = State('Meta')
    spin = State('Spin')

    # Stop to freeball game
    stop_to_freeball = stop.to(freeball)
    # Stop to normal game
    stop_to_normal = stop.to(normal)
    # Stop to penalty
    stop_to_penalty = stop.to(penalty)
    # Freeball to normal game
    freeball_to_normal = freeball.to(normal)
    # Stop to Meta
    stop_to_meta = stop.to(meta)
    # Meta to normal game
    meta_to_normal = meta.to(normal)
    # Penalty to normal game
    penalty_to_normal = penalty.to(normal)

    normal_to_univector = normal.to(univector)
    univector_to_univector = univector.to(univector)

    univector_to_spin = univector.to(spin)

    spin_to_univector = spin.to(univector)

    go = stop_to_freeball | stop_to_normal | stop_to_penalty | freeball_to_normal | penalty_to_normal


class MyModel(object):
    def __init__(self, state):
        self.state = state