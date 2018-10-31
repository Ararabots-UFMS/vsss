from statemachine import StateMachine, State

class Zagueiro(StateMachine):
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
    """
    # Base States
    stop      = State('Stop', initial=True)
    normal    = State('Normal')
    freeball  = State('FreeBall')
    penalty   = State('Penalty')
    meta      = State('Meta')

    # Stop to freeball game
    stop_to_freeball = stop.to(freeball)
    # Stop to normal game
    stop_to_normal = stop.to(normal)
    # Stop to penalty
    stop_to_penalty = stop.to(penalty)
    # Stop to Meta
    stop_to_meta = stop.to(meta)
    # Freeball to normal game
    freeball_to_normal = freeball.to(normal)
    # Penalty to normal game
    penalty_to_normal = penalty.to(normal)
    # Meta to normal game
    meta_to_normal = meta.to(normal)

    go = stop_to_freeball | stop_to_normal | stop_to_penalty | freeball_to_normal | penalty_to_normal


    defend      = State('Defend ball')
    wait_ball   = State('Wait ball')
    do_spin     = State('Spin')
    move        = State('Move')

    normal_to_defend        = normal.to(defend)
    normal_to_wait_ball     = normal.to(wait_ball)

    defend_to_wait_ball     = defend.to(wait_ball)
    defend_to_spin          = defend.to(do_spin)
    spin_to_defend          = do_spin.to(defend)
    defend_to_move          = defend.to(move)
    move_to_defend          = move.to(defend)
    wait_ball_to_defend     = wait_ball.to(defend)
    



class MyModel(object):
    def __init__(self, state):
        self.state = state