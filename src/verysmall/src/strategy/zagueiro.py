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
    border      = State('Border')
    area        = State('Area')
    locked        = State('Locked')

    normal_to_defend      = normal.to(defend)
    normal_to_wait_ball   = normal.to(wait_ball)
    normal_to_do_spin     = normal.to(do_spin)
    normal_to_move        = normal.to(move)
    normal_to_border      = normal.to(border)
    normal_to_area        = normal.to(area)
    normal_to_locked      = normal.to(locked)

    defend_to_wait_ball   = defend.to(wait_ball)
    defend_to_do_spin     = defend.to(do_spin)
    defend_to_normal      = defend.to(normal)
    defend_to_move        = defend.to(move)
    defend_to_border      = defend.to(border)
    defend_to_area        = defend.to(area)
    defend_to_locked      = defend.to(locked)

    do_spin_to_normal     = do_spin.to(normal)
    do_spin_to_defend     = do_spin.to(defend)
    do_spin_to_wait_ball  = do_spin.to(wait_ball)
    do_spin_to_move       = do_spin.to(move)
    do_spin_to_border     = do_spin.to(border)
    do_spin_to_area       = do_spin.to(area)
    do_spin_to_locked     = do_spin.to(locked)

    move_to_defend        = move.to(defend)
    move_to_normal        = move.to(normal)
    move_to_wait_ball     = move.to(wait_ball)
    move_to_do_spin       = move.to(do_spin)
    move_to_border        = move.to(border)
    move_to_area          = move.to(area)
    move_to_locked        = move.to(locked)

    wait_ball_to_defend   = wait_ball.to(defend)
    wait_ball_to_normal   = wait_ball.to(normal)
    wait_ball_to_move     = wait_ball.to(move)
    wait_ball_to_do_spin  = wait_ball.to(do_spin)
    wait_ball_to_border   = wait_ball.to(border)
    wait_ball_to_area     = wait_ball.to(area)
    wait_ball_to_locked   = wait_ball.to(locked)

    border_to_normal      = border.to(normal)
    border_to_defend      = border.to(defend)
    border_to_move        = border.to(move)
    border_to_do_spin     = border.to(do_spin)
    border_to_wait_ball   = border.to(normal)
    border_to_area        = border.to(area)
    border_to_locked      = border.to(locked)

    area_to_normal        = area.to(normal)
    area_to_defend        = area.to(defend)
    area_to_move          = area.to(move)
    area_to_do_spin       = area.to(do_spin)
    area_to_wait_ball     = area.to(normal)
    area_to_wait_locked   = area.to(locked)

    locked_to_normal      = locked.to(normal)
    locked_to_defend      = locked.to(defend)
    locked_to_move        = locked.to(move)
    locked_to_do_spin     = locked.to(do_spin)
    locked_to_wait_ball   = locked.to(normal)
    locked_to_wait_area   = locked.to(area)


class MyModel(object):
    def __init__(self, state):
        self.state = state
