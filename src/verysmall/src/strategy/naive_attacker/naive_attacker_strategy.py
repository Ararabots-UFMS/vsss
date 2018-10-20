from statemachine import StateMachine, State

class NaiveAttacker(StateMachine):
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
    go_to_point = State('Go_To_Point')
    wait_ball = State('Wait_Ball')
    reach_ball  = State('Reach_Ball')
    spin        = State('Spin')
    border      = State('Border')
    walk_border = State('Walk')


    # Stop to freeball game
    stop_to_freeball = stop.to(freeball)
    # Stop to normal game
    stop_to_normal = stop.to(normal)
    # Stop to penalty
    stop_to_penalty = stop.to(penalty)
    # Freeball to normal game
    freeball_to_normal = freeball.to(normal)
    # Penalty to normal game
    penalty_to_normal = penalty.to(normal)
    # non deterministic transition
    go = stop_to_freeball | stop_to_normal | stop_to_penalty | freeball_to_normal | penalty_to_normal
    # Normal to go to point
    normal_to_point = normal.to(go_to_point)
    # Normal to Border
    normal_to_border = normal.to(border)
    # Border to normal
    border_to_normal = border.to(normal)
    # border to go to point
    border_to_point = border.to(go_to_point)
    #border to walk border
    border_to_walk_border = border.to(walk_border)
    #border to reach ball
    border_to_reach_ball = border.to(reach_ball)
    # Normal to reach ball
    normal_to_reach_ball = normal.to(reach_ball)
    # Point to follow ball
    point_to_wait_ball = go_to_point.to(wait_ball)
    # Point to reach ball
    point_to_reach_ball = go_to_point.to(reach_ball)
    # Follow ball to stop
    wait_ball_to_stop = wait_ball.to(stop)
    # Follow to Reach Ball
    wait_to_reach_ball = wait_ball.to(reach_ball)
    # Reach ball to spin
    reach_ball_to_spin = reach_ball.to(spin)
    # Spin to normalS
    spin_to_normal = spin.to(normal)
    #spin to reach ball
    spin_to_reach_ball = spin.to(reach_ball)
    #walk border to normal
    walk_border_to_normal = walk_border.to(normal)
    #walk border to point
    walk_border_to_point = walk_border.to(go_to_point)
    #reach ball to point
    reach_ball_to_point = reach_ball.to(go_to_point)
    #reach ball to border
    reach_ball_to_border = reach_ball.to(border)


class MyModel(object):
    def __init__(self, state):
        self.state = state
